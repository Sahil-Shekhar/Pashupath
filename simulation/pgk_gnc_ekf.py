"""
=============================================================================
YIL-PGK-155  Guidance, Navigation and Control (GNC) System
Extended Kalman Filter + Proportional Navigation Guidance
=============================================================================
Project:    Precision Guidance Kit (PGK), 155 mm Artillery Shell
Author:     Systems Engineering, Yantra India Limited (YIL)
Revision:   1.1
Date:       2026-09-23
Ref:        YIL-PGK-155-GNC-ICD-001

ARCHITECTURE
  Navigation : 16-state strapdown INS / EKF
               (position, velocity, quaternion attitude, IMU biases)
  Guidance   : Proportional Navigation Guidance (PNG), N = 4.5
  Control    : Pitch/yaw PID  →  4-canard X-config allocation
  Frame      : NED (North-East-Down), origin at gun muzzle

STATE VECTOR  x ∈ ℝ¹⁶
  x[ 0: 3]  position  p_NED       [m]
  x[ 3: 6]  velocity  v_NED       [m/s]
  x[ 6:10]  attitude  q [w,x,y,z] [-]
  x[10:13]  accel bias b_a         [m/s²]
  x[13:16]  gyro  bias b_g         [rad/s]

DEPENDENCIES
  numpy >= 1.24
  (scipy optional — RK4 implemented inline)

USAGE
  python pgk_gnc_ekf.py          # standalone simulation
  from pgk_gnc_ekf import PGK_GNC, MissionConfig   # module import
=============================================================================
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional
import logging
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
log = logging.getLogger("PGK-GNC")


# ============================================================================
# 1. CONFIGURATION DATACLASSES
# ============================================================================

@dataclass
class IMUConfig:
    """ADIS16460 noise parameters (1-sigma continuous-time densities)."""
    # Accelerometer
    accel_noise_density:  float = 0.018e-3    # m/s²/√Hz  (18 µg/√Hz)
    accel_bias_instability: float = 0.004e-3  # m/s²
    accel_bias_rw:        float = 1.0e-6      # m/s²/√s
    accel_range:          float = 19620.0     # m/s²  (±2000 g)
    # Gyroscope
    gyro_noise_density:   float = field(default_factory=lambda: np.deg2rad(0.018))
    gyro_bias_instability: float = field(default_factory=lambda: np.deg2rad(0.0024))
    gyro_bias_rw:         float = field(default_factory=lambda: np.deg2rad(1.0e-4))
    gyro_range:           float = field(default_factory=lambda: np.deg2rad(2000.0))
    sample_rate:          float = 2000.0      # Hz

    def __post_init__(self):
        # allow numpy deg2rad to run at instance time when defaults used
        if isinstance(self.gyro_noise_density, float) and self.gyro_noise_density > 0.1:
            self.gyro_noise_density   = np.deg2rad(0.018)
            self.gyro_bias_instability = np.deg2rad(0.0024)
            self.gyro_bias_rw         = np.deg2rad(1.0e-4)
            self.gyro_range           = np.deg2rad(2000.0)


@dataclass
class GNSSConfig:
    """u-blox F9P noise parameters."""
    pos_noise_h:    float = 1.5     # m  1σ horizontal
    pos_noise_v:    float = 2.5     # m  1σ vertical
    vel_noise:      float = 0.05    # m/s 1σ all axes
    update_rate:    float = 10.0    # Hz
    t_available:    float = 45.0    # s post-muzzle (cold-start acquisition)
    hdop_threshold: float = 2.0


@dataclass
class GuidanceConfig:
    """PNG parameters."""
    nav_constant:    float = 4.5    # N  (dimensionless)
    max_lat_accel:   float = 150.0  # m/s²  (~15 g lateral)
    min_closing_vel: float = 50.0   # m/s
    los_rate_tau:    float = 0.05   # s  (LOS rate filter time constant)


@dataclass
class ControlConfig:
    """Canard servo PID gains and limits."""
    Kp: float = 2.5
    Ki: float = 0.8
    Kd: float = 0.05
    max_deflection:      float = field(default_factory=lambda: np.deg2rad(15.0))
    max_deflection_rate: float = field(default_factory=lambda: np.deg2rad(120.0))
    max_trim_deflection: float = field(default_factory=lambda: np.deg2rad(2.0))
    eta_canard:          float = 0.032   # m/rad  (linearised aero effectiveness)

    def __post_init__(self):
        if self.max_deflection > 1.0:      # degrees were passed
            self.max_deflection      = np.deg2rad(15.0)
            self.max_deflection_rate = np.deg2rad(120.0)
            self.max_trim_deflection = np.deg2rad(2.0)


@dataclass
class MissionConfig:
    """Complete mission configuration."""
    target_ned:    np.ndarray = field(
        default_factory=lambda: np.array([28_000.0, 200.0, -50.0])
    )
    muzzle_vel:    float = 827.0   # m/s
    elevation_deg: float = 45.0   # deg
    azimuth_deg:   float = 0.0    # deg  (True North)
    t_max:         float = 120.0  # s   flight duration limit
    ekf_dt:        float = 0.01   # s   100 Hz EKF outer loop
    servo_dt:      float = 0.002  # s   500 Hz servo inner loop
    imu:      IMUConfig     = field(default_factory=IMUConfig)
    gnss:     GNSSConfig    = field(default_factory=GNSSConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    control:  ControlConfig  = field(default_factory=ControlConfig)


# ============================================================================
# 2. QUATERNION UTILITIES  (scalar-first: [w, x, y, z])
# ============================================================================

class Quaternion:
    """Quaternion algebra helpers — all operate on plain numpy arrays."""

    @staticmethod
    def identity() -> np.ndarray:
        return np.array([1.0, 0.0, 0.0, 0.0])

    @staticmethod
    def normalize(q: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(q)
        return q / n if n > 1e-12 else Quaternion.identity()

    @staticmethod
    def multiply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
        """Hamilton product p ⊗ q."""
        pw, px, py, pz = p
        qw, qx, qy, qz = q
        return np.array([
            pw*qw - px*qx - py*qy - pz*qz,
            pw*qx + px*qw + py*qz - pz*qy,
            pw*qy - px*qz + py*qw + pz*qx,
            pw*qz + px*qy - py*qx + pz*qw,
        ])

    @staticmethod
    def conjugate(q: np.ndarray) -> np.ndarray:
        return q * np.array([1.0, -1.0, -1.0, -1.0])

    @staticmethod
    def rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """v' = q ⊗ [0,v] ⊗ q*  →  rotated 3-vector."""
        qv = np.concatenate([[0.0], v])
        return Quaternion.multiply(
            Quaternion.multiply(q, qv), Quaternion.conjugate(q)
        )[1:]

    @staticmethod
    def to_dcm(q: np.ndarray) -> np.ndarray:
        """Direction cosine matrix body → NED from quaternion."""
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ])

    @staticmethod
    def qdot(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
        """q̇ = 0.5 Ω(ω) q  — quaternion kinematic equation."""
        wx, wy, wz = omega_body
        Omega = np.array([
            [  0, -wx, -wy, -wz],
            [ wx,   0,  wz, -wy],
            [ wy, -wz,   0,  wx],
            [ wz,  wy, -wx,   0],
        ])
        return 0.5 * Omega @ q

    @staticmethod
    def Xi(q: np.ndarray) -> np.ndarray:
        """4×3 matrix Ξ(q) such that q̇ = 0.5 Ξ(q) ω."""
        w, x, y, z = q
        return np.array([
            [-x, -y, -z],
            [ w, -z,  y],
            [ z,  w, -x],
            [-y,  x,  w],
        ])

    @staticmethod
    def from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        return np.array([np.cos(angle/2), *(np.sin(angle/2) * axis)])


# ============================================================================
# 3. 16-STATE EXTENDED KALMAN FILTER
# ============================================================================

class InertialEKF:
    """
    16-State Strapdown INS / EKF.

    Predict : RK4 state propagation + linearised covariance (F P Fᵀ + Q)
    Update  : GNSS position + velocity (linear H, Joseph-form covariance)

    Reference: Groves, "Principles of GNSS, Inertial and Multisensor
               Integrated Navigation Systems," 2nd ed., Artech House, 2013.
    """
    NX = 16   # state dimension
    NZ = 6    # GNSS measurement dimension

    def __init__(self, cfg: MissionConfig):
        self.cfg  = cfg
        self.ic   = cfg.imu
        self.gc   = cfg.gnss
        self.dt   = cfg.ekf_dt
        self.g    = np.array([0.0, 0.0, 9.80665])   # NED gravity

        self.x = np.zeros(self.NX); self.x[6] = 1.0   # identity quat
        self.P = np.eye(self.NX) * 1e-6
        self.gnss_fusing = False

        self._build_Q()
        self._build_R()
        log.info("EKF initialised  NX=%d  dt=%.4f s", self.NX, self.dt)

    # ---------- noise matrices ----------

    def _build_Q(self):
        ic = self.ic; dt = self.dt
        qp = 1e-8
        qv = ic.accel_noise_density**2 / dt
        qq = ic.gyro_noise_density**2  / dt
        qb = ic.accel_bias_rw**2
        qg = ic.gyro_bias_rw**2
        self.Q = np.diag([
            qp, qp, qp,
            qv, qv, qv,
            qq, qq, qq, qq,
            qb, qb, qb,
            qg, qg, qg,
        ]) * dt

    def _build_R(self):
        gc = self.gc
        self.R = np.diag([
            gc.pos_noise_h**2, gc.pos_noise_h**2, gc.pos_noise_v**2,
            gc.vel_noise**2,   gc.vel_noise**2,   gc.vel_noise**2,
        ])

    # ---------- initialisation ----------

    def initialize(self, pos: np.ndarray, vel: np.ndarray,
                   quat: np.ndarray,
                   ba: Optional[np.ndarray] = None,
                   bg: Optional[np.ndarray] = None):
        self.x[0:3]  = pos
        self.x[3:6]  = vel
        self.x[6:10] = Quaternion.normalize(quat)
        self.x[10:13] = ba if ba is not None else np.zeros(3)
        self.x[13:16] = bg if bg is not None else np.zeros(3)
        self.P = np.diag([
            5.0, 5.0, 10.0,
            1.0, 1.0,  1.0,
            1e-4, 1e-4, 1e-4, 1e-4,
            0.1, 0.1, 0.1,
            np.deg2rad(5)**2, np.deg2rad(5)**2, np.deg2rad(5)**2,
        ])
        log.info("EKF state init  pos=%.1f %.1f %.1f m  |vel|=%.1f m/s",
                 *pos, np.linalg.norm(vel))

    # ---------- predict ----------

    def _dynamics(self, x, f_b, w_b):
        """Continuous-time strapdown INS dynamics."""
        xd = np.zeros(self.NX)
        v  = x[3:6]; q  = x[6:10]
        ba = x[10:13]; bg = x[13:16]
        f  = f_b - ba
        w  = w_b - bg
        C  = Quaternion.to_dcm(q)
        xd[0:3]  = v
        xd[3:6]  = C @ f + self.g
        xd[6:10] = Quaternion.qdot(q, w)
        # biases modelled as constants (noise via Q)
        return xd

    def predict(self, f_body: np.ndarray, omega_body: np.ndarray):
        """EKF predict: RK4 state + linearised covariance."""
        dt = self.dt
        x  = self.x

        # RK4 propagation
        k1 = self._dynamics(x,       f_body, omega_body)
        k2 = self._dynamics(x+.5*dt*k1, f_body, omega_body)
        k3 = self._dynamics(x+.5*dt*k2, f_body, omega_body)
        k4 = self._dynamics(x+   dt*k3, f_body, omega_body)
        xn = x + dt/6*(k1 + 2*k2 + 2*k3 + k4)
        xn[6:10] = Quaternion.normalize(xn[6:10])
        self.x   = xn

        # Jacobian F  (16×16, linearised at current state)
        F = self._jacobian(x, f_body, omega_body)
        Fd = np.eye(self.NX) + F * dt           # ZOH first-order
        self.P = Fd @ self.P @ Fd.T + self.Q
        self.P = 0.5*(self.P + self.P.T)        # symmetrize

    def _jacobian(self, x, f_body, omega_body):
        """Analytic 16×16 Jacobian of the strapdown INS equations."""
        F  = np.zeros((self.NX, self.NX))
        q  = x[6:10]; ba = x[10:13]; bg = x[13:16]
        fc = f_body - ba
        wc = omega_body - bg
        C  = Quaternion.to_dcm(q)

        # dp/dv
        F[0:3, 3:6] = np.eye(3)

        # dv/dq  — 3×4 Jacobian of (C_bn @ fc) w.r.t. q
        F[3:6, 6:10] = self._J_dcm_q(q, fc)

        # dv/d_ba
        F[3:6, 10:13] = -C

        # dq/dq  (skew-omega term)
        wx, wy, wz = wc
        F[6:10, 6:10] = 0.5*np.array([
            [  0, -wx, -wy, -wz],
            [ wx,   0,  wz, -wy],
            [ wy, -wz,   0,  wx],
            [ wz,  wy, -wx,   0],
        ])

        # dq/d_bg
        F[6:10, 13:16] = -0.5 * Quaternion.Xi(q)
        return F

    @staticmethod
    def _J_dcm_q(q, v):
        """
        3×4 Jacobian: ∂(C_bn @ v)/∂q  evaluated analytically.
        Derivation: Groves (2013) App. A, eq. A.55.
        """
        w, x, y, z = q
        vx, vy, vz = v
        J = 2.0 * np.array([
            [ y*vy + z*vz,
              y*vx - 2*x*vy + w*vz,
              z*vx - w*vy - 2*x*vz,
             -y*vz + z*vy          ],
            [-z*vz + x*vy - 2*y*vx,
              x*vx + z*vz,
              z*vy - 2*y*vz + w*vx,
             -z*vx - w*vz + x*vz  ],
            [ w*vy - 2*z*vx + x*vz,
             -w*vx - 2*z*vy + y*vz,
              x*vx + y*vy,
              x*vy - y*vx          ],
        ])
        return J   # 3×4

    # ---------- GNSS update ----------

    def update_gnss(self, p_gnss: np.ndarray, v_gnss: np.ndarray):
        """
        Linear GNSS measurement update.
        H = [I₃  0  0  0  0; 0  I₃  0  0  0]  (6×16)
        Joseph-form covariance for numerical symmetry.
        """
        H = np.zeros((6, self.NX))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)

        z  = np.concatenate([p_gnss, v_gnss])
        nu = z - np.concatenate([self.x[0:3], self.x[3:6]])

        S  = H @ self.P @ H.T + self.R
        K  = self.P @ H.T @ np.linalg.inv(S)

        self.x   = self.x + K @ nu
        self.x[6:10] = Quaternion.normalize(self.x[6:10])

        IKH = np.eye(self.NX) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T
        self.P = 0.5*(self.P + self.P.T)

        self.gnss_fusing = True

    # ---------- accessors ----------

    @property
    def pos_ned(self)  -> np.ndarray: return self.x[0:3].copy()
    @property
    def vel_ned(self)  -> np.ndarray: return self.x[3:6].copy()
    @property
    def quat(self)     -> np.ndarray: return self.x[6:10].copy()
    @property
    def accel_bias(self) -> np.ndarray: return self.x[10:13].copy()
    @property
    def gyro_bias(self)  -> np.ndarray: return self.x[13:16].copy()

    @property
    def euler_deg(self) -> np.ndarray:
        w, x, y, z = self.quat
        roll  = np.rad2deg(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
        pitch = np.rad2deg(np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0)))
        yaw   = np.rad2deg(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
        return np.array([roll, pitch, yaw])

    @property
    def sigma_pos(self) -> np.ndarray:
        return np.sqrt(np.maximum(0, np.diag(self.P)[0:3]))

    @property
    def sigma_vel(self) -> np.ndarray:
        return np.sqrt(np.maximum(0, np.diag(self.P)[3:6]))


# ============================================================================
# 4. PROPORTIONAL NAVIGATION GUIDANCE (PNG)
# ============================================================================

class PNG_Guidance:
    """
    Pure Proportional Navigation Guidance.

                a_cmd = N · Vc · λ̇

    N  = navigation constant     (4.5)
    Vc = closing speed           (m/s, positive when approaching target)
    λ̇  = LOS angular rate vector (rad/s, 3-D)

    The cross-product form λ̇ = û_los × dû_los/dt gives a vector
    perpendicular to the LOS — acceleration is commanded in this plane.

    Reference: Zarchan, "Tactical and Strategic Missile Guidance,"
               6th ed., AIAA, 2012.  §3.3 and §7.1.
    """

    def __init__(self, cfg: GuidanceConfig):
        self.N       = cfg.nav_constant
        self.a_max   = cfg.max_lat_accel
        self.Vc_min  = cfg.min_closing_vel
        self.tau     = cfg.los_rate_tau

        self._los_rate_filt = np.zeros(3)
        self._los_prev      = None
        self._t_prev        = None

    def compute(self,
                pos_ned: np.ndarray,
                vel_ned: np.ndarray,
                tgt_ned: np.ndarray,
                t: float) -> np.ndarray:
        """
        Returns commanded acceleration in NED frame [m/s²].
        Zero until first two calls (need two LOS samples for rate).
        """
        r_vec = tgt_ned - pos_ned
        R     = np.linalg.norm(r_vec)
        if R < 1.0:
            return np.zeros(3)

        los  = r_vec / R
        Vc   = -float(np.dot(vel_ned, los))   # closing speed

        if Vc < self.Vc_min or self._t_prev is None:
            self._los_prev = los.copy()
            self._t_prev   = t
            return np.zeros(3)

        dt = t - self._t_prev
        if dt < 1e-6:
            return np.zeros(3)

        # Raw LOS angular rate (cross-product form)
        dlos         = (los - self._los_prev) / dt
        los_rate_raw = np.cross(los, dlos)

        # First-order low-pass filter
        alpha = dt / (self.tau + dt)
        self._los_rate_filt += alpha * (los_rate_raw - self._los_rate_filt)

        self._los_prev = los.copy()
        self._t_prev   = t

        # PNG command
        a_cmd = self.N * Vc * self._los_rate_filt

        # Saturate
        a_mag = np.linalg.norm(a_cmd)
        if a_mag > self.a_max:
            a_cmd *= self.a_max / a_mag

        return a_cmd

    def time_to_go(self, pos_ned, vel_ned, tgt_ned) -> float:
        r  = np.linalg.norm(tgt_ned - pos_ned)
        u  = (tgt_ned - pos_ned) / (r + 1e-9)
        Vc = -float(np.dot(vel_ned, u))
        return r / max(Vc, 1.0)


# ============================================================================
# 5. CANARD CONTROL ALLOCATION & PID SERVO
# ============================================================================

class CanardController:
    """
    Maps commanded lateral acceleration → 4 canard deflection angles.

    X-configuration (looking aft):
        C1: 0°   top    → +pitch
        C2: 90°  right  → +yaw
        C3: 180° bottom → −pitch
        C4: 270° left   → −yaw

    Inverse mixing (pseudo-inverse of allocation matrix):
        C1 = +δ_pitch       C3 = −δ_pitch
        C2 = +δ_yaw         C4 = −δ_yaw

    PID error is acceleration error [m/s²]; output converted to canard
    angle via linearised effectiveness η [m/rad]:
        a = η · δ   →   δ = a / η
    """

    def __init__(self, cfg: MissionConfig):
        self.ctrl = cfg.control
        self.dt   = cfg.servo_dt
        # PID integrators & previous errors
        self._ip  = self._iy  = 0.0
        self._ep  = self._ey  = 0.0
        self.delta_prev = np.zeros(4)

    def compute(self, a_cmd_body: np.ndarray,
                a_meas_body: np.ndarray) -> np.ndarray:
        """
        a_cmd_body[0] = commanded pitch accel (body +y)  [m/s²]
        a_cmd_body[1] = commanded yaw   accel (body +z)  [m/s²]
        Returns canard angles [C1,C2,C3,C4] in radians.
        """
        c    = self.ctrl
        dt   = self.dt
        eta  = c.eta_canard

        def _pid(err, integ, e_prev):
            integ = np.clip(integ + err*dt, -10.0, 10.0)
            de    = (err - e_prev) / dt
            out   = c.Kp*err + c.Ki*integ + c.Kd*de
            return out, integ, err

        ep = a_cmd_body[0] - a_meas_body[1]   # pitch error
        ey = a_cmd_body[1] - a_meas_body[2]   # yaw error

        pid_p, self._ip, self._ep = _pid(ep, self._ip, self._ep)
        pid_y, self._iy, self._ey = _pid(ey, self._iy, self._ey)

        dp = np.clip(pid_p / eta, -c.max_deflection, c.max_deflection)
        dy = np.clip(pid_y / eta, -c.max_deflection, c.max_deflection)

        delta_cmd = np.array([dp, dy, -dp, -dy])

        # Rate limit
        rate = (delta_cmd - self.delta_prev) / dt
        rate = np.clip(rate, -c.max_deflection_rate, c.max_deflection_rate)
        delta = self.delta_prev + rate * dt
        self.delta_prev = delta
        return delta

    @staticmethod
    def ned_to_body(a_ned: np.ndarray, q: np.ndarray) -> np.ndarray:
        """Rotate NED vector to body frame via C_nb = C_bn^T."""
        return Quaternion.to_dcm(q).T @ a_ned


# ============================================================================
# 6. GNC DATA STRUCTURES
# ============================================================================

@dataclass
class IMUMeasurement:
    accel: np.ndarray   # specific force, body frame [m/s²]
    gyro:  np.ndarray   # angular rate,   body frame [rad/s]


@dataclass
class GNSSMeasurement:
    pos_ned: np.ndarray
    vel_ned: np.ndarray
    valid:   bool  = True
    hdop:    float = 1.0


@dataclass
class GNCOutput:
    t:             float
    pos_ned:       np.ndarray
    vel_ned:       np.ndarray
    quat:          np.ndarray
    euler_deg:     np.ndarray
    a_cmd_ned:     np.ndarray
    delta_canards_deg: np.ndarray   # [C1, C2, C3, C4] degrees
    tgo:           float
    range_to_go:   float
    gnss_active:   bool
    sigma_pos:     np.ndarray
    sigma_vel:     np.ndarray


# ============================================================================
# 7. TOP-LEVEL GNC INTEGRATOR
# ============================================================================

class PGK_GNC:
    """
    Top-level GNC integrator.

    Execution rates:
      EKF predict    100 Hz  (ekf_dt = 0.01 s)
      GNSS update     10 Hz  (every 10th predict cycle)
      PNG guidance   100 Hz
      Canard servo   500 Hz  (5 servo cycles per EKF cycle, here computed once)
    """

    def __init__(self, cfg: MissionConfig):
        self.cfg  = cfg
        self.ekf  = InertialEKF(cfg)
        self.png  = PNG_Guidance(cfg.guidance)
        self.ctrl = CanardController(cfg)
        self._init = False

    def initialize(self, t0: float = 0.0):
        """Initialise from muzzle conditions in MissionConfig."""
        cfg = self.cfg
        el  = np.deg2rad(cfg.elevation_deg)
        az  = np.deg2rad(cfg.azimuth_deg)
        v0  = cfg.muzzle_vel

        vel_ned = np.array([
             v0 * np.cos(el) * np.cos(az),
             v0 * np.cos(el) * np.sin(az),
            -v0 * np.sin(el),
        ])

        # Initial attitude: body x aligned with velocity (nose forward)
        q_pitch = Quaternion.from_axis_angle(np.array([0.,1.,0.]), -el)
        q_yaw   = Quaternion.from_axis_angle(np.array([0.,0.,1.]),  az)
        q_init  = Quaternion.normalize(Quaternion.multiply(q_yaw, q_pitch))

        self.ekf.initialize(np.zeros(3), vel_ned, q_init)
        self._init = True
        log.info("PGK_GNC ready  V0=%.0f m/s  El=%.1f°  Az=%.1f°",
                 v0, cfg.elevation_deg, cfg.azimuth_deg)

    def step(self,
             imu:  IMUMeasurement,
             gnss: Optional[GNSSMeasurement],
             t:    float) -> GNCOutput:
        assert self._init, "Call initialize() first"

        # 1. EKF predict
        self.ekf.predict(imu.accel, imu.gyro)

        # 2. GNSS update (when valid and available)
        gc = self.cfg.gnss
        if (gnss is not None and gnss.valid
                and t >= gc.t_available
                and gnss.hdop <= gc.hdop_threshold):
            self.ekf.update_gnss(gnss.pos_ned, gnss.vel_ned)

        # 3. State readout
        pos = self.ekf.pos_ned
        vel = self.ekf.vel_ned
        q   = self.ekf.quat

        # 4. PNG guidance
        a_cmd_ned  = self.png.compute(pos, vel, self.cfg.target_ned, t)

        # 5. Body-frame command & canard allocation
        a_cmd_body = CanardController.ned_to_body(a_cmd_ned, q)
        a_meas_b   = imu.accel - self.ekf.accel_bias
        deltas_rad = self.ctrl.compute(a_cmd_body[[1, 2]], a_meas_b)

        return GNCOutput(
            t                 = t,
            pos_ned           = pos,
            vel_ned           = vel,
            quat              = q,
            euler_deg         = self.ekf.euler_deg,
            a_cmd_ned         = a_cmd_ned,
            delta_canards_deg = np.rad2deg(deltas_rad),
            tgo               = self.png.time_to_go(pos, vel, self.cfg.target_ned),
            range_to_go       = float(np.linalg.norm(self.cfg.target_ned - pos)),
            gnss_active       = self.ekf.gnss_fusing,
            sigma_pos         = self.ekf.sigma_pos,
            sigma_vel         = self.ekf.sigma_vel,
        )


# ============================================================================
# 8. SIMULATION DRIVER  (simplified Newtonian truth model for Phase 4 test)
#    Full 6-DOF Monte Carlo is Phase 5.
# ============================================================================

def simulate_gnc(cfg: MissionConfig,
                 seed: int = 42) -> list:
    """
    Drive GNC through a noisy ballistic trajectory.
    Truth model: Newtonian gravity + Mach-dependent drag.
    Returns list of GNCOutput records (one per 100 Hz GNC cycle).
    """
    rng  = np.random.default_rng(seed)
    gnc  = PGK_GNC(cfg)
    gnc.initialize(t0=0.0)

    # Truth state
    el = np.deg2rad(cfg.elevation_deg)
    az = np.deg2rad(cfg.azimuth_deg)
    v0 = cfg.muzzle_vel
    pos_t = np.zeros(3)
    vel_t = np.array([
         v0*np.cos(el)*np.cos(az),
         v0*np.cos(el)*np.sin(az),
        -v0*np.sin(el),
    ])

    g_ned   = np.array([0., 0., 9.80665])
    mass    = 47.0                          # kg
    area    = np.pi * (0.155/2)**2          # m²
    rho_sl  = 1.225

    def Cd(M):
        return 0.45 - 0.05*M + 0.02*M**2 if M < 3.0 else 0.42

    dt       = cfg.ekf_dt
    t        = 0.0
    a_guid   = np.zeros(3)   # guidance accel fed back into truth
    history  = []
    gnss_cnt = 0

    log.info("Simulation start  target N=%.0f m  E=%.0f m",
             cfg.target_ned[0], cfg.target_ned[1])
    t_wall = time.time()

    while t <= cfg.t_max:
        alt = -pos_t[2]                     # altitude AGL [m]
        if alt < -5.0 and t > 1.0:
            log.info("Impact T=%.2f s  pos N=%.1f E=%.1f D=%.1f m",
                     t, *pos_t)
            break

        # Atmosphere
        rho  = rho_sl * np.exp(-alt / 8500.0)
        vmag = np.linalg.norm(vel_t) + 1e-9
        Mach = vmag / 340.0
        qdyn = 0.5 * rho * vmag**2

        # Drag (opposes velocity)
        F_drag = -(Cd(Mach) * qdyn * area / mass) * (vel_t / vmag)

        # Truth accel
        a_true = g_ned + F_drag + a_guid

        # IMU simulation: specific force = a_true − g (accelerometer output)
        a_imu  = a_true - g_ned + rng.normal(
            0, cfg.imu.accel_noise_density * np.sqrt(cfg.imu.sample_rate), 3)
        w_imu  = rng.normal(
            0, cfg.imu.gyro_noise_density  * np.sqrt(cfg.imu.sample_rate), 3)
        imu = IMUMeasurement(accel=a_imu, gyro=w_imu)

        # GNSS simulation (10 Hz when available)
        gnss = None
        gnss_cnt += 1
        if t >= cfg.gnss.t_available and gnss_cnt >= 10:
            gnss_cnt = 0
            pn = rng.normal(0, [cfg.gnss.pos_noise_h,
                                 cfg.gnss.pos_noise_h,
                                 cfg.gnss.pos_noise_v])
            vn = rng.normal(0, cfg.gnss.vel_noise, 3)
            gnss = GNSSMeasurement(
                pos_ned=pos_t+pn, vel_ned=vel_t+vn,
                valid=True, hdop=1.2)

        # GNC step
        out = gnc.step(imu, gnss, t)
        history.append(out)

        # Feed guidance acceleration back into truth
        a_guid = out.a_cmd_ned.copy()

        # Integrate truth
        pos_t += vel_t * dt + 0.5 * a_true * dt**2
        vel_t += a_true * dt
        t      = round(t + dt, 6)

    elapsed = time.time() - t_wall
    n = len(history)
    log.info("Simulation done  %d steps in %.3f s  (%.1f× real-time)",
             n, elapsed, n*dt/elapsed)

    if history:
        f    = history[-1]
        miss = np.linalg.norm(f.pos_ned[:2] - cfg.target_ned[:2])
        log.info("Miss distance (2-D horizontal): %.2f m", miss)
        log.info("Final σ_pos  N=%.2f m  E=%.2f m  D=%.2f m", *f.sigma_pos)

    return history


def print_telemetry(history: list, cfg: MissionConfig, interval_s: float = 10.0):
    """Print a formatted telemetry table."""
    dt   = cfg.ekf_dt
    skip = max(1, int(interval_s / dt))

    hdr  = (f"{'T(s)':>6}  {'N(m)':>8} {'E(m)':>8} {'D(m)':>8}  "
            f"{'Spd':>6}  {'aCN':>7} {'aCE':>7}  "
            f"{'dC1(d)':>7} {'dC2(d)':>7}  "
            f"{'Rng(m)':>8} {'Tgo(s)':>7}  {'GNSS':>4}")

    print("\n" + "="*90)
    print("  YIL-PGK-155  GNC TELEMETRY")
    print(f"  Target: N={cfg.target_ned[0]:.0f} m  E={cfg.target_ned[1]:.0f} m")
    print("="*90)
    print(hdr)
    print("-"*90)

    for i, o in enumerate(history):
        if i % skip == 0:
            spd = np.linalg.norm(o.vel_ned)
            print(f"{o.t:6.1f}  "
                  f"{o.pos_ned[0]:8.1f} {o.pos_ned[1]:8.1f} {o.pos_ned[2]:8.1f}  "
                  f"{spd:6.1f}  "
                  f"{o.a_cmd_ned[0]:7.2f} {o.a_cmd_ned[1]:7.2f}  "
                  f"{o.delta_canards_deg[0]:6.2f} {o.delta_canards_deg[1]:6.2f}  "
                  f"{o.range_to_go:8.0f} {o.tgo:7.1f}  "
                  f"{'YES' if o.gnss_active else 'NO':>4}")

    print("="*90)
    if history:
        f    = history[-1]
        miss = np.linalg.norm(f.pos_ned[:2] - cfg.target_ned[:2])
        print(f"\n  ► FINAL 2-D MISS DISTANCE : {miss:.2f} m")
        print(f"  ► Position 1-σ            : "
              f"N={f.sigma_pos[0]:.2f} m  E={f.sigma_pos[1]:.2f} m  "
              f"D={f.sigma_pos[2]:.2f} m")
        print("="*90 + "\n")


# ============================================================================
# MAIN  -  Phase 4 validation run
# ============================================================================

if __name__ == "__main__":
    import sys
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    print("\n" + "="*60)
    print("  YIL-PGK-155  GNC System  -  Phase 4 Validation Run")
    print("="*60 + "\n")

    cfg = MissionConfig(
        target_ned    = np.array([28_000.0, 150.0, 0.0]),
        muzzle_vel    = 827.0,
        elevation_deg = 45.0,
        azimuth_deg   = 0.5,   # slight azimuth offset to exercise yaw PNG
    )
    # Shorten GNSS acquisition for single-run validation
    # (Phase 5 full Monte Carlo uses the default 45 s cold-start)
    cfg.gnss.t_available      = 20.0
    cfg.guidance.los_rate_tau = 0.03   # tighter LOS filter; 0.05 s used in MC

    history = simulate_gnc(cfg, seed=42)
    print_telemetry(history, cfg, interval_s=10.0)
