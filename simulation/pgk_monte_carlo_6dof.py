"""
=============================================================================
YIL-PGK-155  Phase 5: 7-State Point Mass Trajectory & Monte Carlo
=============================================================================
Project  : Precision Guidance Kit (PGK), 155 mm Artillery Shell
Author   : Trajectory & Ballistics, Yantra India Limited (YIL)
Revision : 1.3   Date: 2026-09-25
Ref      : YIL-PGK-155-TRAJ-001

KEY FIXES vs Rev 1.2:
  * Elevation angle auto-calibrated via bisection so that the nominal
    zero-dispersion trajectory lands exactly on target (28,000 m).
    (At Charge 8 / 827 m/s with Cd model, this is ~53 deg, not 45 deg.)
  * PNG guidance then only corrects the small dispersion errors (~200 m
    for unguided) rather than trying to cover a 5.6 km systematic miss.

MODEL     : 7-state point mass in NED frame.
            Forces: drag (Mach-Cd), gravity, Coriolis, Magnus drift, canard.
            Spin state tracked for Magnus + spin-decay.

USAGE     : python pgk_monte_carlo_6dof.py
            python pgk_monte_carlo_6dof.py --N 200   # quick demo
=============================================================================
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple
from concurrent.futures import ProcessPoolExecutor, as_completed
import time, sys, os, logging, argparse

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("PGK")

# ============================================================================
# CONSTANTS
# ============================================================================
G0       = 9.80665
R_GAS    = 287.058
GAMMA    = 1.4
OMEGA_E  = 7.292115e-5        # Earth rotation [rad/s]
LAT      = np.deg2rad(23.0)   # Rajasthan, India

# ============================================================================
# ATMOSPHERE  (ISA 1976)
# ============================================================================
class Atmosphere:
    _layers = [
        (0,     -6.5e-3, 288.15, 101325.0),
        (11000,  0.0,    216.65,  22632.1),
        (20000,  1.0e-3, 216.65,   5474.89),
        (32000,  2.8e-3, 228.65,    868.02),
        (47000,  0.0,    270.65,    110.91),
        (51000, -2.8e-3, 270.65,     66.94),
        (71000, -2.0e-3, 214.65,      3.956),
    ]
    def at_altitude(self, h):
        h = float(np.clip(h, 0.0, 86e3))
        lyr = self._layers[0]
        for L in self._layers[1:]:
            if h >= L[0]: lyr = L
            else: break
        h0,L,T0,P0 = lyr
        T = max(T0+L*(h-h0), 50.0)
        P = P0*np.exp(-G0*(h-h0)/(R_GAS*T0)) if abs(L)<1e-10 \
            else P0*(T/T0)**(-G0/(L*R_GAS))
        rho = P/(R_GAS*T)
        Vs  = (GAMMA*R_GAS*T)**0.5
        return rho, T, Vs

_atm = Atmosphere()

# ============================================================================
# AERODYNAMIC COEFFICIENTS  (155 mm M795/M549A1 class, BRL data)
# ============================================================================
class AeroModel:
    _M   = np.array([0.0,  0.3,  0.6,  0.8,  0.9,  0.95, 1.0,  1.05,
                     1.1,  1.2,  1.5,  2.0,  2.5,  3.0,  4.0])
    # Zero-lift drag (base), scaled by 0.70 to match US Army FT 155-AM-2:
    # V0=827 m/s, el~49 deg -> 28 km range.
    _Cd  = np.array([0.170,0.170,0.190,0.230,0.290,0.350,0.430,0.470,
                     0.440,0.380,0.300,0.260,0.240,0.230,0.220])
    _Cd_scale = 0.70
    # Spin damping
    _Cpd = np.array([0.018]*8+[0.017,0.016,0.015,0.013,0.012,0.011,0.010])
    # Magnus lateral force (calibrated: ~35 m rightward drift at 28 km)
    _CMg = np.array([0.0014,0.0014,0.0013,0.0012,0.0011,0.0011,0.0010,0.0010,
                     0.0009,0.0009,0.0008,0.0007,0.0006,0.0006,0.0005])
    def _i(self,t,M): return float(np.interp(M,self._M,t))
    def Cd(self,M):   return self._i(self._Cd,M)*self._Cd_scale
    def Cpd(self,M):  return self._i(self._Cpd,M)
    def CMg(self,M):  return self._i(self._CMg,M)

_aero = AeroModel()

# ============================================================================
# SHELL PROPERTIES
# ============================================================================
@dataclass
class Shell:
    mass: float = 43.2
    diam: float = 0.155
    Ixx:  float = 0.135      # axial MOI for spin decay
    area: float = field(init=False)
    def __post_init__(self): self.area = np.pi*(self.diam/2)**2

# ============================================================================
# EQUATIONS OF MOTION
# ============================================================================
def eom(s, shell, wind, F_can):
    """7-state PM EOM. s=[N,E,D, vN,vE,vD, p_spin]."""
    vel  = s[3:6]
    p    = float(s[6])
    alt  = float(np.clip(-s[2], 0.0, 86e3))
    rho,_,Vs = _atm.at_altitude(alt)
    v_air = vel - wind
    V  = float(np.linalg.norm(v_air))
    V_s = max(V, 0.5)
    M  = V_s / max(Vs, 100.0)

    Cd  = _aero.Cd(M)
    Cpd = _aero.Cpd(M)
    CMg = _aero.CMg(M)
    qd  = 0.5*rho*V_s**2

    # Drag  (opposes airspeed)
    Fd = -Cd*qd*shell.area*(v_air/V_s)

    # Gravity
    Fg = shell.mass*np.array([0.,0.,G0])

    # Coriolis
    Oe = OMEGA_E*np.array([np.cos(LAT),0.,-np.sin(LAT)])
    Fc = -2.*shell.mass*np.cross(Oe, vel)

    # Magnus lateral drift (right of heading for RH rifling, NH)
    # Validated direction: heading N -> drift E; heading E -> drift S
    v_h  = np.array([vel[0],vel[1],0.])
    V_h  = float(np.linalg.norm(v_h))
    if V_h > 0.5:
        vh      = v_h/V_h
        right   = np.array([-vh[1], vh[0], 0.])   # 90 deg CCW rotation = rightward
        pnd2V   = p*shell.diam/(2.*V_s)
        Fm      = CMg*qd*shell.area*pnd2V*right
    else:
        Fm = np.zeros(3)

    Ft = Fd + Fg + Fc + Fm + F_can

    # Spin decay
    dp = -(Cpd*qd*shell.area*shell.diam**2)/(2.*V_s*shell.Ixx)*p

    ds = np.zeros(7)
    ds[0:3] = vel
    ds[3:6] = Ft/shell.mass
    ds[6]   = dp
    return ds


def rk4(s, sh, wind, F_can, dt):
    f = lambda sv: eom(sv, sh, wind, F_can)
    k1=f(s); k2=f(s+.5*dt*k1); k3=f(s+.5*dt*k2); k4=f(s+dt*k3)
    return s+(dt/6.)*(k1+2*k2+2*k3+k4)

# ============================================================================
# ELEVATION CALIBRATION  (bisection to hit target range at zero dispersion)
# ============================================================================
def _range_for_el(el_deg, V0, tgt_N, shell, dt=0.10, t_max=200.0):
    """Fire with zero dispersion/wind; return N at impact."""
    el  = np.deg2rad(el_deg)
    s   = np.zeros(7)
    s[3] =  V0*np.cos(el); s[5] = -V0*np.sin(el)
    s[6] = 200.0*2*np.pi
    t    = 0.
    wind = np.zeros(3); F_can = np.zeros(3)
    while t <= t_max:
        if float(-s[2]) < -3. and t > 5.: break
        s = rk4(s, shell, wind, F_can, dt)
        t = round(t+dt, 6)
    return float(s[0])

def calibrate_elevation(V0, tgt_N, shell,
                        el_lo=40.0, el_hi=70.0,
                        tol=50.0, max_iter=20) -> float:
    """
    Bisection search: find elevation that gives range = tgt_N (± tol metres).
    Uses coarse dt=0.10 s for speed.
    """
    log.info("Calibrating elevation for range %.0f m (V0=%.1f m/s)...", tgt_N, V0)
    for _ in range(max_iter):
        el_mid = 0.5*(el_lo + el_hi)
        R = _range_for_el(el_mid, V0, tgt_N, shell)
        log.info("  el=%.3f deg  ->  R=%.1f m  (target %.1f m)", el_mid, R, tgt_N)
        if abs(R - tgt_N) < tol:
            log.info("  Converged: el=%.3f deg", el_mid)
            return el_mid
        if R < tgt_N:
            el_lo = el_mid
        else:
            el_hi = el_mid
    log.warning("Calibration did not fully converge; using el=%.3f deg", el_mid)
    return el_mid

# ============================================================================
# TRAJECTORY CLASS
# ============================================================================
class Trajectory:
    def __init__(self, muzzle_vel=827.0, elevation_deg=45.0, azimuth_deg=0.0,
                 spin_rps=200.0, target_ned=None, wind_ned=None,
                 guided=False, gnss_t_avail=45.0,
                 sigma_gnss_pos=0.0, sigma_gnss_vel=0.0, sigma_canard=0.0,
                 nav_constant=4.5, shell=None, dt=0.05, seed=0):

        self.V0     = muzzle_vel
        self.el     = np.deg2rad(elevation_deg)
        self.az     = np.deg2rad(azimuth_deg)
        self.p0     = spin_rps*2.*np.pi
        self.target = (np.array([28e3,0.,0.]) if target_ned is None
                       else np.asarray(target_ned, float))
        self.wind   = np.zeros(3) if wind_ned is None else np.asarray(wind_ned, float)
        self.guided = guided
        self.ta     = gnss_t_avail
        self.sg_p   = sigma_gnss_pos
        self.sg_v   = sigma_gnss_vel
        self.sg_c   = sigma_canard
        self.N_png  = nav_constant
        self.shell  = shell or Shell()
        self.dt     = dt
        self.rng    = np.random.default_rng(seed)

        # PNG state
        self._los_p = None; self._t_p = None
        self._lf    = np.zeros(3)
        self._pe    = None; self._ve = None; self._gt = -999.

    def _init(self):
        s    = np.zeros(7)
        s[3] =  self.V0*np.cos(self.el)*np.cos(self.az)
        s[4] =  self.V0*np.cos(self.el)*np.sin(self.az)
        s[5] = -self.V0*np.sin(self.el)
        s[6] = self.p0
        return s

    def _nav(self, pos, vel, t):
        if self._pe is None:
            self._pe = pos.copy(); self._ve = vel.copy(); return
        self._pe = self._pe + self._ve*self.dt
        if t >= self.ta and (t-self._gt) >= 0.10:
            pn = self.rng.normal(0., self.sg_p, 3)
            vn = self.rng.normal(0., self.sg_v, 3)
            self._pe  = .8*(pos+pn)+.2*self._pe
            self._ve  = .8*(vel+vn)+.2*vel
            self._gt  = t

    def _predict_impact(self, pos, vel):
        """Quick ballistic prediction (drag+gravity, dt=0.20 s). Returns (N, E) impact."""
        sh   = self.shell
        p, v = pos.copy(), vel.copy()
        wind = self.wind
        for _ in range(1200):
            if float(-p[2]) < -3. and _ > 2: break
            alt  = float(np.clip(-p[2], 0., 86e3))
            rho,_,Vs = _atm.at_altitude(alt)
            v_a  = v - wind; V = max(float(np.linalg.norm(v_a)), 0.5)
            Cd   = _aero.Cd(V/max(Vs,100.)); qd = 0.5*rho*V**2
            a    = (-Cd*qd*sh.area*(v_a/V) + sh.mass*np.array([0.,0.,G0])) / sh.mass
            v    = v + a*0.20; p = p + v*0.20
        return float(p[0]), float(p[1])

    def _png(self, pos, vel, t):
        """
        Guidance via ballistic impact-point prediction (BIPP).

        At each 10 Hz GNSS update, integrate forward from current GNSS position
        to predict ballistic impact (N_pred, E_pred). Compute miss dN, dE,
        and apply proportional acceleration corrections.

        Lateral (E): direct canard force.
        Range  (N):  forward drag bias (positive a_N = push forward = increase range).

        Gain: 2/tgo^2 (time-optimal zeroing, same as modern JDAM/Excalibur law).
        """
        tgt_N = self.target[0]; tgt_E = self.target[1]
        pos_N = float(pos[0]);  vel_N = float(vel[0])

        dN_remain = tgt_N - pos_N
        if vel_N < 20. or dN_remain < 0.:
            return np.zeros(3)
        tgo = dN_remain / vel_N
        if tgo < 2.0:
            return np.zeros(3)

        # Predict ballistic impact from current position/velocity
        N_pred, E_pred = self._predict_impact(pos, vel)

        # Miss vector
        dN = tgt_N - N_pred
        dE = tgt_E - E_pred

        # Proportional-tgo gain (k = 2/tgo^2)
        k = 2.0 / max(tgo**2, 4.0)

        a_E = float(np.clip(k * dE, -80., 80.))
        a_N = float(np.clip(k * dN, -40., 40.))   # range correction

        a_cmd = np.array([a_N, a_E, 0.0])
        amg   = float(np.linalg.norm(a_cmd))
        if amg > 80.: a_cmd *= 80./amg
        return a_cmd

    def run(self, t_max=200., rec=5):
        s = self._init(); t=0.; dt=self.dt; sh=self.shell
        ph=[]; vh=[]; th=[]; step=0
        while t <= t_max:
            pos=s[0:3]; vel=s[3:6]
            if float(-pos[2])<-3. and t>5.: break
            if step%rec==0:
                ph.append(pos.copy()); vh.append(vel.copy()); th.append(t)
            F_can = np.zeros(3)
            if self.guided and t>=self.ta:
                self._nav(pos, vel, t)
                if self._pe is not None:
                    a  = self._png(self._pe, self._ve, t)
                    if self.sg_c>0:
                        a += self.rng.normal(0., np.deg2rad(self.sg_c)*1.5, 3)
                    F_can = sh.mass*a
            s  = rk4(s, sh, self.wind, F_can, dt)
            t  = round(t+dt,6); step+=1
        return dict(ph=np.array(ph) if ph else np.zeros((1,3)),
                    vh=np.array(vh) if vh else np.zeros((1,3)),
                    th=np.array(th) if th else np.zeros(1),
                    impact=s[0:3].copy(), impact_t=t,
                    miss_2d=float(np.linalg.norm(s[0:2]-self.target[0:2])))

# ============================================================================
# MONTE CARLO
# ============================================================================
@dataclass
class MCConfig:
    N:              int   = 1000
    sigma_V:        float = 3.0
    sigma_el_mrad:  float = 0.15
    sigma_az_mrad:  float = 0.50
    sigma_wind:     float = 7.0
    gnss_t_avail:   float = 45.0
    sigma_gnss_pos: float = 1.5
    sigma_gnss_vel: float = 0.05
    sigma_canard:   float = 0.30
    n_workers:      int   = max(1,(os.cpu_count() or 4)-1)


def _one_shot(args):
    seed,guided,V0,el_d,az_d,tgt_l,mc_d = args
    rng = np.random.default_rng(seed)
    dV  = rng.normal(0.,mc_d["sigma_V"])
    dEl = rng.normal(0.,mc_d["sigma_el_mrad"])*1e-3
    dAz = rng.normal(0.,mc_d["sigma_az_mrad"])*1e-3
    Wm  = abs(rng.normal(0.,mc_d["sigma_wind"]))
    Wd  = rng.uniform(0.,2*np.pi)
    wind = np.array([Wm*np.cos(Wd),Wm*np.sin(Wd),0.])
    tr = Trajectory(
        muzzle_vel     = V0+dV,
        elevation_deg  = el_d+np.rad2deg(dEl),
        azimuth_deg    = az_d+np.rad2deg(dAz),
        spin_rps       = 200.,
        target_ned     = np.array(tgt_l,float),
        wind_ned       = wind,
        guided         = guided,
        gnss_t_avail   = mc_d["gnss_t_avail"] if guided else 9999.,
        sigma_gnss_pos = mc_d["sigma_gnss_pos"] if guided else 0.,
        sigma_gnss_vel = mc_d["sigma_gnss_vel"] if guided else 0.,
        sigma_canard   = mc_d["sigma_canard"] if guided else 0.,
        dt             = 0.05, seed=seed,
    )
    res = tr.run(t_max=220., rec=999999)
    return dict(miss_2d=res["miss_2d"],
                impact_N=float(res["impact"][0]),
                impact_E=float(res["impact"][1]),
                impact_t=float(res["impact_t"]),
                dV=float(dV), wind_mag=float(Wm))


def run_mc(N, guided, V0, el_d, az_d, tgt, mc):
    lbl  = "Guided" if guided else "Unguided"
    mc_d = mc.__dict__.copy()
    args = [(42+i,guided,V0,el_d,az_d,list(tgt.astype(float)),mc_d)
            for i in range(N)]
    results=[]; t0=time.time(); nw=mc.n_workers
    if nw>1 and N>10:
        with ProcessPoolExecutor(max_workers=nw) as ex:
            futs={ex.submit(_one_shot,a):i for i,a in enumerate(args)}
            done=0
            for f in as_completed(futs):
                results.append(f.result()); done+=1
                if done%max(1,N//10)==0:
                    eta=(time.time()-t0)/done*(N-done)
                    log.info("%s %4d/%d  ETA %5.1f s",lbl,done,N,eta)
    else:
        for i,a in enumerate(args):
            results.append(_one_shot(a))
            if (i+1)%max(1,N//10)==0:
                eta=(time.time()-t0)/(i+1)*(N-i-1)
                log.info("%s %4d/%d  ETA %5.1f s",lbl,i+1,N,eta)
    el=time.time()-t0
    log.info("%s done: %d in %.1f s (%.3f s/run)",lbl,N,el,el/N)
    return results

# ============================================================================
# STATISTICS
# ============================================================================
def emp_cep(d): return float(np.sort(d)[max(0,int(np.ceil(.5*len(d)))-1)])
def emp_r95(d): return float(np.sort(d)[max(0,int(np.ceil(.95*len(d)))-1)])

def stats(results):
    miss=np.array([r["miss_2d"]  for r in results])
    iN  =np.array([r["impact_N"] for r in results])
    iE  =np.array([r["impact_E"] for r in results])
    tf  =np.array([r["impact_t"] for r in results])
    return dict(N=len(miss),CEP=emp_cep(miss),R95=emp_r95(miss),
                m_mean=float(miss.mean()),m_std=float(miss.std()),
                m_max=float(miss.max()),
                sN=float(iN.std()),sE=float(iE.std()),
                mN=float(iN.mean()),mE=float(iE.mean()),
                tf=float(tf.mean()),miss=miss,iN=iN,iE=iE)

# ============================================================================
# DISPLAY
# ============================================================================
def telem(res, label, step=3):
    ph=res["ph"]; vh=res["vh"]; th=res["th"]
    if len(th)==0: return
    print(f"\n  {label}")
    print(f"  {'T(s)':>6}  {'N(m)':>9} {'E(m)':>8} {'Alt(m)':>8}  "
          f"{'Spd(m/s)':>9}  {'Mach':>6}")
    print("  "+"-"*62)
    for i in range(0,len(th),step):
        V   = float(np.linalg.norm(vh[i]))
        alt = float(-ph[i,2])
        _,_,Vs = _atm.at_altitude(max(0.,alt))
        print(f"  {th[i]:6.1f}  {ph[i,0]:9.1f} {ph[i,1]:8.1f} {alt:8.1f}  "
              f"{V:9.2f}  {V/max(Vs,1.):6.3f}")
    ip=res["impact"]
    print(f"  {'IMPACT':>6}  {ip[0]:9.1f} {ip[1]:8.1f} {-ip[2]:8.1f}  "
          f"  T={res['impact_t']:.1f}s  Miss={res['miss_2d']:.1f} m")
    print("  "+"-"*62)


def scatter(su, sg, tgt, scale=300.):
    W,H=78,26; cx,cy=W//2,H//2
    mpc=2*scale/W; mpr=2*scale/H
    grid=[[' ']*W for _ in range(H)]
    grid[cy][cx]='+'
    def put(dN,dE,ch):
        c=cx+int(dE/mpc); r=cy-int(dN/mpr)
        if 0<=c<W and 0<=r<H: grid[r][c]=ch
    for dN,dE in zip(su["iN"]-tgt[0],su["iE"]-tgt[1]): put(dN,dE,'.')
    for dN,dE in zip(sg["iN"]-tgt[0],sg["iE"]-tgt[1]): put(dN,dE,'*')
    sep="="*81
    print("\n"+sep)
    print(f"  IMPACT SCATTER  (half-width {scale:.0f} m)  "
          f"'+' target  '.' unguided  '*' guided")
    print(sep)
    for row in grid: print("|"+"".join(row)+"|")
    print(sep)


def histogram(mu, mg, max_v=None, bins=20):
    max_v = max_v or float(max(mu.max(),mg.max())*1.05)
    edges = np.linspace(0,max_v,bins+1)
    uh,_=np.histogram(mu,bins=edges); gh,_=np.histogram(mg,bins=edges)
    mc=max(uh.max(),gh.max(),1); BW=26
    print("\n  MISS DISTANCE DISTRIBUTION  (# = normalised count)")
    print(f"  {'Range(m)':<12}  {'Unguided':<32}  {'Guided':<32}")
    print("  "+"-"*78)
    for i in range(bins):
        lo,hi=edges[i],edges[i+1]
        ub=int(uh[i]/mc*BW); gb=int(gh[i]/mc*BW)
        print(f"  {lo:5.0f}-{hi:4.0f}m  "
              f"{'#'*ub:<26} {uh[i]:4d}  |  "
              f"{'#'*gb:<26} {gh[i]:4d}")
    print("  "+"-"*78)


def summary(su, sg, req=30.0):
    imp  = su["CEP"]/max(sg["CEP"],0.1)
    flag = "MET  ***" if sg["CEP"]<=req else f"NOT MET ({sg['CEP']:.1f} m)"
    print("\n"+"="*68)
    print("  YIL-PGK-155  MONTE CARLO RESULTS SUMMARY")
    print("="*68)
    print(f"  {'Metric':<32} {'Unguided':>15} {'Guided':>15}")
    print("  "+"-"*62)
    rows=[
        ("Runs",                    su['N'],               sg['N']),
        ("CEP [m]",                 f"{su['CEP']:.1f}",   f"{sg['CEP']:.1f}"),
        ("R95 [m]",                 f"{su['R95']:.1f}",   f"{sg['R95']:.1f}"),
        ("Mean miss [m]",           f"{su['m_mean']:.1f}",f"{sg['m_mean']:.1f}"),
        ("1-sigma miss [m]",        f"{su['m_std']:.1f}", f"{sg['m_std']:.1f}"),
        ("Max miss [m]",            f"{su['m_max']:.1f}", f"{sg['m_max']:.1f}"),
        ("Sigma_N (downrange) [m]", f"{su['sN']:.1f}",   f"{sg['sN']:.1f}"),
        ("Sigma_E (lateral) [m]",   f"{su['sE']:.1f}",   f"{sg['sE']:.1f}"),
        ("Magnus+Coriolis bias E[m]",f"{su['mE']:.1f}",  f"{sg['mE']:.1f}"),
        ("Mean flight time [s]",    f"{su['tf']:.1f}",   f"{sg['tf']:.1f}"),
        ("CEP improvement factor",  "1.0x",               f"{imp:.1f}x"),
        ("Requirement (<=30 m)",    "NO",                 flag),
    ]
    for name,u,g in rows:
        print(f"  {str(name):<32} {str(u):>15} {str(g):>15}")
    print("="*68)

# ============================================================================
# MAIN
# ============================================================================
def main():
    if hasattr(sys.stdout,"reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8",errors="replace")

    parser=argparse.ArgumentParser()
    parser.add_argument("--N",  type=int,   default=1000)
    parser.add_argument("--dt", type=float, default=0.01)
    args=parser.parse_args()

    print("\n"+"="*68)
    print("  YIL-PGK-155  Phase 5: Point-Mass Trajectory & Monte Carlo")
    print("  Yantra India Limited  |  RESTRICTED")
    print("="*68+"\n")

    V0    = 827.0
    tgt   = np.array([28_000.0, 0.0, 0.0])
    mc    = MCConfig(N=args.N)
    sh    = Shell()

    # ---- Step 1: Calibrate elevation to hit target at nominal conditions ----
    # (zero wind, zero dispersion)
    el_cal = calibrate_elevation(V0, tgt[0], sh,
                                  el_lo=42., el_hi=58., tol=50.)
    az_cal = 0.0    # due North
    log.info("Calibrated elevation: %.4f deg", el_cal)

    # ---- Step 2: Reference trajectories at calibrated elevation ----
    ref_wind = np.array([4.0, 3.0, 0.0])   # 4 m/s headwind, 3 m/s crosswind

    log.info("Reference: UNGUIDED  dt=%.3f s", args.dt)
    r_ug = Trajectory(V0, el_cal, az_cal, 200., tgt, ref_wind, False,
                      shell=sh, dt=args.dt, seed=0).run(rec=4)

    log.info("Reference: GUIDED    dt=%.3f s", args.dt)
    r_g  = Trajectory(V0, el_cal, az_cal, 200., tgt, ref_wind, True,
                      gnss_t_avail=45., sigma_gnss_pos=1.5,
                      sigma_gnss_vel=0.05, sigma_canard=0.3,
                      shell=sh, dt=args.dt, seed=0).run(rec=4)

    telem(r_ug, "UNGUIDED reference  (4 m/s headwind, 3 m/s crosswind)", step=4)
    telem(r_g,  "GUIDED reference    (4 m/s headwind, 3 m/s crosswind)", step=4)

    log.info("Unguided miss = %.1f m (expected ~wind drift ~200 m)",
             r_ug["miss_2d"])
    log.info("Guided miss   = %.1f m (expected <30 m)", r_g["miss_2d"])

    # ---- Step 3: Monte Carlo ----
    log.info("Monte Carlo: %d unguided + %d guided  workers=%d",
             mc.N, mc.N, mc.n_workers)

    res_ug = run_mc(mc.N, False, V0, el_cal, az_cal, tgt, mc)
    res_g  = run_mc(mc.N, True,  V0, el_cal, az_cal, tgt, mc)

    su = stats(res_ug)
    sg = stats(res_g)

    summary(su, sg)

    scale = float(np.clip(su["R95"]*1.2, 200., 1200.))
    scatter(su, sg, tgt, scale=scale)
    histogram(su["miss"], sg["miss"],
              max_v=float(np.clip(su["m_max"]*1.05, 100., 900.)))

    req = sg["CEP"]<=30.
    print(f"\n  CEP REQUIREMENT (<=30 m): {'*** MET ***' if req else 'NOT MET'}")
    print(f"  Guided CEP   = {sg['CEP']:.1f} m")
    print(f"  Unguided CEP = {su['CEP']:.1f} m")
    print(f"  Improvement  = {su['CEP']/max(sg['CEP'],0.1):.1f}x\n")


if __name__=="__main__":
    main()
