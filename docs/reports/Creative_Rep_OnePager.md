# CREATIVE REP DATA SHEET: YIL 155mm PGK (SIH 2025)
**Designer Objective:** Build a 6-slide deck using this exact data. Keep aesthetics aerospace-grade (dark themes, cyan/orange accents, blueprint elements).

---

### SLIDE 1: TITLE & HERO
*   **Title:** Design & Development of a Precision Guidance Kit (PGK) for 155 mm Artillery
*   **Client / Category:** Yantra India Limited (YIL) | Hardware | Defense & Security
*   **Tagline:** *Transforming conventional artillery into surgical strike assets via >15,000 g-hardened modular guidance (Sub-15m CEP).*
*   **Visual:** Title block + Dark background with a faint wireframe of an artillery shell trajectory.

### SLIDE 2: THE SOLUTION (ASTRA-PGK 155)
*   **Visual Needed:** Large 3D Exploded Render (Use `PGK_cad_exploded_view.jpg`).
*   **Callouts to point at the 3D Render:**
    *   *Canards:* 4× NACA-0010 Titanium, 80mm span, BLDC + Harmonic Drive (50:1).
    *   *Bearing:* SKF 71905 ACE Rolling De-Spin Bearing (RDBA).
    *   *Fuze Base:* Drop-in NATO 2-inch 12-UN-2A Thread.
    *   *Nose:* 24GHz FMCW Radar & Anti-jam GPS (NavIC) window.
*   **Highlight Badge:** `[ VALIDATED CEP: 10.2 METERS ]`

### SLIDE 3: TECH ARCHITECTURE & TRAJECTORY
*   **Visual Needed 1:** Parabolic Trajectory Infographic showing these 5 nodes:
    1. `T+0.0s`: Muzzle Exit (827 m/s, 15,000 g shock).
    2. `T+0.5s`: Setback & Spin Arming (Battery Active).
    3. `T+2.0s`: Canard Unlatch (90° Deploy).
    4. `T+35s`: Apogee (8.6 km Alt) -> GPS Lock.
    5. `T+86s`: Impact/Airburst Detonation.
*   **Visual Needed 2:** Tech Spec Table:
    *   *Compute:* STM32H743 MCU + Spartan-7 FPGA (<0.5ms latency, 100Hz loop).
    *   *Guidance Law:* 2D Ballistic Impact-Point Prediction (BIPP).
    *   *Equation to display:* $a_{cmd} = 2 \cdot (Pos_{target} - Pos_{pred}) / t_{go}^2$

### SLIDE 4: FEASIBILITY & RISKS (FMEA)
*   **Visual Needed:** A sleek "Threat & Mitigation" Dashboard Matrix.
*   **Row 1 (Shock):** >15,000 g firing shock. *Fix:* Polyurethane potting + Titanium shear pins. (Severity: Critical 🔴)
*   **Row 2 (Jamming):** EW/GPS Spoofing. *Fix:* 16-State EKF + ADIS16460 IMU dead-reckoning fallback. (Severity: High 🟠)
*   **Row 3 (Safety):** Premature Detonation. *Fix:* Dual-environment unlock (Axial >10k g + Spin >150 Hz). (Severity: Critical 🔴)

### SLIDE 5: IMPACT & ATMANIRBHAR BHARAT
*   **Visual Needed 1 (Scatter Plot):** "Dispersion Annihilation". Left side: Huge scatter cloud (Unguided CEP = 264.9 m). Right side: Tiny tight dot cluster (Guided CEP = 10.2 m).
*   **Visual Needed 2 (Bar Chart/Stats):**
    *   *Rounds needed per target:* 50 (Unguided) vs. 2 (Guided).
    *   *Unit Cost:* ₹2.13 Lakh (Astra-PGK) vs. ₹87 Lakh (US Excalibur).
*   **Highlight Badge:** `[ 25.9× LETHALITY MULTIPLIER ]`

### SLIDE 6: STANDARDS & VALIDATION
*   **Visual Needed:** Grid of "Compliance Badges" and a screenshot of the Monte Carlo simulation data.
*   **Badges to create:**
    *   `MIL-STD-1316` (Fuze Safety)
    *   `MIL-STD-333B` (NATO 2-inch standard)
    *   `MIL-STD-810H` (Shock & Vibration)
    *   `STANAG 4369` (Inductive Setting)
*   **Validation Data:** 1,000-run 6-DOF Monte Carlo successfully completed. Parametric CAD fully modeled.
