# SIH 2025 IDEATION PRESENTATION BLUEPRINT

## SLIDE 1: TITLE PAGE

**PROJECT TITLE**
Design and Development of a Precision Guidance Kit (PGK) with Canard Actuation and Multi-Mode Electronic Fuze for a 155 mm Artillery Shell

**CHALLENGE METADATA**
*   **Organization:** Yantra India Limited (YIL)
*   **Category:** Hardware
*   **Theme:** Defense & Security / Advanced Manufacturing
*   **Team ID:** `[Insert Team ID]`
*   **Team Name:** `[Insert Team Name]`
*   **Institute/Email:** `[Insert Institute Name / Email]`

**TAGLINE**
> "Transforming conventional 155 mm artillery into surgical strike assets through bolt-on, >15,000 g-hardened precision mechatronics—achieving sub-15 meter CEP at a fraction of standard guided-munitions cost."

### [DIRECTIVES FOR CREATIVE LEAD / GRAPHIC DESIGNER]
*   **Layout & Grid:** Center-aligned, highly structured title block. Bottom 20% reserved for YIL, SIH, and MoE logos. 
*   **Visual Assets:** Use a dark, aerospace-grade aesthetic (deep navy, gunmetal grey, high-contrast cyan/orange accents). Background should feature a subtle, high-tech blueprint or wireframe of the 155 mm shell trajectory.
*   **Highlight Badge:** Add a glowing, metallic badge in the top right corner: `[ NATO 2-INCH MIL-STD COMPLIANT ]`.

***

## SLIDE 2: IDEA TITLE & PROPOSED SOLUTION

**SYSTEM IDENTITY**
*   **Project Code Name:** *Astra-PGK 155*
*   **Interface Architecture:** Drop-in fuze replacement (NATO standard 2-inch 12-UN-2A thread). No structural modifications to standard M795 / ERFB casings required.

**CORE SYSTEM BREAKDOWN**
1.  **Canard Actuation Assembly (CAA):**
    *   **Mechanism:** Rolling De-Spin Bearing Assembly (RDBA) using duplex-DB SKF 71905 ACE bearings.
    *   **Aero-Surfaces:** 4× NACA-0010 Ti-6Al-4V Titanium canards (80 mm span, 40 mm root chord, 5° LE sweep). 
    *   **Drive:** 4× BLDC motors + 50:1 Harmonic Drives for high-torque aerodynamic deflection under Mach 2.5 flow.
2.  **Integrated Multi-Mode Electronic Fuze:**
    *   **Modes:** Height of Burst (24 GHz FMCW Radar), Electronic Time (ET), Point Detonating (PD), and Delay.
    *   **Interlock:** 3-Environment Electronic In-Line Safety & Arming (EISA) Fire State Machine (FSM).
3.  **Guidance, Navigation, and Control (GNC):**
    *   **Navigation:** Loosely-coupled 16-state EKF fusing u-blox F9P GNSS (NavIC/GPS) + ADIS16460 high-g MEMS IMU.

**INNOVATION & PROBLEM ALIGNMENT**
*   **The Problem:** Standard 155 mm ballistics (827 m/s muzzle velocity) yield a CEP of >260 m at 28 km due to wind, Magnus drift (35m), and Coriolis forces.
*   **The Innovation:** Replaces expensive, dedicated smart rounds (e.g., Excalibur) with a modular, ₹2.13 Lakh retrofit kit. Mitigates high-g shock (>15k g) via polyurethane potting and silicon micro-springs.

### [DIRECTIVES FOR CREATIVE LEAD / GRAPHIC DESIGNER]
*   **Layout & Grid:** 60/40 Vertical Split. Left 60%: Three heavily styled architectural cards (CAA, Fuze, GNC). Right 40%: Large, transparent 3D exploded CAD render of the PGK.
*   **Visual Assets:** Embed `PGK_cad_exploded_view.jpg`. Add connecting leader lines from the text cards to the physical CAD components (e.g., line from "CAA" to the Titanium canards).
*   **Callout Box:** `[ VALIDATED CEP: 10.2 m (1,000-Run Monte Carlo) ]` in a high-vis neon green or amber box.

***

## SLIDE 3: TECHNICAL APPROACH & ARCHITECTURE

**HARDWARE & EMBEDDED STACK**
*   **Primary Compute:** Xilinx Spartan-7 XC7S15 FPGA (Radar DSP/S&A) + STM32H743 MCU (GNC/EKF at 480 MHz).
*   **Power Architecture:** Reserve thermal battery (Li/FeS2) activated by 15,000 g axial setback + 250 Hz spin, paired with 5F supercapacitor buffer for servo peak loads.
*   **Setter Interface:** MIL-STD-1760 / AOP-22 compliant inductive programming coil for pre-launch targeting.

**GNC ALGORITHM ARCHITECTURE**
*   **Loop Rate:** 100 Hz strict rate-monotonic RTOS execution.
*   **Guidance Law:** 2D Ballistic Impact-Point Prediction (BIPP) overriding standard PNG to prevent vertical axis gravity-drop saturation.
*   **Correction Equation:** `a_cmd = 2 * (Pos_target - Pos_pred) / t_go^2` (Gain limited to 80 m/s² lateral).

**TRAJECTORY EXECUTION FLOW**
```text
[T+0.0s] MUZZLE EXIT (827 m/s, 15,000 g, 200 RPS Spin)
   │
   ▼
[T+0.5s] SETBACK & SPIN ARMING (EISA State 1: Battery Active)
   │
   ▼
[T+2.0s] CANARD UNLATCH (Centrifugal release -> 90° Deployment)
   │
   ▼
[T+35.0s] APOGEE (8.6 km Alt) -> GNSS Cold Start Fix Acquired
   │
   ▼
[T+45.0s] GNC ACTIVE (100 Hz EKF + BIPP Canard Deflection)
   │
   ▼
[T+86.8s] TERMINAL PHASE (24 GHz FMCW HOB / Point Detonation)
```

### [DIRECTIVES FOR CREATIVE LEAD / GRAPHIC DESIGNER]
*   **Layout & Grid:** Top 40% for Hardware/Software specs using dense, 2-column tech-spec tables. Bottom 60% for a sweeping, full-width Trajectory Arc graphic.
*   **Visual Assets:** Replace the raw ASCII text with a polished vector infographic (A parabolic curve over a terrain grid). Place distinct waypoint nodes along the curve matching the ASCII timestamps (Muzzle Exit -> Apogee -> Impact).
*   **Callout Box:** `< 0.5 ms Compute Latency on STM32H743 Cortex-M7`

***

## SLIDE 4: FEASIBILITY, RISKS & MITIGATION

**STRUCTURAL & AERODYNAMIC FEASIBILITY**
*   **Survivability:** Kit structurally analyzed for `15,000 g` axial setback and `1,257 rad/s` centrifugal forces.
*   **Aero Penalty:** Drag scale factor (`Cd × 0.70`) yields 28 km range at 48.9° elevation. Canard drag penalty compensated by precision trajectory shaping.

**FAILURE MODE & EFFECTS ANALYSIS (FMEA)**

| Risk Identification | Severity | Mitigation Strategy & Protocol |
| :--- | :---: | :--- |
| **High-g Delamination / Component Shearing** | **CRITICAL** | Sub-assembly encapsulation via Epoxies/Polyurethane potting; 3× M3 Ti shear pins for canard stowage; **MIL-STD-810H** shock qualification. |
| **GNSS Jamming / Spoofing in EW Environments** | **HIGH** | Loose-coupled EKF reverts to ADIS16460 dead-reckoning fallback; multi-constellation (NavIC/GPS) baseband filtering. |
| **Premature Detonation (In-Bore / Muzzle)** | **CRITICAL** | Dual-environment threshold: Axial acceleration `>10,000 g` AND sustained spin `>150 Hz` required to unlock S&A barrier per **MIL-STD-1316**. |
| **Actuator Saturation at High Mach** | **MODERATE** | BLDC torque multiplied by 50:1 Harmonic Drive (CSF-8); lateral acceleration hard-limited to `80 m/s^2` in control firmware. |

### [DIRECTIVES FOR CREATIVE LEAD / GRAPHIC DESIGNER]
*   **Layout & Grid:** Split diagonally. Top-Left: The FMEA table formatted as a sleek, color-coded dashboard (Severity = Red/Amber dots). Bottom-Right: A semi-transparent internal CAD rendering of the RDBA and bearing stack.
*   **Visual Assets:** Use warning icons (shields, crosshairs, radiation/EW symbols) next to each FMEA row. 
*   **Callout Box:** `[ SAFETY FIRST: FULLY MIL-STD-1316 COMPLIANT EISA ]`

***

## SLIDE 5: IMPACT, BENEFITS & DEFENSE READINESS

**OPERATIONAL & STRATEGIC DOMINANCE**
*   **Dispersion Annihilation:** Monte Carlo 6-DOF simulation demonstrates CEP reduction from **264.9 m (unguided)** to **10.2 m (guided)**—a 25.9× accuracy multiplier.
*   **Logistics Optimization:** Reduces the ammunition expenditure from 20–50 rounds per neutralized target down to **1–2 rounds**. Exponentially reduces supply chain fuel burden and extends artillery barrel life by conserving shots.
*   **Collateral Mitigation:** Sub-15m accuracy enables surgical strikes in contested urban terrain without escalating collateral damage.

**ECONOMIC & INDUSTRIAL VIABILITY (ATMANIRBHAR BHARAT)**
*   **Disruptive Unit Economics:** 
    *   BOM Cost: ₹1,16,370 (Electronics) + ₹1,21,960 (Mech) = **~₹2.13 Lakh** unit cost (at 1k qty).
    *   Contrast: US M1156 PGK (~$20,000) or Excalibur (~$100,000).
*   **Manufacturing Readiness:** 100% compatible with YIL's existing 155mm production lines. Machined from domestic Ti-6Al-4V and Al 7075-T6 stock. No casing redesign required.

### [DIRECTIVES FOR CREATIVE LEAD / GRAPHIC DESIGNER]
*   **Layout & Grid:** Three vertical pillars. Pillar 1: Accuracy (Radar charts/scatter plot). Pillar 2: Logistics (Bar charts for round reduction). Pillar 3: Economics (Cost comparison).
*   **Visual Assets:** Create a side-by-side scatter plot visualization. Left cluster: "Unguided Dispersion (Large scatter cloud)". Right cluster: "Guided Dispersion (Tight dot cluster around crosshairs)".
*   **Callout Box:** `[ 25.9× LETHALITY MULTIPLIER @ 1/40th THE COST OF EXCALIBUR ]`

***

## SLIDE 6: RESEARCH, REFERENCES & STANDARDS

**DEFENSE STANDARDS COMPLIANCE**
*   **MIL-STD-333B:** Fuze contours, dimensions, and 2-inch standard threading.
*   **MIL-STD-1316:** Fuze design safety criteria and dual-environmental arming.
*   **MIL-STD-810H:** Environmental engineering guidelines (Shock, vibration, temperature extremes).
*   **STANAG 4369 / 4114:** Inductive setting of artillery fuzes and NATO ballistic modeling.

**AEROSPACE LITERATURE & BALLISTIC MODELING**
*   *Modified Point Mass Trajectory Modeling:* Based on McCoy's "Modern Exterior Ballistics" for 6-DOF high-spin projectile modeling.
*   *Magnus Drift Derivations:* Integration of rightward Magnus force drift calculations yielding ~35 m cross-range drift at 28 km unguided.
*   *Aerodynamic Cross-Coupling:* Canard roll-decoupled dynamics and lift-vector orientation studies for non-linear control loops.

**VALIDATION ARTIFACTS**
*   **Simulation Rig:** 1000-run Monte Carlo 6-DOF simulation (Python/NumPy).
*   **CAD Modeling:** Parametric CadQuery STEP/STL generation.
*   **Hardware Description:** Verilog Fire State Machine & CFAR FMCW radar detection logic.

### [DIRECTIVES FOR CREATIVE LEAD / GRAPHIC DESIGNER]
*   **Layout & Grid:** Heavy grid/matrix format. Group the MIL-STDs into "Standardization Badges". Place equations and modeling citations in clean, academic-looking blockquotes.
*   **Visual Assets:** Embed `PGK_cad_assembly_render.jpg` in the bottom right corner as a concluding, photorealistic visual anchor. 
*   **Callout Box:** Show mathematical rigor by dropping the exact Magnus force vector formula in a sleek blackboard-style overlay graphic.
