"""
inputs_wing.py
==============
Mock inputs for the standalone Wing module.

In the combined main_fuselage_wing.py the fuselage-derived inputs
(fus_L_fus, x_cg) are computed by Fuselage and passed programmatically;
they are NOT hardcoded here.

These replace the outputs that would normally come from:
  - MissionAnalysis   (mission_*)
  - WeightAndCG       (x_cg)       ← provided by Fuselage in combined run
  - Fuselage          (fus_*)      ← provided by Fuselage in combined run
  - constants.py      (fixed aerodynamic / structural parameters)

All values are in SI units (kg, m, s, Pa, N, rad) unless the name ends in _deg.

HOW TO USE
----------
Standalone (python wing.py):
  - The fallback values for fus_L_fus and x_cg below are used.
  - Edit any value and run; the ParaPy GUI reflects changes instantly.

Combined (python main_fuselage_wing.py):
  - fus_L_fus and x_cg are forwarded from the live Fuselage object.
  - Only the aero/structural constants here are used.
"""

# ── Mission-derived inputs (normally from MissionAnalysis) ────────────────────
# q_max: 0.5 * rho * V_max**2 at altitude of maximum dynamic pressure.
# Conservative design point (ascent, Mach 3.7 / 15 km).
mission_q_max = 60_000.0   # [Pa]   maximum dynamic pressure
mission_M_max =      3.7   # [-]    maximum Mach number (Mach 3.7 @ ~20 km)
mission_MTOW  =    200.0   # [kg]   take-off mass estimate (seeded value)


# ── Fuselage geometry (normally from Fuselage) ────────────────────────────────
# In combined mode these are overridden by Fuselage outputs (fus.L_fus etc.).
# Provide sensible standalone fallback values matching inputs_fuselage.py defaults.
fus_L_fus     =      4.0   # [m]    total fuselage length  (Fuselage.total_length)
fus_R_fus     =      0.20  # [m]    fuselage outer radius  (Fuselage.outer_diameter/2)
                           #        standalone fallback; overridden in main.py

# ── Weight & CoM inputs (normally from WeightAndCG) ───────────────────────────
# In combined mode these are overridden by Fuselage outputs.
x_cg          =      0.5 * fus_L_fus   # [m]    longitudinal centre of mass from nose tip

# ── Aerodynamic / structural constants (normally from constants.py) ────────────
AIRFOIL_TC    =      0.05  # [-]    thickness-to-chord ratio t/c  (NACA 64A-005)
Lambda_LE_deg =     60.0   # [deg]  leading-edge sweep (spec §9.3, M>2)
AR            =      2.5   # [-]    aspect ratio (supersonic optimum, Raymer §12.5)
lambda_t      =      0.35  # [-]    taper ratio (near-elliptic, Raymer Ch. 6)
N_DESIGN      =      3.0   # [-]    design load factor, pull-out (CS-23)
SM_TARGET     =      0.05  # [-]    static-margin target (fraction of MAC)
G0            =      9.807 # [m/s²] standard gravity