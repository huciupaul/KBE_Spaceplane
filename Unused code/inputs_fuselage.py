"""
inputs_fuselage.py
==================
Mock inputs for the standalone Fuselage module (and combined Fuselage+Wing main).

These replace the outputs that would normally come from:
  - MissionAnalysis   (mission_*)
  - PropellantTanks   (tanks_*)
  - UserInputs        (payload_*)
  - constants.py      (fixed geometric / structural parameters)

All values are in SI units (kg, m, s, Pa, N) unless the name ends in _deg.

HOW TO USE
----------
1. Edit any value below.
2. Run fuselage.py for the standalone fuselage viewer, or
   main_fuselage_wing.py for the combined fuselage + wing assembly.
3. In the ParaPy side-panel every Input mirrors a variable here;
   change a value and press Enter to rebuild instantly.

TARGET VEHICLE: Dawn Aerospace Aurora-class suborbital spaceplane
  MTOW ~ 150-250 kg | D_fus ~ 0.25-0.40 m | L_fus ~ 3.0-5.0 m
"""

# ── Mission-derived inputs (normally from MissionAnalysis) ────────────────────
# q_max: 0.5 * rho * V_max**2 at altitude of maximum dynamic pressure.
# Conservative design point at Mach 3.7 / 15 km: ~60 kPa.
mission_q_max  = 60_000.0   # [Pa]   maximum dynamic pressure
mission_M_max  =      3.7   # [-]    maximum Mach number
mission_MTOW   =    200.0   # [kg]   MTOW estimate (seeded value)

# Propellant masses (normally from MissionAnalysis via Tsiolkovsky equation)
# Placeholder values consistent with a ~200 kg MTOW Aurora-class vehicle
mission_m_ox   =     62.0   # [kg]   oxidiser mass (N2O)
mission_m_fu   =     48.0   # [kg]   fuel mass (ethanol)

# ── Propellant tank geometry (normally from PropellantTanks) ──────────────────
# Tank lengths are sized for the propellant volumes at inner fuselage diameter.
# First-pass: inner diameter = ENGINE_DIAM = 0.20 m (will be resolved by loop).
tanks_L_tank_ox    = 0.65   # [m]   oxidiser tank total length
tanks_L_tank_fu    = 0.55   # [m]   fuel tank total length
tanks_D_inner      = 0.22   # [m]   inner fuselage diameter (from tanks module)

# ── User inputs (payload mission requirements) ─────────────────────────────────
payload_mass       =  10.0  # [kg]   payload mass
payload_volume     =  0.010 # [m^3]  payload volume (10 litres)

# ── Constants (normally from constants.py) ────────────────────────────────────

# Engine envelope
ENGINE_DIAM    =  0.20      # [m]    outer engine diameter
ENGINE_LEN     =  0.60      # [m]    engine overall length

# Avionics bay (fixed, from equipment catalogue)
AVIONICS_LEN   =  0.35      # [m]    avionics bay length (fixed)
AVIONICS_DIAM  =  0.18      # [m]    avionics bay outer diameter
AVIONICS_MASS  =  8.0       # [kg]   avionics system mass

# Material / structural
RHO_AL         = 2700.0     # [kg/m^3]  aluminium alloy 2024-T3 density
SIGMA_AL       = 200e6      # [Pa]      allowable stress (yielded)
K_FUS          =  0.15      # [-]       fuselage structural mass fraction const

# Geometric ratios (Raymer §6.3 / Aurora geometry)
LAMBDA_FUSE    = 10.0       # [-]    fuselage slenderness L/D
K_NOSE         =  2.8       # [-]    nose cone length factor k*D
K_TAIL         =  1.5       # [-]    boat-tail length factor
K_FILL         =  0.70      # [-]    payload bay volumetric fill factor
