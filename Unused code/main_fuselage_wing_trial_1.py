# main_fuselage_wing.py  --  Combined Fuselage + Wing Assembly
# =============================================================
# Assembles the Fuselage and Wing modules into a single ParaPy
# KBEObject.  Fuselage outputs (L_fus, x_cg_fus) are wired
# directly into Wing inputs — no hardcoded fuselage values in wing.
#
# Interior boxes:
#   - avionics bay  : fixed set from constants (AVIONICS_DIAM, AVIONICS_LEN)
#   - payload bay   : user-driven (payload_mass, payload_volume)
#
# Interactive workflow:
#   python main_fuselage_wing.py
#   The ParaPy viewer opens.  Every Input (fuselage AND wing) is editable
#   in the side-panel.  Change a value, press Enter → full geometry rebuilds.
#
# Design consistency:
#   Wing.fus_L_fus  <─ Fuselage.L_fus   (live attribute reference)
#   Wing.x_cg       <─ Fuselage.xcm_fus (structural shell CoM seed)
#   Wing.mission_q_max == Fuselage.mission_q_max   (same scalar)
#   Wing.mission_M_max / MTOW / constants shared through top-level Inputs
#
# Coordinate system:
#   X -> downstream (nose tip = 0)
#   Y -> starboard
#   Z -> up
# All lengths in metres, masses in kilograms.

from __future__ import annotations

# -- ParaPy core ---------------------------------------------------------------
from parapy.core import Base, Input, Attribute, Part
from parapy.gui import display

# -- Subsystem modules ---------------------------------------------------------
from fuselage_trial_1 import Fuselage
from wing_trial_1 import Wing          # Wing class from wing_trial_1.py

# -- Mock input defaults -------------------------------------------------------
import inputs_fuselage as _IF
import inputs_wing      as _IW


# ==============================================================================
# Top-level assembly
# ==============================================================================

class FuselageWingAssembly(Base):
    """
    Combined Fuselage + Wing assembly.

    All design inputs are exposed at this level so that the ParaPy
    side-panel gives a single control panel for the whole vehicle.

    Fuselage-derived quantities (L_fus, structural CoM) are forwarded
    to the Wing via @Attribute wiring — no hardcoded coupling values.

    INPUTS
    ------
    Mission / sizing (shared by both subsystems):
      mission_q_max  [Pa]    max dynamic pressure
      mission_M_max  [-]     max Mach number
      mission_MTOW   [kg]    MTOW seed

    Fuselage-specific:
      mission_m_ox   [kg]    oxidiser mass (from MissionAnalysis)
      mission_m_fu   [kg]    fuel mass     (from MissionAnalysis)
      tanks_L_tank_ox [m]   oxidiser tank length (from PropellantTanks)
      tanks_L_tank_fu [m]   fuel tank length     (from PropellantTanks)
      tanks_D_inner   [m]   inner diameter from tanks
      payload_mass   [kg]    payload mass (UserInputs)
      payload_volume [m^3]   payload volume (UserInputs)
      ... all fuselage constants ...

    Wing-specific:
      AIRFOIL_TC     [-]     t/c ratio
      Lambda_LE_deg  [deg]   leading-edge sweep
      AR             [-]     aspect ratio
      lambda_t       [-]     taper ratio
      N_DESIGN       [-]     design load factor
      SM_TARGET      [-]     static margin target
    """

    # ── Shared mission inputs ─────────────────────────────────────────────────
    mission_q_max   = Input(_IF.mission_q_max,
        doc="Max dynamic pressure [Pa]  (MissionAnalysis)")
    mission_M_max   = Input(_IW.mission_M_max,
        doc="Max Mach number  (MissionAnalysis)")
    mission_MTOW    = Input(_IF.mission_MTOW,
        doc="MTOW estimate [kg]  (WeightAndCG seed)")

    # ── Fuselage-specific inputs ──────────────────────────────────────────────
    mission_m_ox    = Input(_IF.mission_m_ox,
        doc="Oxidiser mass [kg]  (MissionAnalysis)")
    mission_m_fu    = Input(_IF.mission_m_fu,
        doc="Fuel mass [kg]  (MissionAnalysis)")

    tanks_L_tank_ox = Input(_IF.tanks_L_tank_ox,
        doc="Oxidiser tank length [m]  (PropellantTanks)")
    tanks_L_tank_fu = Input(_IF.tanks_L_tank_fu,
        doc="Fuel tank length [m]  (PropellantTanks)")
    tanks_D_inner   = Input(_IF.tanks_D_inner,
        doc="Inner fuselage diameter from tanks [m]  (PropellantTanks)")

    payload_mass    = Input(_IF.payload_mass,
        doc="Payload mass [kg]  (UserInputs)")
    payload_volume  = Input(_IF.payload_volume,
        doc="Payload volume [m^3]  (UserInputs)")

    # Fuselage constants
    ENGINE_DIAM     = Input(_IF.ENGINE_DIAM,  doc="Engine outer diameter [m]")
    ENGINE_LEN      = Input(_IF.ENGINE_LEN,   doc="Engine overall length [m]")
    AVIONICS_LEN    = Input(_IF.AVIONICS_LEN, doc="Avionics bay length [m] (fixed)")
    AVIONICS_DIAM   = Input(_IF.AVIONICS_DIAM,doc="Avionics outer diameter [m]")
    AVIONICS_MASS   = Input(_IF.AVIONICS_MASS,doc="Avionics system mass [kg]")
    K_FUS           = Input(_IF.K_FUS,        doc="Fuselage structural mass fraction const")
    LAMBDA_FUSE     = Input(_IF.LAMBDA_FUSE,  doc="Fuselage slenderness L/D")
    K_NOSE          = Input(_IF.K_NOSE,       doc="Nose cone length factor k*D")
    K_TAIL          = Input(_IF.K_TAIL,       doc="Boat-tail length factor")
    K_FILL          = Input(_IF.K_FILL,       doc="Payload bay fill factor")

    # ── Wing-specific inputs ──────────────────────────────────────────────────
    AIRFOIL_TC      = Input(_IW.AIRFOIL_TC,
        doc="Thickness-to-chord ratio t/c  (NACA 64A-005)")
    Lambda_LE_deg   = Input(_IW.Lambda_LE_deg,
        doc="Leading-edge sweep [deg]  (60 deg for M>2, Raymer §12.4)")
    AR              = Input(_IW.AR,
        doc="Aspect ratio  (supersonic optimum 2.5, Raymer §12.5)")
    lambda_t        = Input(_IW.lambda_t,
        doc="Taper ratio  (near-elliptic 0.35)")
    N_DESIGN        = Input(_IW.N_DESIGN,
        doc="Design load factor  (CS-23 pull-out)")
    SM_TARGET       = Input(_IW.SM_TARGET,
        doc="Static-margin target as fraction of MAC")
    G0              = Input(_IW.G0,
        doc="Standard gravity [m/s^2]")

    # ── Fuselage sub-object ───────────────────────────────────────────────────

    @Part
    def fuselage(self):
        """
        Fuselage: nose cone + barrel + boat-tail + avionics box + payload box.
        All fuselage inputs forwarded from assembly-level Inputs.
        """
        return Fuselage(
            mission_q_max    = self.mission_q_max,
            mission_MTOW     = self.mission_MTOW,
            mission_m_ox     = self.mission_m_ox,
            mission_m_fu     = self.mission_m_fu,
            tanks_L_tank_ox  = self.tanks_L_tank_ox,
            tanks_L_tank_fu  = self.tanks_L_tank_fu,
            tanks_D_inner    = self.tanks_D_inner,
            payload_mass     = self.payload_mass,
            payload_volume   = self.payload_volume,
            ENGINE_DIAM      = self.ENGINE_DIAM,
            ENGINE_LEN       = self.ENGINE_LEN,
            AVIONICS_LEN     = self.AVIONICS_LEN,
            AVIONICS_DIAM    = self.AVIONICS_DIAM,
            AVIONICS_MASS    = self.AVIONICS_MASS,
            K_FUS            = self.K_FUS,
            LAMBDA_FUSE      = self.LAMBDA_FUSE,
            K_NOSE           = self.K_NOSE,
            K_TAIL           = self.K_TAIL,
            K_FILL           = self.K_FILL,
            label            = "Fuselage",
        )

    # ── Fuselage→Wing coupling attributes ─────────────────────────────────────

    @Attribute
    def _fus_L_fus(self):
        """
        Total fuselage length forwarded to Wing [m].
        Live reference — any change to fuselage geometry propagates instantly.
        """
        return self.fuselage.L_fus

    @Attribute
    def _x_cg_seed(self):
        """
        Centre-of-mass seed for wing positioning [m].
        Uses fuselage structural shell CoM (midpoint) as a first-pass x_cg
        before the full WeightAndCG module is available.
        This reflects the spec §9.4 static-margin loop seed.
        """
        return self.fuselage.xcm_fus

    # ── Wing sub-object ───────────────────────────────────────────────────────

    @Part
    def wing(self):
        """
        Wing: starboard and port half-surfaces sized from shared mission
        inputs and live fuselage geometry.

        KEY COUPLING (spec §9.4):
          Wing.fus_L_fus  <── Fuselage.L_fus   (live, not hardcoded)
          Wing.x_cg       <── Fuselage.xcm_fus  (structural CoM seed)
          Wing.mission_q_max == assembly.mission_q_max  (same value)

        Change payload_volume → fuselage grows/shrinks → L_fus changes
        → wing repositioned automatically to maintain static margin.
        """
        return Wing(
            mission_q_max  = self.mission_q_max,
            mission_M_max  = self.mission_M_max,
            mission_MTOW   = self.mission_MTOW,
            x_cg           = self._x_cg_seed,
            fus_L_fus      = self._fus_L_fus,
            AIRFOIL_TC     = self.AIRFOIL_TC,
            Lambda_LE_deg  = self.Lambda_LE_deg,
            AR             = self.AR,
            lambda_t       = self.lambda_t,
            N_DESIGN       = self.N_DESIGN,
            SM_TARGET      = self.SM_TARGET,
            G0             = self.G0,
            label          = "Wing",
        )

    # ── Assembly-level derived quantities ─────────────────────────────────────

    @Attribute
    def xcg_combined(self):
        """
        First-pass combined CoM [m] weighting fuselage structural mass
        and wing structural mass (payload and propellant CoMs are
        approximated at zone midpoints from the fuselage).
        Full WeightAndCG iteration is in weight_cg.py (spec §8).
        """
        m_fus  = self.fuselage.m_fus
        xcm_f  = self.fuselage.xcm_fus
        m_wing = self.wing.m_wing_struct
        xcm_w  = self.wing.xcm_wing
        m_avi  = self.AVIONICS_MASS
        xcm_a  = self.fuselage.xcm_avionics
        m_pl   = self.payload_mass
        xcm_pl = self.fuselage.xcm_payload

        total_m = m_fus + m_wing + m_avi + m_pl
        x_cg    = (m_fus * xcm_f + m_wing * xcm_w
                   + m_avi * xcm_a + m_pl * xcm_pl) / total_m
        return x_cg

    @Attribute
    def static_margin_approx(self):
        """
        Approximate static margin (fraction of MAC).
        x_ac estimated at 25% MAC of wing (simplified, body lift ignored).
        """
        x_ac = self.wing.x_wing_le + 0.25 * self.wing.MAC
        return (x_ac - self.xcg_combined) / self.wing.MAC

    # ── Console summary ────────────────────────────────────────────────────────

    def print_summary(self):
        """Print a combined sizing summary."""
        self.fuselage.print_summary()
        self.wing.print_summary()
        div = "=" * 64
        print("\n" + div)
        print("  ASSEMBLY SUMMARY")
        print(div)
        print("  {:<30s}= {:>10.4f}  m  ".format("Fuselage L_fus",   self.fuselage.L_fus))
        print("  {:<30s}= {:>10.4f}  m  ".format("Fuselage D_fus",   self.fuselage.D_fus))
        print("  {:<30s}= {:>10.4f}  m  ".format("Wing x_wing_LE",   self.wing.x_wing_le))
        print("  {:<30s}= {:>10.4f}  m  ".format("Wing b (span)",    self.wing.b))
        print("  {:<30s}= {:>10.4f}  m  ".format("Wing MAC",         self.wing.MAC))
        print("  {:<30s}= {:>10.4f}  m  ".format("x_cg combined",    self.xcg_combined))
        print("  {:<30s}= {:>10.4f}     ".format("SM approx (×MAC)", self.static_margin_approx))
        print("  {:<30s}= {:>10.3f}  kg ".format("m_fus",            self.fuselage.m_fus))
        print("  {:<30s}= {:>10.3f}  kg ".format("m_wing_struct",    self.wing.m_wing_struct))
        print("  {:<30s}= {:>10.3f}  kg ".format("m_avionics",       self.AVIONICS_MASS))
        print("  {:<30s}= {:>10.3f}  kg ".format("payload_mass",     self.payload_mass))
        print(div)
        print("  NOTE: x_cg is a structural seed.  Run weight_cg.py for full MTOW loop.")
        print(div + "\n")


# ==============================================================================
# Entry point
# ==============================================================================

if __name__ == "__main__":
    assembly = FuselageWingAssembly(label="Spaceplane")
    assembly.print_summary()
    display(assembly)
