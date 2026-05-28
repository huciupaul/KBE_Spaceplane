# fuselage.py  --  Standalone Fuselage Module
# ============================================
# Implements section 6 of the Spaceplane Concept Design Spec (Rev 1.0).
#
# Self-contained: reads all inputs from inputs_fuselage.py.
# No other spaceplane modules required.
#
# ParaPy @Part grammar rule (strictly enforced by the framework parser):
#   Every @Part function body MUST be a single return statement.
#   All multi-step logic lives in @Attribute methods instead.
#
# Running:
#   python fuselage.py
#   The ParaPy viewer opens. Every Input is editable in the side-panel.
#   Change a value, press Enter -> geometry rebuilds instantly.
#
# Coordinate system:
#   X -> downstream (nose tip = 0)
#   Y -> starboard
#   Z -> up
# All lengths in metres, masses in kilograms.

from __future__ import annotations
import math

# -- ParaPy core ---------------------------------------------------------------
from parapy.core import Base, Input, Attribute, Part
from parapy.geom import (
    BSplineCurve,
    LoftedSurface,
    Box,
    Point,
)
from parapy.gui import display

# -- Load mock inputs ----------------------------------------------------------
import inputs_fuselage as _IF


# ------------------------------------------------------------------------------
# Helper: circular cross-section control points at axial position x_pos
# ------------------------------------------------------------------------------

def _circle_points(x_pos: float, radius: float, n: int = 9):
    """
    Return n Points forming a closed circular cross-section in the YZ-plane
    at axial position x_pos.  n=9 gives a good B-spline circle approximation.
    Last point == first point so the curve is closed.
    """
    pts = []
    for i in range(n):
        angle = 2.0 * math.pi * i / (n - 1)
        pts.append(Point(x_pos,
                         radius * math.cos(angle),
                         radius * math.sin(angle)))
    return pts


# ------------------------------------------------------------------------------
# Nose Cone sub-object  (Von Kármán ogive approximation)
# ------------------------------------------------------------------------------

class _NoseCone(Base):
    """
    Von Kármán-style ogive nose cone lofted through n_sects cross-sections.

    Kármán-Haack radius distribution:
        theta(x) : x = L*(1 - cos(theta))/2,  theta in [0, pi]
        r(theta) = R * sqrt((theta - 0.5*sin(2*theta)) / pi)
    """
    L_nose  = Input()       # nose length [m]
    R_fus   = Input()       # fuselage outer radius at base [m]
    n_sects = Input(6)      # number of intermediate loft sections

    @Attribute
    def _profile_curves(self):
        """
        Build all BSplineCurve cross-sections from tip to base.
        Stored as an Attribute so @Part surface has a single return.
        """
        curves = []
        for i in range(self.n_sects):
            theta = math.pi * (i + 0.5) / self.n_sects   # avoid exact 0
            x_pos = self.L_nose * (1.0 - math.cos(theta)) / 2.0
            r_k   = self.R_fus * math.sqrt(
                        (theta - 0.5 * math.sin(2.0 * theta)) / math.pi)
            pts = _circle_points(x_pos, max(r_k, 1e-4))
            curves.append(BSplineCurve(control_points=pts,
                                       label="nose_sect_%d" % i))
        # Exact base circle closes the loft at the barrel junction
        pts_base = _circle_points(self.L_nose, self.R_fus)
        curves.append(BSplineCurve(control_points=pts_base,
                                   label="nose_base"))
        return curves

    @Part
    def surface(self):
        return LoftedSurface(profiles=self._profile_curves,
                             label="nose_cone")


# ------------------------------------------------------------------------------
# Cylindrical Barrel sub-object
# ------------------------------------------------------------------------------

class _Barrel(Base):
    """Straight cylindrical fuselage barrel lofted between two circle sections."""
    x_start = Input()
    x_end   = Input()
    R_fus   = Input()

    # -- Forward circle --------------------------------------------------------
    @Attribute
    def _pts_fwd(self):
        return _circle_points(self.x_start, self.R_fus)

    @Part
    def fwd_curve(self):
        return BSplineCurve(control_points=self._pts_fwd,
                            label="barrel_fwd")

    # -- Aft circle ------------------------------------------------------------
    @Attribute
    def _pts_aft(self):
        return _circle_points(self.x_end, self.R_fus)

    @Part
    def aft_curve(self):
        return BSplineCurve(control_points=self._pts_aft,
                            label="barrel_aft")

    # -- Lofted surface --------------------------------------------------------
    @Part
    def surface(self):
        return LoftedSurface(profiles=[self.fwd_curve, self.aft_curve],
                             label="barrel_surface")


# ------------------------------------------------------------------------------
# Boat-Tail sub-object
# ------------------------------------------------------------------------------

class _BoatTail(Base):
    """
    Conical aft closure tapering from fuselage radius to engine nozzle radius.
    """
    x_start  = Input()
    L_tail   = Input()
    R_fus    = Input()
    R_engine = Input()

    @Attribute
    def x_end(self):
        return self.x_start + self.L_tail

    # -- Forward (wide) circle -------------------------------------------------
    @Attribute
    def _pts_fwd(self):
        return _circle_points(self.x_start, self.R_fus)

    @Part
    def fwd_curve(self):
        return BSplineCurve(control_points=self._pts_fwd,
                            label="tail_fwd")

    # -- Aft (narrow) circle ---------------------------------------------------
    @Attribute
    def _pts_aft(self):
        return _circle_points(self.x_end, self.R_engine)

    @Part
    def aft_curve(self):
        return BSplineCurve(control_points=self._pts_aft,
                            label="tail_aft")

    # -- Lofted surface --------------------------------------------------------
    @Part
    def surface(self):
        return LoftedSurface(profiles=[self.fwd_curve, self.aft_curve],
                             label="boat_tail")


# ------------------------------------------------------------------------------
# Interior Box sub-object  (avionics bay and payload bay)
# ------------------------------------------------------------------------------

class _InteriorBox(Base):
    """
    Rectangular box centred on the fuselage axis (Y=0, Z=0) representing an
    interior zone.  Positioned so its forward face is at x_start.

    ParaPy Box valid kwargs (from Box._inputs.keys()):
        width    -> X extent (streamwise)
        length   -> Y extent
        height   -> Z extent
        centered -> if True, position is the box centre; no manual corner needed
        position -> placement point

    With centered=True we place the box centre at the zone midpoint on the axis.
    """
    x_start    = Input()
    box_length = Input()   # X extent [m]
    box_width  = Input()   # Y extent [m]
    box_height = Input()   # Z extent [m]

    # Centre point: midpoint along X, on the fuselage axis (Y=Z=0).
    # Stored as an Attribute so @Part box remains a single return statement.
    @Attribute
    def _centre(self):
        return Point(self.x_start + self.box_length / 2.0, 0.0, 0.0)

    @Part
    def box(self):
        return Box(width    = self.box_length,
                   length   = self.box_width,
                   height   = self.box_height,
                   centered = True,
                   position = self._centre,
                   label    = self.label)


# ------------------------------------------------------------------------------
# Main Fuselage class
# ------------------------------------------------------------------------------

class Fuselage(Base):
    """
    Cylindrical fuselage with Von Kármán nose, constant barrel,
    boat-tail closure, and interior avionics/payload boxes.

    Sized per Spaceplane Design Spec section 6 (Rev 1.0).
    All inputs default to values in inputs_fuselage.py.

    OUTPUTS CONSUMED BY WING (wired live in main_fuselage_wing.py):
        L_fus    -- total fuselage length [m]
        D_fus    -- outer fuselage diameter [m]
        xcm_fus  -- structural shell CoM [m]  (= L_fus / 2 seed)
    """

    # ── Mission inputs ────────────────────────────────────────────────────────
    mission_q_max   = Input(_IF.mission_q_max,
        doc="Max dynamic pressure [Pa]  (MissionAnalysis)")
    mission_MTOW    = Input(_IF.mission_MTOW,
        doc="MTOW estimate [kg]  (iteration seed)")
    mission_m_ox    = Input(_IF.mission_m_ox,
        doc="Oxidiser mass [kg]  (MissionAnalysis)")
    mission_m_fu    = Input(_IF.mission_m_fu,
        doc="Fuel mass [kg]  (MissionAnalysis)")

    # ── Propellant tank inputs ────────────────────────────────────────────────
    tanks_L_tank_ox = Input(_IF.tanks_L_tank_ox,
        doc="Oxidiser tank length [m]  (PropellantTanks)")
    tanks_L_tank_fu = Input(_IF.tanks_L_tank_fu,
        doc="Fuel tank length [m]  (PropellantTanks)")
    tanks_D_inner   = Input(_IF.tanks_D_inner,
        doc="Inner fuselage diameter from tanks [m]  (PropellantTanks)")

    # ── Payload inputs ────────────────────────────────────────────────────────
    payload_mass    = Input(_IF.payload_mass,
        doc="Payload mass [kg]  (UserInputs)")
    payload_volume  = Input(_IF.payload_volume,
        doc="Payload volume [m^3]  (UserInputs)")

    # ── Constants ─────────────────────────────────────────────────────────────
    ENGINE_DIAM     = Input(_IF.ENGINE_DIAM,
        doc="Engine outer diameter [m]  (constants.py)")
    ENGINE_LEN      = Input(_IF.ENGINE_LEN,
        doc="Engine overall length [m]  (constants.py)")
    AVIONICS_LEN    = Input(_IF.AVIONICS_LEN,
        doc="Avionics bay length [m]  (fixed, constants.py)")
    AVIONICS_DIAM   = Input(_IF.AVIONICS_DIAM,
        doc="Avionics outer diameter [m]  (constants.py)")
    AVIONICS_MASS   = Input(_IF.AVIONICS_MASS,
        doc="Avionics system mass [kg]  (constants.py)")
    SIGMA_AL        = Input(_IF.SIGMA_AL,
        doc="Allowable stress [Pa]  (constants.py)")
    K_FUS           = Input(_IF.K_FUS,
        doc="Fuselage structural mass fraction const  (Raymer Eq. 15.46)")
    LAMBDA_FUSE     = Input(_IF.LAMBDA_FUSE,
        doc="Fuselage slenderness L/D  (Aurora geometry)")
    K_NOSE          = Input(_IF.K_NOSE,
        doc="Nose cone length factor k*D  (Raymer §6.3)")
    K_TAIL          = Input(_IF.K_TAIL,
        doc="Boat-tail length factor  (Raymer §6.3)")
    K_FILL          = Input(_IF.K_FILL,
        doc="Payload bay volumetric fill factor")

    # ── Diameter sizing  (spec §6.2) ──────────────────────────────────────────

    @Attribute
    def t_wall_initial(self):
        """Initial wall thickness estimate [m] (engine diameter seed)."""
        return (self.mission_q_max * (self.ENGINE_DIAM / 2.0)) / (2.0 * self.SIGMA_AL)

    @Attribute
    def D_engine_min(self):
        """Min inner diameter from engine envelope [m]."""
        return self.ENGINE_DIAM + 2.0 * self.t_wall_initial

    @Attribute
    def D_payload_min(self):
        """
        Min inner diameter from payload volume [m]  (spec §6.2).
        Assumes a bay length-to-diameter ratio of 2 for the initial estimate.
        """
        return (4.0 * self.payload_volume / (math.pi * self.K_FILL)) ** (1.0 / 3.0)

    @Attribute
    def D_fus_inner(self):
        """Inner fuselage diameter [m] — max of three constraints (spec §6.2)."""
        return max(self.D_engine_min, self.D_payload_min, self.tanks_D_inner)

    @Attribute
    def t_wall(self):
        """Wall thickness [m]  (Shigley Eq. 3-68, spec §6.2)."""
        return (self.mission_q_max * (self.D_fus_inner / 2.0)) / (2.0 * self.SIGMA_AL)

    @Attribute
    def D_fus(self):
        """Outer fuselage diameter [m]."""
        return self.D_fus_inner + 2.0 * self.t_wall

    @Attribute
    def R_fus(self):
        """Outer fuselage radius [m]."""
        return self.D_fus / 2.0

    # ── Zone lengths  (spec §6.3) ─────────────────────────────────────────────

    @Attribute
    def L_nose(self):
        """Von Kármán ogive nose length [m]."""
        return self.K_NOSE * self.D_fus

    @Attribute
    def L_avionics(self):
        """Avionics bay length [m]  (fixed constant)."""
        return self.AVIONICS_LEN

    @Attribute
    def L_payload(self):
        """Payload bay length [m]  (from payload volume, spec §6.3)."""
        A_bay = math.pi * (self.D_fus_inner / 2.0) ** 2
        return self.payload_volume / A_bay

    @Attribute
    def L_ox_tank(self):
        """Oxidiser tank length [m]  (from PropellantTanks)."""
        return self.tanks_L_tank_ox

    @Attribute
    def L_fu_tank(self):
        """Fuel tank length [m]  (from PropellantTanks)."""
        return self.tanks_L_tank_fu

    @Attribute
    def L_engine(self):
        """Engine bay length [m]  (engine + 50 mm clearance, spec §6.3)."""
        return self.ENGINE_LEN + 0.05

    @Attribute
    def L_tail(self):
        """Boat-tail length [m]  (spec §6.3)."""
        return self.K_TAIL * self.D_fus

    @Attribute
    def L_fus_raw(self):
        """Sum of all zone lengths before slenderness check [m]."""
        return (self.L_nose + self.L_avionics + self.L_payload
                + self.L_ox_tank + self.L_fu_tank + self.L_engine + self.L_tail)

    @Attribute
    def L_fus(self):
        """
        Total fuselage length [m] with slenderness enforcement.
        If |L/D - LAMBDA_FUSE| > 0.5, clamp to LAMBDA_FUSE * D_fus (spec §6.3).
        """
        l_raw = self.L_fus_raw
        l_target = self.LAMBDA_FUSE * self.D_fus
        if abs(l_raw / self.D_fus - self.LAMBDA_FUSE) > 0.5:
            return l_target
        return l_raw

    # ── Zone axial positions from nose tip  (spec §6.4) ───────────────────────

    @Attribute
    def x_nose_end(self):
        return self.L_nose

    @Attribute
    def x_avionics_start(self):
        return self.x_nose_end

    @Attribute
    def x_avionics_end(self):
        return self.x_nose_end + self.L_avionics

    @Attribute
    def x_payload_start(self):
        return self.x_avionics_end

    @Attribute
    def x_payload_end(self):
        return self.x_avionics_end + self.L_payload

    @Attribute
    def x_ox_start(self):
        return self.x_payload_end

    @Attribute
    def x_ox_end(self):
        return self.x_payload_end + self.L_ox_tank

    @Attribute
    def x_fu_start(self):
        return self.x_ox_end

    @Attribute
    def x_fu_end(self):
        return self.x_ox_end + self.L_fu_tank

    @Attribute
    def x_engine_start(self):
        return self.x_fu_end

    @Attribute
    def x_engine_end(self):
        return self.x_fu_end + self.L_engine

    @Attribute
    def x_tail_start(self):
        return self.x_engine_end

    # ── Zone CoM positions  (midpoint rule, spec §6.4) ────────────────────────

    @Attribute
    def xcm_avionics(self):
        return self.x_avionics_start + self.L_avionics / 2.0

    @Attribute
    def xcm_payload(self):
        return self.x_payload_start + self.L_payload / 2.0

    @Attribute
    def xcm_ox(self):
        return self.x_ox_start + self.L_ox_tank / 2.0

    @Attribute
    def xcm_fu(self):
        return self.x_fu_start + self.L_fu_tank / 2.0

    @Attribute
    def xcm_engine(self):
        return self.x_engine_start + self.L_engine / 2.0

    @Attribute
    def xcm_fus(self):
        """Structural shell CoM [m] — barrel midpoint seed."""
        return self.L_fus / 2.0

    # ── Structural mass  (spec §6.5) ──────────────────────────────────────────

    @Attribute
    def S_wet_fus(self):
        """Wetted area [m^2]  (cylinder approximation, spec §6.5)."""
        return math.pi * self.D_fus * self.L_fus

    @Attribute
    def m_fus(self):
        """
        Fuselage structural mass [kg]  (Raymer Eq. 15.46, spec §6.5):
            m_fus = K_FUS * q_max^0.5 * S_wet^1.1
        """
        return self.K_FUS * self.mission_q_max ** 0.5 * self.S_wet_fus ** 1.1

    # ── Payload-box side length as an Attribute (keeps @Part clean) ───────────

    @Attribute
    def _payload_box_side(self):
        """80 % of inner fuselage diameter — payload box cross-section [m]."""
        return 0.80 * self.D_fus_inner

    # ── 3-D geometry — every @Part is a single return statement ───────────────

    @Part
    def nose_cone(self):
        """Von Kármán ogive nose cone."""
        return _NoseCone(L_nose=self.L_nose,
                         R_fus=self.R_fus,
                         label="nose_cone")

    @Part
    def barrel(self):
        """Main cylindrical barrel from nose junction to tail junction."""
        return _Barrel(x_start=self.x_nose_end,
                       x_end=self.x_tail_start,
                       R_fus=self.R_fus,
                       label="fuselage_barrel")

    @Part
    def boat_tail(self):
        """Conical aft closure tapering to engine nozzle radius."""
        return _BoatTail(x_start=self.x_tail_start,
                         L_tail=self.L_tail,
                         R_fus=self.R_fus,
                         R_engine=self.ENGINE_DIAM / 2.0 * 0.9,
                         label="boat_tail")

    @Part
    def avionics_box(self):
        """
        Fixed avionics bay box  (spec §3.3).
        Dimensions: AVIONICS_DIAM × AVIONICS_DIAM × AVIONICS_LEN.
        Not user-editable — comes from equipment catalogue constants.
        """
        return _InteriorBox(x_start=self.x_avionics_start,
                            box_length=self.L_avionics,
                            box_width=self.AVIONICS_DIAM,
                            box_height=self.AVIONICS_DIAM,
                            label="avionics_bay")

    @Part
    def payload_box(self):
        """
        Payload bay box sized by payload_volume (user input).
        Cross-section = 80 % of inner fuselage diameter.
        """
        return _InteriorBox(x_start=self.x_payload_start,
                            box_length=self.L_payload,
                            box_width=self._payload_box_side,
                            box_height=self._payload_box_side,
                            label="payload_bay")

    # ── Console summary ────────────────────────────────────────────────────────

    def print_summary(self):
        """Print a formatted fuselage sizing summary to stdout."""
        div = "=" * 64
        print("\n" + div)
        print("  FUSELAGE SIZING SUMMARY  --  spec section 6  (Rev 1.0)")
        print(div)
        print("  {:<26s}= {:>10.4f}  m  ".format("D_fus_inner",       self.D_fus_inner))
        print("  {:<26s}= {:>10.4f}  m  ".format("t_wall",            self.t_wall))
        print("  {:<26s}= {:>10.4f}  m  ".format("D_fus (outer)",     self.D_fus))
        print("  {:<26s}= {:>10.4f}  m  ".format("L_fus",             self.L_fus))
        print("  {:<26s}= {:>10.2f}     ".format("L/D (slenderness)", self.L_fus / self.D_fus))
        print(div)
        print("  Zone lengths:")
        print("  {:<26s}= {:>10.4f}  m  ".format("  L_nose",       self.L_nose))
        print("  {:<26s}= {:>10.4f}  m  ".format("  L_avionics",   self.L_avionics))
        print("  {:<26s}= {:>10.4f}  m  ".format("  L_payload",    self.L_payload))
        print("  {:<26s}= {:>10.4f}  m  ".format("  L_ox_tank",    self.L_ox_tank))
        print("  {:<26s}= {:>10.4f}  m  ".format("  L_fu_tank",    self.L_fu_tank))
        print("  {:<26s}= {:>10.4f}  m  ".format("  L_engine",     self.L_engine))
        print("  {:<26s}= {:>10.4f}  m  ".format("  L_tail",       self.L_tail))
        print(div)
        print("  Zone axial starts (from nose tip):")
        print("  {:<26s}= {:>10.4f}  m  ".format("  x_avionics",   self.x_avionics_start))
        print("  {:<26s}= {:>10.4f}  m  ".format("  x_payload",    self.x_payload_start))
        print("  {:<26s}= {:>10.4f}  m  ".format("  x_ox_tank",    self.x_ox_start))
        print("  {:<26s}= {:>10.4f}  m  ".format("  x_fu_tank",    self.x_fu_start))
        print("  {:<26s}= {:>10.4f}  m  ".format("  x_engine",     self.x_engine_start))
        print("  {:<26s}= {:>10.4f}  m  ".format("  x_tail_start", self.x_tail_start))
        print(div)
        print("  Mass & wetted area:")
        print("  {:<26s}= {:>10.4f}  m^2".format("S_wet_fus",  self.S_wet_fus))
        print("  {:<26s}= {:>10.3f}  kg ".format("m_fus",      self.m_fus))
        print("  {:<26s}= {:>10.4f}  m  ".format("xcm_fus",    self.xcm_fus))
        print(div)
        print("  Outputs forwarded to Wing module:")
        print("  {:<26s}= {:>10.4f}  m  ".format("  fus_L_fus", self.L_fus))
        print("  {:<26s}= {:>10.4f}  m  ".format("  fus_D_fus", self.D_fus))
        print(div + "\n")


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------

if __name__ == "__main__":
    fus = Fuselage(label="Fuselage")
    fus.print_summary()
    display(fus)