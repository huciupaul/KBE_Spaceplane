# wing.py  --  Standalone Wing Module
# =====================================
# Implements section 9 of the Spaceplane Concept Design Spec (Rev 1.0).
#
# Self-contained: reads all inputs from inputs_wing.py.
# No other spaceplane modules required.
#
# Wing structural mass:
#   Raymer Eq. 15.25 semi-empirical seed (spec section 9.5).
#   Replace with AVL structural call once avl_interface.py exists.
#
# AVL path (for future integration):
#   AVL_EXE = r"C:/Users/huciu/Desktop/avl_downloaded/AVL3.52rel09032025/AVL352.exe"
#
# Running:
#   python wing.py
#   The ParaPy viewer opens. Every Input is editable in the side-panel.
#   Change a value, press Enter -> geometry rebuilds instantly.
#
# Coordinate system:
#   X -> downstream (nose to tail)
#   Y -> starboard
#   Z -> up
# All lengths in metres, masses in kilograms.

from __future__ import annotations
import math
import importlib

# -- ParaPy core ---------------------------------------------------------------
from parapy.core import Base, Input, Attribute, Part
from parapy.geom import (
    BSplineCurve,
    LoftedSurface,
    Point,
)
from parapy.gui import display

# -- Load mock inputs ----------------------------------------------------------
import inputs_wing as _IW


# ------------------------------------------------------------------------------
# Section profile helper
# ------------------------------------------------------------------------------

def _diamond_section(chord, tc, x_le, y):
    """
    Return 5 Points forming a closed symmetric diamond aerofoil cross-section
    in the XZ-plane at spanwise coordinate y.

    Profile order: LE -> upper max-t -> TE -> lower max-t -> LE (closed).
    Suitable as BSplineCurve control_points for lofting.
    """
    t_half = 0.5 * tc * chord
    return [
        Point(x_le,                 y,  0.0),     # LE
        Point(x_le + 0.15 * chord,  y,  t_half),  # upper max-thickness
        Point(x_le + chord,         y,  0.0),     # TE
        Point(x_le + 0.15 * chord,  y, -t_half),  # lower max-thickness
        Point(x_le,                 y,  0.0),     # LE again (closed)
    ]


# ------------------------------------------------------------------------------
# Wing-half surface sub-object
# ------------------------------------------------------------------------------

class _WingHalf(Base):
    """
    One trapezoidal wing half-surface, lofted from a root BSpline section
    to a tip BSpline section.
    """

    c_root    = Input()       # root chord [m]
    c_tip     = Input()       # tip chord  [m]
    semi_span = Input()       # half-span  [m]
    tip_le_x  = Input()       # LE x-offset at tip (due to sweep) [m]
    x_root_le = Input()       # root LE x-position from nose [m]
    tc        = Input()       # thickness-to-chord ratio
    port      = Input(False)  # True -> port side (negative Y)

    @Attribute
    def _y_tip(self):
        return -self.semi_span if self.port else self.semi_span

    @Attribute
    def _root_ctrl_pts(self):
        return _diamond_section(self.c_root, self.tc, self.x_root_le, 0.0)

    @Attribute
    def _tip_ctrl_pts(self):
        return _diamond_section(self.c_tip, self.tc,
                                self.x_root_le + self.tip_le_x,
                                self._y_tip)

    @Part
    def root_curve(self):
        # 'control_points' is the correct kwarg in this ParaPy build
        return BSplineCurve(control_points=self._root_ctrl_pts,
                            label="root_section")

    @Part
    def tip_curve(self):
        return BSplineCurve(control_points=self._tip_ctrl_pts,
                            label="tip_section")

    @Part
    def surface(self):
        # LoftedSurface takes a list called 'profiles'
        return LoftedSurface(profiles=[self.root_curve, self.tip_curve],
                             label=self.label)


# ------------------------------------------------------------------------------
# Main Wing class
# ------------------------------------------------------------------------------

class Wing(Base):
    """
    Trapezoidal delta wing sized per Spaceplane Design Spec section 9.

    All inputs default to the values in inputs_wing.py.  Change them
    interactively in the ParaPy side-panel to explore the design space.
    """

    # -- Inputs ----------------------------------------------------------------

    mission_q_max = Input(_IW.mission_q_max,
        doc="Max dynamic pressure q_max [Pa]  (MissionAnalysis)")
    mission_M_max = Input(_IW.mission_M_max,
        doc="Max Mach number M_max  (MissionAnalysis)")
    mission_MTOW  = Input(_IW.mission_MTOW,
        doc="MTOW estimate [kg]  (WeightAndCG / iteration seed)")

    x_cg          = Input(_IW.x_cg,
        doc="Centre-of-gravity x from nose [m]  (WeightAndCG)")
    fus_L_fus     = Input(_IW.fus_L_fus,
        doc="Total fuselage length [m]  (Fuselage)")

    AIRFOIL_TC    = Input(_IW.AIRFOIL_TC,
        doc="Thickness-to-chord ratio t/c  (NACA 64A-005 = 0.05)")
    Lambda_LE_deg = Input(_IW.Lambda_LE_deg,
        doc="Leading-edge sweep angle [deg]  (60 deg for M>2, Raymer 12.4)")
    AR            = Input(_IW.AR,
        doc="Aspect ratio  (supersonic optimum 2.5, Raymer 12.5)")
    lambda_t      = Input(_IW.lambda_t,
        doc="Taper ratio  (near-elliptic 0.35, Raymer Ch.6)")
    N_DESIGN      = Input(_IW.N_DESIGN,
        doc="Design load factor  (CS-23 pull-out)")
    SM_TARGET     = Input(_IW.SM_TARGET,
        doc="Static-margin target as fraction of MAC  (Raymer 16.2)")
    G0            = Input(_IW.G0,
        doc="Standard gravity [m/s^2]")

    # -- Derived sizing attributes ---------------------------------------------

    @Attribute
    def Lambda_LE(self):
        """Leading-edge sweep [rad]."""
        return math.radians(self.Lambda_LE_deg)

    @Attribute
    def CL_alpha_sup(self):
        """
        Lift-curve slope [1/rad]:
          Supersonic  M > 1 : Ackeret  4/sqrt(M^2-1)       (Anderson Ch.14)
          Subsonic    M < 1 : Prandtl-Glauert  2*pi/beta    (Anderson 11.4)
        """
        M = self.mission_M_max
        if M > 1.0:
            return 4.0 / math.sqrt(M ** 2 - 1.0)
        beta = math.sqrt(max(1e-9, 1.0 - M ** 2))
        return 2.0 * math.pi / beta

    @Attribute
    def alpha_max(self):
        """Maximum incidence 10 deg [rad]  (spec 9.2)."""
        return math.radians(10.0)

    @Attribute
    def CL_max_comp(self):
        """Compressibility-corrected CL_max  (spec 9.2)."""
        return self.CL_alpha_sup * self.alpha_max

    @Attribute
    def L_required(self):
        """Required lift at design load factor [N]  (spec 9.2)."""
        return self.N_DESIGN * self.mission_MTOW * self.G0

    @Attribute
    def S_ref(self):
        """Reference wing area [m^2]  (spec 9.2)."""
        return self.L_required / (self.mission_q_max * self.CL_max_comp)

    @Attribute
    def b(self):
        """Wing span [m]  (spec 9.3)."""
        return math.sqrt(self.AR * self.S_ref)

    @Attribute
    def c_root(self):
        """Root chord [m]  (trapezoidal planform, spec 9.3)."""
        return 2.0 * self.S_ref / (self.b * (1.0 + self.lambda_t))

    @Attribute
    def c_tip(self):
        """Tip chord [m]  (spec 9.3)."""
        return self.lambda_t * self.c_root

    @Attribute
    def MAC(self):
        """Mean aerodynamic chord [m]  (spec 9.3)."""
        lt = self.lambda_t
        return (2.0 / 3.0) * self.c_root * (1.0 + lt + lt ** 2) / (1.0 + lt)

    @Attribute
    def y_MAC(self):
        """Spanwise MAC position [m] from root  (spec 9.3)."""
        lt = self.lambda_t
        return (self.b / 6.0) * (1.0 + 2.0 * lt) / (1.0 + lt)

    @Attribute
    def tip_le_x(self):
        """X-offset of tip LE from root LE due to sweep [m]."""
        return (self.b / 2.0) * math.tan(self.Lambda_LE)

    @Attribute
    def x_wing_le(self):
        """
        Root leading-edge x-position from nose [m], solved from the
        static-margin condition (spec 9.4):

            x_ac - x_cg = SM_TARGET * MAC

        Combined AC with 10 % body-lift contribution:
            x_ac       = 0.9 * x_ac_wing + 0.1 * (0.40 * L_fus)
            x_ac_wing  = x_wing_le + y_MAC*tan(Lambda_LE) + 0.25*MAC

        Rearranged:
            x_wing_le = [(x_cg + SM*MAC) - f*x_ac_body] / (1-f) - offset
        """
        f        = 0.10
        x_ac_body = 0.40 * self.fus_L_fus
        offset   = self.y_MAC * math.tan(self.Lambda_LE) + 0.25 * self.MAC
        x_le     = ((self.x_cg + self.SM_TARGET * self.MAC
                     - f * x_ac_body) / (1.0 - f)) - offset
        # Physical guard: keep wing inside fuselage
        x_le_min = 0.05 * self.fus_L_fus
        x_le_max = 0.85 * self.fus_L_fus - self.c_root
        return max(x_le_min, min(x_le, x_le_max))

    @Attribute
    def xcm_wing(self):
        """Wing structural CoM x from nose [m]  (40 % MAC rule, spec 9.6)."""
        return self.x_wing_le + 0.40 * self.MAC

    @Attribute
    def m_wing_struct(self):
        """
        Wing structural mass [kg] -- Raymer Eq. 15.25, modified for
        unmanned/composite (spec 9.5 seed formula):

            m = 0.036 * S^0.758 * (AR/cos^2(Lambda))^0.6
                      * q^0.006 * (100*t/c)^(-0.3)
        """
        S  = self.S_ref
        A  = self.AR / math.cos(self.Lambda_LE) ** 2
        q  = self.mission_q_max
        tc = self.AIRFOIL_TC
        return 0.036 * S ** 0.758 * A ** 0.6 * q ** 0.006 * (100.0 * tc) ** (-0.3)

    # -- 3-D geometry ----------------------------------------------------------

    @Part
    def starboard(self):
        """Starboard (right) wing half-surface."""
        return _WingHalf(
            c_root    = self.c_root,
            c_tip     = self.c_tip,
            semi_span = self.b / 2.0,
            tip_le_x  = self.tip_le_x,
            x_root_le = self.x_wing_le,
            tc        = self.AIRFOIL_TC,
            port      = False,
            label     = "starboard_wing",
        )

    @Part
    def port(self):
        """Port (left) wing half-surface."""
        return _WingHalf(
            c_root    = self.c_root,
            c_tip     = self.c_tip,
            semi_span = self.b / 2.0,
            tip_le_x  = self.tip_le_x,
            x_root_le = self.x_wing_le,
            tc        = self.AIRFOIL_TC,
            port      = True,
            label     = "port_wing",
        )

    # -- Console summary -------------------------------------------------------

    def print_summary(self):
        """Print a formatted wing sizing summary to stdout."""
        div = "=" * 62
        print("\n" + div)
        print("  WING SIZING SUMMARY  --  spec section 9  (Rev 1.0)")
        print(div)
        print("  {:<22s}= {:>10.4f}  m^2".format("S_ref",          self.S_ref))
        print("  {:<22s}= {:>10.4f}  m  ".format("b (span)",       self.b))
        print("  {:<22s}= {:>10.2f}     ".format("AR",             self.AR))
        print("  {:<22s}= {:>10.2f}     ".format("lambda_t",       self.lambda_t))
        print("  {:<22s}= {:>10.1f}  deg".format("Lambda_LE",      self.Lambda_LE_deg))
        print("  {:<22s}= {:>10.4f}  m  ".format("c_root",         self.c_root))
        print("  {:<22s}= {:>10.4f}  m  ".format("c_tip",          self.c_tip))
        print("  {:<22s}= {:>10.4f}  m  ".format("MAC",            self.MAC))
        print("  {:<22s}= {:>10.4f}  m  ".format("y_MAC",          self.y_MAC))
        print("  {:<22s}= {:>10.4f}  m  ".format("x_wing_LE",      self.x_wing_le))
        print("  {:<22s}= {:>10.4f}  m  ".format("xcm_wing",       self.xcm_wing))
        print("  {:<22s}= {:>10.3f}  kg ".format("m_wing_struct",  self.m_wing_struct))
        print("  {:<22s}= {:>10.4f}     ".format("CL_max_comp",    self.CL_max_comp))
        print("  {:<22s}= {:>10.1f}  N  ".format("L_required",     self.L_required))
        print(div)
        print("  m_wing_struct: Raymer Eq. 15.25 semi-empirical seed.")
        print(div + "\n")


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------

if __name__ == "__main__":
    wing = Wing(label="Wing")
    wing.print_summary()
    display(wing)