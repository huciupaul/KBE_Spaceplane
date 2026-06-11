"""
fuselage.py

Parametric fuselage for the suborbital research spaceplane KBE application.

Geometry is built from three clean sub-objects:
    _NoseCone  - Von Karman-Haack ogive lofted through BSpline cross-sections
    _Barrel    - straight cylinder lofted between two circles
    _BoatTail  - conical taper to engine exit diameter

Interior reference boxes (not structural):
    payload_bay_box  - red,    CubeSat standard envelope
    avionics_bay_box - yellow, exact dimension box

Part of Team 24 KBE Assignment - Spaceplane conceptual design tool.
"""
from linecache import clearcache
from math import pi, sqrt
import math
import numpy as np
import warnings

from parapy.core import *
from parapy.geom import *
from parapy.core.validate import *


# ---------------------------------------------------------------------------
# Warning helper
# ---------------------------------------------------------------------------

def generate_warning(header: str, msg: str):
    from tkinter import Tk, messagebox
    window = Tk()
    window.withdraw()
    messagebox.showwarning(header, msg)
    window.destroy()


# ---------------------------------------------------------------------------
# CubeSat standard dimensions
# ---------------------------------------------------------------------------

CUBESAT_STANDARDS = {
    "1U":  (0.100, 0.100, 0.113),
    "3U":  (0.100, 0.100, 0.340),
    "6U":  (0.100, 0.226, 0.340),
    "12U": (0.226, 0.226, 0.340),
}



# ---------------------------------------------------------------------------
# Fuselage skin material database
# ---------------------------------------------------------------------------

FUSELAGE_MATERIALS = {
    "Al-6061-T6": dict(
        density=2700.0,       # kg/m³  https://asm.matweb.com
        sigma_allow=276e6,    # Pa     yield strength
        color="silver",
        note="Common aerospace aluminium alloy. Good machinability.",
    ),
    "Al-2024-T3": dict(
        density=2780.0,
        sigma_allow=345e6,
        color="gray",
        note="Higher-strength aluminium. Used in aircraft fuselage skins.",
    ),
    "CFRP": dict(
        density=1600.0,       # kg/m³  typical CFRP laminate
        sigma_allow=600e6,    # Pa     tensile (layup-dependent)
        color="black",
        note="Carbon fibre reinforced polymer. Best mass fraction.",
    ),
    "Ti-6Al-4V": dict(
        density=4430.0,
        sigma_allow=880e6,
        color="darkgray",
        note="Titanium alloy. High temp / high stress regions.",
    ),
}

# ---------------------------------------------------------------------------
# Helper: circular cross-section control points
# ---------------------------------------------------------------------------

def _circle_points(x_pos, radius, n=9):
    """
    n Points forming a closed circle in the YZ-plane at x_pos.
    n=9 gives a good BSpline circle approximation; last == first (closed).
    """
    pts = []
    for i in range(n):
        angle = 2.0 * math.pi * i / (n - 1)
        pts.append(Point(x_pos,
                         radius * math.cos(angle),
                         radius * math.sin(angle)))
    return pts


# ---------------------------------------------------------------------------
# _NoseCone  - Von Karman-Haack ogive
# ---------------------------------------------------------------------------

class _NoseCone(Base):
    """
    Von Karman-Haack ogive nose cone lofted through n_sects BSpline sections.

    Radius distribution (minimum-wave-drag body of revolution):
        theta(x):  x = L_nose * (1 - cos theta) / 2,   theta in [0, pi]
        r(theta) = R_fus * sqrt[(theta - 0.5*sin(2*theta)) / pi]

    The loft runs tip to base so it connects flush with the barrel.
    """

    L_nose:  float = Input()
    R_fus:   float = Input()
    n_sects: int   = Input(8)

    @Attribute
    def _profile_curves(self):
        """
        BSpline cross-sections from tip to base.
        Kept as an @Attribute so @Part surface stays a single return.
        """
        curves = []
        for i in range(self.n_sects):
            theta = math.pi * (i + 0.5) / self.n_sects
            x_pos = self.L_nose * (1.0 - math.cos(theta)) / 2.0
            r_k   = self.R_fus * math.sqrt(
                        (theta - 0.5 * math.sin(2.0 * theta)) / math.pi)
            pts = _circle_points(x_pos, max(r_k, 1e-4))
            curves.append(BSplineCurve(control_points=pts,
                                       label="nose_sect_%d" % i))
        pts_base = _circle_points(self.L_nose, self.R_fus)
        curves.append(BSplineCurve(control_points=pts_base,
                                   label="nose_base"))
        return curves

    @Part
    def surface(self):
        return LoftedSurface(profiles=self._profile_curves,
                             label="nose_cone")


# ---------------------------------------------------------------------------
# _Barrel  - straight cylindrical section
# ---------------------------------------------------------------------------

class _Barrel(Base):
    """Straight cylindrical barrel lofted between two BSpline circles."""

    x_start: float = Input()
    x_end:   float = Input()
    R_fus:   float = Input()

    @Attribute
    def _pts_fwd(self):
        return _circle_points(self.x_start, self.R_fus)

    @Part
    def fwd_curve(self):
        return BSplineCurve(control_points=self._pts_fwd,
                            label="barrel_fwd")

    @Attribute
    def _pts_aft(self):
        return _circle_points(self.x_end, self.R_fus)

    @Part
    def aft_curve(self):
        return BSplineCurve(control_points=self._pts_aft,
                            label="barrel_aft")

    @Part
    def surface(self):
        return LoftedSurface(profiles=[self.fwd_curve, self.aft_curve],
                             label="barrel_surface")


# ---------------------------------------------------------------------------
# _BoatTail  - conical aft closure to engine exit
# ---------------------------------------------------------------------------

class _BoatTail(Base):
    """Conical taper from fuselage radius to engine exit radius."""

    x_start:  float = Input()
    L_tail:   float = Input()
    R_fus:    float = Input()
    R_engine: float = Input()

    @Attribute
    def x_end(self):
        return self.x_start + self.L_tail

    @Attribute
    def _pts_fwd(self):
        return _circle_points(self.x_start, self.R_fus)

    @Part
    def fwd_curve(self):
        return BSplineCurve(control_points=self._pts_fwd,
                            label="tail_fwd")

    @Attribute
    def _pts_aft(self):
        return _circle_points(self.x_end, self.R_engine)

    @Part
    def aft_curve(self):
        return BSplineCurve(control_points=self._pts_aft,
                            label="tail_aft")

    @Part
    def surface(self):
        return LoftedSurface(profiles=[self.fwd_curve, self.aft_curve],
                             label="boat_tail")


# ---------------------------------------------------------------------------
# StandardPayloadBay
# ---------------------------------------------------------------------------

class StandardPayloadBay(Base):
    """
    Primary payload bay sized to a CubeSat standard (1U / 3U / 6U / 12U).
    required_diameter = diagonal of CubeSat cross-section + 2*clearance.
    """

    cubesat_standard: str   = Input("3U", validator=OneOf(list(CUBESAT_STANDARDS.keys())))
    n_units_stacked:  int   = Input(1,    validator=Positive())
    clearance:        float = Input(0.030, validator=Positive(incl_zero=True))

    @Attribute
    def cubesat_dims(self):
        return CUBESAT_STANDARDS[self.cubesat_standard]

    @Attribute
    def cs_length(self):
        return self.cubesat_dims[2]

    @Attribute
    def cs_width(self):
        return self.cubesat_dims[0]

    @Attribute
    def cs_depth(self):
        return self.cubesat_dims[1]

    @Attribute
    def required_longitudinal(self):
        return self.cs_length * self.n_units_stacked + 2.0 * self.clearance

    @Attribute
    def required_lateral(self):
        return self.cs_width + 2.0 * self.clearance

    @Attribute
    def required_vertical(self):
        return self.cs_depth + 2.0 * self.clearance

    @Attribute
    def required_diameter(self):
        return sqrt(self.required_lateral ** 2 + self.required_vertical ** 2+ 2* self.clearance)

    @Attribute
    def required_volume(self):
        return (self.required_longitudinal
                * self.required_lateral
                * self.required_vertical)


# ---------------------------------------------------------------------------
# AvionicsBay
# ---------------------------------------------------------------------------

class AvionicsBay(Base):
    """Avionics bay - exact box dimensions given in informal engineering model, no clearance logic."""

    avionics_box_length: float = Input(0.150, validator=Positive())
    avionics_box_width:  float = Input(0.120, validator=Positive())
    avionics_box_height: float = Input(0.080, validator=Positive())

    @Attribute
    def total_bay_length(self):
        return self.avionics_box_length


# ---------------------------------------------------------------------------
# Fuselage  - root class
# ---------------------------------------------------------------------------

class Fuselage(Base):
    """
    Rule-based parametric fuselage for a suborbital research spaceplane.

    Geometry (three clean surfaces, no boolean operations):
        nose_cone  - Von Karman-Haack lofted surface
        barrel     - straight cylinder lofted between two circles
        boat_tail  - conical taper to engine exit diameter

    Interior reference boxes (transparent, not structural):
        payload_bay_box  - red,    CubeSat primary science payload
        avionics_bay_box - yellow, avionics electronics

    Section layout (nose tip to tail tip):
        [nose cone] [payload bay] [avionics bay] [propulsion bay] [boat-tail]

    Slenderness enforcement:
        If L/D < min_slenderness (default 12), the barrel is automatically
        extended via slenderness_barrel_extension so that L/D >= min_slenderness.
        The extension is added to cylindrical_length and propagates to all
        downstream x-positions and total_length automatically.

    Soft-rule warnings on slenderness and fineness ratios.
    """

    # --- Payload inputs exposed at top level ---
    cubesat_standard: str = Input("6U")
    n_units_stacked: int = Input(1)
    clearance: float = Input(0.030)

    # --- Avionics inputs exposed at top level ---
    avionics_box_length: float = Input(0.150)
    avionics_box_width:  float = Input(0.120)
    avionics_box_height: float = Input(0.080)

    fuselage_material: str = Input("Al-6061-T6",
                                   validator=OneOf(list(FUSELAGE_MATERIALS.keys())))
    skin_thickness: float = Input(0.002, validator=Positive())

    propulsion_bay_length: float = Input(1.20, validator=GreaterThan(0))
    min_inner_diameter:    float = Input(0.150)
    nose_fineness:         float = Input(1.8)
    tail_fineness:         float = Input(2.5)
    engine_exit_diameter:  float = Input(0.30, validator=Positive())
    n_nose_sects:          int   = Input(8)
    popup_warnings:        bool  = Input(False)

    #: Minimum allowable slenderness ratio L/D. Barrel is extended
    #: automatically if the natural geometry falls below this value.
    #: Reference: Vos Slide 12 — transonic spaceplane target 12–20.
    min_slenderness: float = Input(12.0, validator=Between(1.0, 30.0))

    # ── Sub-parts ─────────────────────────────────────────────────────

    @Part
    def payload_bay(self):
        return StandardPayloadBay(
            cubesat_standard=self.cubesat_standard,
            n_units_stacked=self.n_units_stacked,
            clearance=self.clearance,
        )

    @Part
    def avionics(self):
        return AvionicsBay(
            avionics_box_length=self.avionics_box_length,
            avionics_box_width=self.avionics_box_width,
            avionics_box_height=self.avionics_box_height,
        )

    # ── Diameters ─────────────────────────────────────────────────────

    @Attribute
    def inner_diameter(self):
        return max(self.payload_bay.required_diameter, self.min_inner_diameter)

    @Attribute
    def outer_diameter(self):
        return self.inner_diameter + 2.0 * self.skin_thickness

    @Attribute
    def outer_radius(self):
        return 0.5 * self.outer_diameter

    @Attribute
    def inner_radius(self):
        return 0.5 * self.inner_diameter

    # ── Section lengths ───────────────────────────────────────────────

    @Attribute
    def nose_length(self):
        """L_nose = nose_fineness x D_outer"""
        return self.nose_fineness * self.outer_diameter

    @Attribute
    def tail_length(self):
        """L_tail = tail_fineness x D_outer"""
        return self.tail_fineness * self.outer_diameter

    @Attribute
    def _base_cylindrical_length(self):
        """
        Cylindrical barrel length from functional bays only, before any
        slenderness correction is applied.
        """
        return (self.payload_bay.required_longitudinal
                + self.avionics.total_bay_length
                + self.propulsion_bay_length)

    @Attribute
    def slenderness_barrel_extension(self):
        """
        Extra barrel length [m] added to meet min_slenderness (default 12).

        Derivation:
            L_total = nose + (cyl_base + ext) + tail
            L/D >= min_slenderness
            => ext >= min_slenderness * D_outer - nose - cyl_base - tail
            => ext = max(0, min_slenderness * D_outer - nose - cyl_base - tail)

        When the natural geometry already meets the requirement, ext = 0
        and nothing changes. When it does not, the barrel grows just enough
        to hit exactly L/D = min_slenderness.

        This extra length is added inside the propulsion bay — it represents
        additional structural / margin volume aft of the tank stack, which
        is physically reasonable. The propulsion_bay_length Input is NOT
        modified; instead the extension sits transparently inside the barrel.
        """
        natural_total = (self.nose_length
                         + self._base_cylindrical_length
                         + self.tail_length)
        required_total = self.min_slenderness * self.outer_diameter
        ext = max(0.0, required_total - natural_total)
        if ext > 0.0:
            msg = (
                f"Slenderness ratio would be "
                f"{natural_total / self.outer_diameter:.2f} < {self.min_slenderness}. "
                f"Barrel extended by {ext * 1e3:.1f} mm to reach "
                f"L/D = {self.min_slenderness:.1f}."
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Slenderness barrel extension", msg)
        return ext

    @Attribute
    def cylindrical_length(self):
        """
        Final barrel length including any slenderness extension [m].
        = functional bays + extension to meet min_slenderness.
        """
        return self._base_cylindrical_length + self.slenderness_barrel_extension

    @Attribute
    def total_length(self):
        return self.nose_length + self.cylindrical_length + self.tail_length

    # ── Slenderness check (now always satisfied by construction) ──────

    @Attribute
    def slenderness_ratio(self):
        """
        Actual L/D after barrel extension. Always >= min_slenderness.
        A warning is still issued above 20 (bending loads critical).
        """
        sr = self.total_length / self.outer_diameter
        if sr > 20.0:
            msg = f"Slenderness ratio {sr:.2f} > 20 - bending loads critical."
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Slenderness ratio", msg)
        return sr


    #next two functions not needed unless nose and tail fineness are user inputs
    @Attribute
    def checked_nose_fineness(self):
        if self.nose_fineness < 1.5:
            msg = (f"nose_fineness ({self.nose_fineness:.2f}) < 1.5 - "
                   "blunt nose, elevated wave drag at transonic speed.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Nose fineness", msg)
        return self.nose_fineness

    @Attribute
    def checked_tail_fineness(self):
        if self.tail_fineness < 1.0:
            msg = (f"tail_fineness ({self.tail_fineness:.2f}) < 1.0 - "
                   "steep boat-tail, flow separation risk.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Tail fineness", msg)
        return self.tail_fineness

    # ── X-positions (origin = nose tip) ──────────────────────────────

    @Attribute
    def x_nose_base(self):
        return self.nose_length

    @Attribute
    def x_payload_bay_start(self):
        return self.x_nose_base

    @Attribute
    def x_avionics_start(self):
        return self.x_payload_bay_start + self.payload_bay.required_longitudinal

    @Attribute
    def x_propulsion_bay_start(self):
        return self.x_avionics_start + self.avionics.total_bay_length

    @Attribute
    def x_tail_start(self):
        return self.nose_length + self.cylindrical_length

    @Attribute
    def x_tail_tip(self):
        return self.total_length

    #  Material properties

    @Attribute
    def _mat(self):
        """Material property dict for the selected fuselage_material."""
        return FUSELAGE_MATERIALS[self.fuselage_material]

    @Attribute
    def skin_density(self):
        """Shell material density [kg/m³]."""
        return self._mat["density"]

    @Attribute
    def skin_color(self):
        """Display color matching the selected material."""
        return self._mat["color"]

    # ── Structural mass ───────────────────────────────────────────────

    @Attribute
    def nose_cone_wetted_area(self):
        """
        Lateral surface area of the Von Karman ogive [m²].
        Approximated as a cone with same base radius and slant height:
            A = pi * r * sqrt(r^2 + L^2)
        Slight overestimate (ogive is fuller than cone) - conservative.
        """
        r = self.outer_radius
        return math.pi * r * math.sqrt(r ** 2 + self.nose_length ** 2)

    @Attribute
    def barrel_wetted_area(self):
        """Lateral surface area of the cylindrical barrel [m²]."""
        return math.pi * self.outer_diameter * self.cylindrical_length

    @Attribute
    def boat_tail_wetted_area(self):
        """
        Assume cylindrical barrel for simplicity
        """
        return math.pi * self.outer_diameter * self.tail_length

    @Attribute
    def total_wetted_area(self):
        """Total outer surface area of the fuselage shell [m²]."""
        return (self.nose_cone_wetted_area
                + self.barrel_wetted_area
                + self.boat_tail_wetted_area)

    @Attribute
    def nose_cone_mass(self):
        """Nose cone skin mass [kg] = rho * area * thickness."""
        return self.skin_density * self.nose_cone_wetted_area * self.skin_thickness

    @Attribute
    def barrel_mass(self):
        """Barrel skin mass [kg]."""
        return self.skin_density * self.barrel_wetted_area * self.skin_thickness

    @Attribute
    def boat_tail_mass(self):
        """Boat-tail skin mass [kg]."""
        return self.skin_density * self.boat_tail_wetted_area * self.skin_thickness

    @Attribute
    def fuselage_structural_mass(self):
        """
        Total fuselage shell structural mass [kg].
        = nose_cone_mass + barrel_mass + boat_tail_mass
        Does not include payload, avionics, propulsion, or wings.
        """
        return self.nose_cone_mass + self.barrel_mass + self.boat_tail_mass

    # ── Geometry Parts ────────────────────────────────────────────────

    @Part
    def nose_cone(self):
        return _NoseCone(
            L_nose=self.nose_length,
            R_fus=self.outer_radius,
            n_sects=self.n_nose_sects,
            label="nose_cone",
        )

    @Part
    def barrel(self):
        return _Barrel(
            x_start=self.x_nose_base,
            x_end=self.x_tail_start,
            R_fus=self.outer_radius,
            label="barrel",
        )

    @Part
    def boat_tail(self):
        return _BoatTail(
            x_start=self.x_tail_start,
            L_tail=self.tail_length,
            R_fus=self.outer_radius,
            R_engine=self.outer_radius,
            label="boat_tail",
        )

    # ── Interior reference boxes ──────────────────────────────────────

    @Part
    def payload_bay_box(self):
        """CubeSat primary payload envelope - red, semi-transparent."""
        return Box(
            length=self.payload_bay.required_lateral,
            width=self.payload_bay.required_longitudinal,
            height=self.payload_bay.required_vertical,
            centered=True,
            position=Position(Point(
                self.x_payload_bay_start
                + 0.5 * self.payload_bay.required_longitudinal,
                0, 0,
            )),
            color="red",
            transparency=0.5,
        )

    @Part
    def avionics_bay_box(self):
        """Avionics box envelope - yellow, semi-transparent."""
        return Box(
            length=self.avionics.avionics_box_width,
            width=self.avionics.avionics_box_length,
            height=self.avionics.avionics_box_height,
            centered=True,
            position=Position(Point(
                self.x_avionics_start
                + 0.5 * self.avionics.avionics_box_length,
                0, 0,
            )),
            color="yellow",
            transparency=0.4,
        )

    # ── Summary ───────────────────────────────────────────────────────

    @Attribute
    def summary(self):
        return {
            "inner_diameter_m":              round(self.inner_diameter, 3),
            "outer_diameter_m":              round(self.outer_diameter, 3),
            "nose_length_m":                 round(self.nose_length, 3),
            "cylindrical_length_m":          round(self.cylindrical_length, 3),
            "slenderness_barrel_ext_mm":     round(self.slenderness_barrel_extension * 1e3, 1),
            "tail_length_m":                 round(self.tail_length, 3),
            "total_length_m":                round(self.total_length, 3),
            "slenderness_ratio":             round(self.slenderness_ratio, 2),
            "min_slenderness":               self.min_slenderness,
            "cubesat_standard":              self.payload_bay.cubesat_standard,
            "payload_bay_volume_m3":         round(self.payload_bay.required_volume, 4),
            "x_payload_bay_start_m":         round(self.x_payload_bay_start, 3),
            "x_avionics_start_m":            round(self.x_avionics_start, 3),
            "x_propulsion_bay_start_m":      round(self.x_propulsion_bay_start, 3),
            "x_tail_start_m":                round(self.x_tail_start, 3),
            # ── Structural mass breakdown ─────────────────────────
            "fuselage_material":             self.fuselage_material,
            "skin_thickness_mm":             round(self.skin_thickness * 1e3, 2),
            "skin_density_kg_m3":            round(self.skin_density, 1),
            "nose_cone_wetted_area_m2":      round(self.nose_cone_wetted_area, 4),
            "barrel_wetted_area_m2":         round(self.barrel_wetted_area, 4),
            "boat_tail_wetted_area_m2":      round(self.boat_tail_wetted_area, 4),
            "total_wetted_area_m2":          round(self.total_wetted_area, 4),
            "nose_cone_mass_kg":             round(self.nose_cone_mass, 3),
            "barrel_mass_kg":                round(self.barrel_mass, 3),
            "boat_tail_mass_kg":             round(self.boat_tail_mass, 3),
            "fuselage_structural_mass_kg":   round(self.fuselage_structural_mass, 3),
        }

    def print_summary(self):
        print("\n" + "=" * 55)
        print("  FUSELAGE SUMMARY")
        print("=" * 55)
        for k, v in self.summary.items():
            print(f"  {k:<40} {v}")
        print("=" * 55)


# ---------------------------------------------------------------------------
# Stand-alone entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from parapy.gui import display

    fu = Fuselage(
        label="Spaceplane Fuselage (6U CubeSat)",
        cubesat_standard="1U",
        n_units_stacked=1,
        clearance=0.030,
        avionics_box_length=0.150,
        avionics_box_width=0.120,
        avionics_box_height=0.080,
        propulsion_bay_length=1.20,
        min_inner_diameter=0.100,
        nose_fineness=1.8,
        tail_fineness=1.3,
        engine_exit_diameter=0.443,
        n_nose_sects=8,
        min_slenderness=12.0,
        fuselage_material="Al-6061-T6",
        skin_thickness=0.002,
        popup_warnings=False,
    )

    fu.print_summary()
    display(fu)