"""
fuselage.py

Parametric fuselage model for the suborbital research spaceplane KBE application.
"""

from math import pi, sqrt
import numpy as np
import warnings

from parapy.core import *
from parapy.geom import *
from parapy.core.validate import *


def generate_warning(header: str, msg: str):
    from tkinter import Tk, messagebox
    window = Tk()
    window.withdraw()
    messagebox.showwarning(header, msg)
    window.destroy()


CUBESAT_STANDARDS = {
    "1U":  (0.100, 0.100, 0.113),
    "3U":  (0.100, 0.100, 0.340),
    "6U":  (0.100, 0.226, 0.340),
    "12U": (0.226, 0.226, 0.340),
}


class NoseConePayloadBay(Base):
    """
    Optional payload volume inside the nose cone, modelled as a cone
    matching the nose taper.  LEFT EMPTY in the baseline design.
    """

    nose_base_diameter: float = Input(0.30, validator=Positive())
    nose_length:        float = Input(0.54, validator=Positive())
    usable_fraction:    float = Input(0.60, validator=Between(0.1, 0.85))
    wall_margin:        float = Input(0.025, validator=Positive())
    x_nose_tip:         float = Input(0.0)

    @Attribute
    def usable_radius(self) -> float:
        return 0.5 * self.nose_base_diameter - self.wall_margin

    @Attribute
    def usable_length(self) -> float:
        return self.usable_fraction * self.nose_length

    @Attribute
    def usable_volume(self) -> float:
        """V_cone = (1/3) * pi * r^2 * h"""
        return (1.0 / 3.0) * pi * self.usable_radius ** 2 * self.usable_length

    @Part
    def envelope_box(self):
        """Conical envelope – tip at nose tip, base radius = usable_radius."""
        return Cone(
            radius1=0.005,
            radius2=self.usable_radius,
            height=self.nose_length,
            position=rotate(
                translate(XOY, "x", self.x_nose_tip),
                "y", 90, deg=True
            ),
            color="yellow",
            transparency=0.55,
        )




class StandardPayloadBay(Base):
    """
    Primary payload bay sized to a CubeSat standard (1U / 3U / 6U / 12U).
    """

    cubesat_standard: str = Input(
        "3U", validator=OneOf(list(CUBESAT_STANDARDS.keys()))
    )
    n_units_stacked: int   = Input(1,     validator=Positive())
    clearance:       float = Input(0.030, validator=Positive(incl_zero=True))

    @Attribute
    def cubesat_dims(self):
        return CUBESAT_STANDARDS[self.cubesat_standard]

    @Attribute
    def cs_length(self) -> float:
        return self.cubesat_dims[2]

    @Attribute
    def cs_width(self) -> float:
        return self.cubesat_dims[0]

    @Attribute
    def cs_depth(self) -> float:
        return self.cubesat_dims[1]

    @Attribute
    def required_longitudinal(self) -> float:
        return self.cs_length * self.n_units_stacked + 2.0 * self.clearance

    @Attribute
    def required_lateral(self) -> float:
        return self.cs_width + 2.0 * self.clearance

    @Attribute
    def required_vertical(self) -> float:
        return self.cs_depth + 2.0 * self.clearance

    @Attribute
    def required_diameter(self) -> float:
        return sqrt(self.required_lateral ** 2 + self.required_vertical ** 2)

    @Attribute
    def required_volume(self) -> float:
        return (self.required_longitudinal
                * self.required_lateral
                * self.required_vertical)


class AvionicsBay(Base):
    """
    Avionics bay – a simple box with the exact user-supplied dimensions.
    No internal clearance calculations; the box IS the envelope.
    """

    #: Box length along fuselage X axis [m]
    avionics_box_length: float = Input(0.150, validator=Positive())
    #: Box width along Y axis [m]
    avionics_box_width:  float = Input(0.120, validator=Positive())
    #: Box height along Z axis [m]
    avionics_box_height: float = Input(0.080, validator=Positive())

    @Attribute
    def total_bay_length(self) -> float:
        """Axial space the avionics bay occupies in the fuselage [m]."""
        return self.avionics_box_length


class Fuselage(Base):
    """
    Rule-based parametric fuselage for a suborbital research spaceplane.

    Section layout (nose → tail):
        1. Nose cone            – empty / optional nose-cone payload (cone shape)
        2. Standard payload bay – CubeSat primary science payload (red box)
        3. Avionics bay         – flight computer + electronics (yellow box)
        4. Propulsion bay       – tanks and engine
    """

    payload_bay:           StandardPayloadBay = Input(StandardPayloadBay())
    avionics:              AvionicsBay        = Input(AvionicsBay())
    propulsion_bay_length: float              = Input(1.20, validator=GreaterThan(0))
    structural_wall_depth: float              = Input(0.05, validator=Between(0.02, 0.15))
    min_inner_diameter:    float              = Input(0.30)
    nose_fineness:         float              = Input(1.8)
    tail_fineness:         float              = Input(3.0)
    popup_warnings:        bool               = Input(False)
    n_profile_points:      int                = Input(40)

    # ── Diameters ─────────────────────────────────────────────────────

    @Attribute
    def inner_diameter(self) -> float:
        return max(self.payload_bay.required_diameter, self.min_inner_diameter)

    @Attribute
    def outer_diameter(self) -> float:
        return self.inner_diameter + 2.0 * self.structural_wall_depth

    @Attribute
    def inner_radius(self) -> float:
        return 0.5 * self.inner_diameter

    @Attribute
    def outer_radius(self) -> float:
        return 0.5 * self.outer_diameter

    # ── Section lengths ───────────────────────────────────────────────

    @Attribute
    def cylindrical_length(self) -> float:
        return (self.payload_bay.required_longitudinal
                + self.avionics.total_bay_length
                + self.propulsion_bay_length)

    @Attribute
    def nose_length(self) -> float:
        return self.nose_fineness * self.outer_diameter

    @Attribute
    def tail_length(self) -> float:
        return self.tail_fineness * self.outer_diameter

    @Attribute
    def total_length(self) -> float:
        return self.nose_length + self.cylindrical_length + self.tail_length

    # ── Soft-rule checks ──────────────────────────────────────────────

    @Attribute
    def slenderness_ratio(self) -> float:
        sr = self.total_length / self.outer_diameter
        if sr < 20.0:
            warnings.warn(f"Fineness ratio {sr:.2f} < 20 — pressure drag elevated.")
            if self.popup_warnings:
                generate_warning("Slenderness ratio warning",
                                 f"Fineness ratio {sr:.2f} < 20.")
        elif sr > 25.0:
            warnings.warn(f"Fineness ratio {sr:.2f} > 25 — bending loads critical.")
        return sr

    # ── X-positions (origin = nose tip) ──────────────────────────────

    @Attribute
    def x_nose_base(self) -> float:
        return self.nose_length

    @Attribute
    def x_tail_start(self) -> float:
        return self.nose_length + self.cylindrical_length

    @Attribute
    def x_tail_mid(self) -> float:
        return self.x_tail_start + 0.5 * self.tail_length

    @Attribute
    def x_tail_tip(self) -> float:
        return self.total_length

    @Attribute
    def x_payload_bay_start(self) -> float:
        return self.x_nose_base

    @Attribute
    def x_avionics_start(self) -> float:
        return self.x_payload_bay_start + self.payload_bay.required_longitudinal

    @Attribute
    def x_propulsion_bay_start(self) -> float:
        return self.x_avionics_start + self.avionics.total_bay_length

    # ── Profile helper ────────────────────────────────────────────────

    def _section_coordinates(self, radius: float, x: float):
        angles = np.linspace(0, 2 * np.pi, self.n_profile_points, endpoint=True)
        return [(x, radius * np.cos(t), radius * np.sin(t)) for t in angles]

    # ── Outer loft ────────────────────────────────────────────────────

    @Part
    def outer_nose_point(self):
        return FittedCurve(points=self._section_coordinates(0.01, 0.0))

    @Part
    def outer_nose_base(self):
        return FittedCurve(
            points=self._section_coordinates(self.outer_radius, self.x_nose_base))

    @Part
    def outer_tail_start(self):
        return FittedCurve(
            points=self._section_coordinates(self.outer_radius, self.x_tail_start))

    @Part
    def outer_tail_mid(self):
        return FittedCurve(
            points=self._section_coordinates(self.outer_radius, self.x_tail_mid))

    @Part
    def outer_tail_tip(self):
        return FittedCurve(
            points=self._section_coordinates(self.outer_radius, self.x_tail_tip))

    @Attribute
    def outer_profiles(self):
        return [self.outer_nose_point, self.outer_nose_base,
                self.outer_tail_start, self.outer_tail_mid, self.outer_tail_tip]

    @Part
    def outer_loft(self):
        return LoftedSolid(profiles=self.outer_profiles)

    # ── Inner loft ────────────────────────────────────────────────────

    @Part
    def inner_nose_start(self):
        return FittedCurve(
            points=self._section_coordinates(0.01, 0.05 * self.nose_length))

    @Part
    def inner_nose_base(self):
        return FittedCurve(
            points=self._section_coordinates(self.inner_radius, self.x_nose_base))

    @Part
    def inner_tail_start(self):
        return FittedCurve(
            points=self._section_coordinates(self.inner_radius, self.x_tail_start))

    @Part
    def inner_tail_mid(self):
        return FittedCurve(
            points=self._section_coordinates(self.inner_radius, self.x_tail_mid))

    @Part
    def inner_tail_end(self):
        return FittedCurve(
            points=self._section_coordinates(self.inner_radius, self.x_tail_tip))

    @Attribute
    def inner_profiles(self):
        return [self.inner_nose_start, self.inner_nose_base,
                self.inner_tail_start, self.inner_tail_mid, self.inner_tail_end]

    @Part
    def inner_loft(self):
        return LoftedSolid(profiles=self.inner_profiles)

    @Part
    def fuselage_shell(self):
        return SubtractedSolid(
            shape_in=self.outer_loft,
            tool=self.inner_loft,
            color="gray",
            transparency=0.2,
        )

    # ── Nose-cone payload bay (empty, conical) ────────────────────────

    @Part
    def nose_cone_payload_bay(self):
        return NoseConePayloadBay(
            nose_base_diameter=self.outer_diameter,
            nose_length=self.nose_length,
            usable_fraction=0.60,
            wall_margin=self.structural_wall_depth + 0.010,
            x_nose_tip=0.0,
        )

    # ── Primary CubeSat payload bay (red box) ─────────────────────────

    @Part
    def payload_bay_box(self):
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

    # ── Avionics bay (yellow box, exact input dimensions) ─────────────

    @Part
    def avionics_bay_box(self):
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
    def summary(self) -> dict:
        return {
            "inner_diameter_m":         round(self.inner_diameter, 3),
            "outer_diameter_m":         round(self.outer_diameter, 3),
            "nose_length_m":            round(self.nose_length, 3),
            "cylindrical_length_m":     round(self.cylindrical_length, 3),
            "tail_length_m":            round(self.tail_length, 3),
            "total_length_m":           round(self.total_length, 3),
            "x_payload_bay_start_m":    round(self.x_payload_bay_start, 3),
            "x_avionics_start_m":       round(self.x_avionics_start, 3),
            "x_propulsion_bay_start_m": round(self.x_propulsion_bay_start, 3),
            "cubesat_standard":         self.payload_bay.cubesat_standard,
            "payload_bay_volume_m3":    round(self.payload_bay.required_volume, 4),
            "avionics_bay_length_m":    round(self.avionics.total_bay_length, 3),
            "slenderness_ratio":        round(self.slenderness_ratio, 2),
        }


if __name__ == "__main__":
    from parapy.gui import display

    fu = Fuselage(
        label="Spaceplane Fuselage (6U CubeSat)",
        payload_bay=StandardPayloadBay(cubesat_standard="6U", n_units_stacked=1,
                                       clearance=0.030),
        avionics=AvionicsBay(avionics_box_length=0.150,
                             avionics_box_width=0.120,
                             avionics_box_height=0.080),
        propulsion_bay_length=1.20,
        structural_wall_depth=0.05,
        min_inner_diameter=0.30,
        nose_fineness=1.8,
        tail_fineness=2.5,
    )

    print("\n=== Fuselage Summary ===")
    for k, v in fu.summary.items():
        print(f"  {k:<40} {v}")

    display(fu)

