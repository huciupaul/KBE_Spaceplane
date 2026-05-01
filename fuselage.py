"""
fuselage.py

Parametric fuselage model for the suborbital research spaceplane KBE application.

Engineering knowledge sourced from:
    Vos, R., Hoogreef, M.F.M., Zandbergen, B.T.C.
    "Aerospace Design and Systems Engineering Elements I – The Design of the Fuselage"
    TU Delft lecture slides (2025)

The fuselage is split into three sections:
    1. Nose cone   – from tip to max diameter
    2. Mid section – constant-diameter cylinder housing payload bay + avionics
    3. Tail cone   – boattail from max diameter to nozzle exit diameter

Part of Team 24 KBE Assignment – Spaceplane conceptual design tool.
"""

from math import pi,sqrt
import numpy as np
import warnings

from parapy.core import *
from parapy.geom import *
from parapy.core.validate import *

def generate_warning(header: str, msg: str):
    """Show a modal warning dialog and wait for the user to dismiss it."""
    from tkinter import Tk, messagebox
    window = Tk()
    window.withdraw()
    messagebox.showwarning(header, msg)
    window.deiconify()
    window.destroy()
    window.quit()

class PayloadBay(Base):
    """

    Designed as a simple payload-bay envelope. The fuselage cross section is
    designed to envelope the payload with a min clearance margin.

    The required circular envelope diameter sets te inner fuselage diameter.
    """
    payload_longitudinal = Input(1.20)
    payload_lateral = Input(0.80)
    payload_vertical = Input(0.80)
    clearance = Input(0.10)

    @Attribute
    def required_longitudinal(self):
        return self.payload_longitudinal + 2 * self.clearance

    @Attribute
    def required_lateral(self):
        return self.payload_lateral + 2 * self.clearance

    @Attribute
    def required_vertical(self):
        return self.payload_vertical + 2 * self.clearance
    @Attribute
    def required_diameter(self):
        return 1.05* sqrt(self.required_lateral**2 + self.required_vertical**2)

    @Attribute
    def required_volume(self):
        """Payload bay box volume (with clearance) [m3]"""
        return self.required_longitudinal * self.required_vertical * self.required_lateral

class Fuselage(Base):
    """
    Parametric fuselage for a suborbital research spaceplane.

    Designed inside-out (payload first, then structure around it), following
    the standard conceptual design procedure [Slide 6, Slide 61].

    The three-section layout (nose / mid / tail) follows the standard
    top-view decomposition described in [Slide 51, Slide 86].

    Design is driven by:
        - Payload requirements       [Slide 6, Slide 23]
        - Aerodynamic considerations [Slide 10-12, Slide 56-58]
        - Structural considerations  [Slide 13-19, Slide 44-45]
        - Ground handling            [Slide 20-21, Slide 72]
    """

    payload_bay: PayloadBay = Input(PayloadBay())

    avionics_bay_length: float = Input(0.40)
    propulsion_bay_length: float = Input(1.20)

    structural_wall_depth: float = Input(0.05) #fighter jets/trainers
    min_inner_diameter: float = Input(0.50) #to avoid unrealistic narrow bodies

    #: Nose fineness ratio: nose_length / outer_diameter
    #: Target >= 1.5 for acceptable transonic/supersonic wave drag
    nose_fineness: float = Input(1.8)

    #: Tail fineness ratio: tail_length / outer_diameter [Slide 56]
    #: Larger value -> shallower boattail -> less base drag
    tail_fineness: float = Input(2.5)

    upsweep_angle: float = Input(10.0, validator=Between(0, 25))

    @Attribute
    def inner_diameter(self) -> float:
        """
        Inner (usable) fuselage diameter [m].
        """
        return max(self.payload_bay.required_diameter, self.min_inner_diameter)

    @Attribute
    def outer_diameter(self) -> float:
        """
        Outer fuselage diameter [m] = inner + 2 x wall layers.
        A 10% larger diameter yields 1.5-3% total drag increase. [Slide 7]
        """
        return self.inner_diameter + 2 * self.structural_wall_depth

    @Attribute
    def inner_radius(self) -> float:
        return 0.5 * self.inner_diameter

    @Attribute
    def outer_radius(self) -> float:
        return 0.5 * self.outer_diameter

    # Section Lengths
    @Attribute
    def cylindrical_length(self) -> float:
        """
        Total length of the constant-diameter mid-section [m].
        Avionics bay + payload bay + propulsion bay + margin. [Slide 51]
        """
        return (self.avionics_bay_length
                + self.payload_bay.required_longitudinal
                + self.propulsion_bay_length)

    @Attribute
    def nose_length(self) -> float:
        """
        Nose cone length [m] = nose_fineness x outer_diameter. [Slide 57]
        """
        return self.nose_fineness * self.outer_diameter

    @Attribute
    def tail_length(self) -> float:
        """
        Tail cone length [m] = tail_fineness x outer_diameter. [Slide 56]
        """
        return self.tail_fineness * self.outer_diameter

    @Attribute
    def total_length(self) -> float:
        """Total fuselage length [m]. [Slide 51]"""
        return self.nose_length + self.cylindrical_length + self.tail_length

    #Cross-Parameter checks
    @Attribute
    def checked_nose_fineness(self) -> float:
        """
        Soft check on nose fineness. [Slide 57]
        Target: nose_length / outer_diameter >= 1.5 for acceptable wave drag
        at transonic speed.
        Returns the input value unchanged; only warns when below the target.
        """
        if self.nose_fineness < 1.5:
            msg = (
                f"nose_fineness ({self.nose_fineness:.2f}) < 1.5 — blunt nose will "
                "have elevated wave drag at transonic speed. "
                "Recommend nose_fineness >= 1.5. [Slide 57]"
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Nose fineness warning", msg)
        return self.nose_fineness

    @Attribute
    def checked_tail_fineness(self) -> float:
        """
        Soft check on tail fineness. [Slide 56]
        Tail slenderness < 1.0 risks flow separation and high base drag.
        Returns the input value unchanged; only warns when below the target.
        """
        if self.tail_fineness < 1.0:
            msg = (
                f"tail_fineness ({self.tail_fineness:.2f}) < 1.0 — steep boattail "
                "will cause flow separation and high base drag. "
                "Recommend tail_fineness >= 1.0. [Slide 56]"
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Tail fineness warning", msg)
        return self.tail_fineness

    @Attribute
    def checked_upsweep(self) -> float:
        """
        Soft check on tail upsweep angle. [Slide 72]
        First-loop target: <= 14 deg. Values above 15 deg increase drag
        noticeably. Returns the input value; warns when above the target.
        """
        if self.upsweep_angle > 15.0:
            msg = (
                f"upsweep_angle ({self.upsweep_angle:.1f} deg) > 15 deg — "
                "drag increases significantly. First-loop target is 14 deg. [Slide 72]"
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Upsweep angle warning", msg)
        return self.upsweep_angle

    @Attribute
    def slenderness_ratio(self) -> float:
        """
        Fineness ratio: total_length / outer_diameter.
        Soft warning when outside the transonic spaceplane target of 10–20.
        [Slide 12]
        """
        sr = self.total_length / self.outer_diameter
        if sr < 10.0:
            msg = (
                f"Fineness ratio {sr:.2f} < 10 — below the transonic spaceplane "
                "target (10–20). Pressure drag will be elevated. [Slide 12]"
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Slenderness ratio warning", msg)
        elif sr > 20.0:
            msg = (
                f"Fineness ratio {sr:.2f} > 20 — structural bending loads will "
                "be critical. [Slide 12]"
            )
            warnings.warn(msg)
        return sr

    @Attribute
    def payload_clearance_check(self) -> float:
        """
        Warns when the payload envelope occupies more than 90 % of the inner
        diameter — leaving very little margin for wiring and structural frames.
        Returns the inner diameter (unchanged). [Slide 30]
        """
        ratio = self.payload_bay.required_diameter / self.inner_diameter
        if ratio > 0.90:
            msg = (
                f"Payload envelope ({self.payload_bay.required_diameter:.3f} m) "
                f"occupies {ratio:.0%} of inner diameter ({self.inner_diameter:.3f} m). "
                "Very little margin for wiring and frames. "
                "Increase min_inner_diameter or reduce payload envelope. [Slide 30]"
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Payload clearance warning", msg)
        return self.inner_diameter

    #: Show a Tk pop-up for critical soft-rule violations?
    popup_warnings: bool = Input(False)

    # Volume Attributes
    @Attribute
    def internal_cylindrical_volume(self) -> float:
        """Total inner volume of the cylindrical section [m3]."""
        return pi * self.inner_radius ** 2 * self.cylindrical_length

    #X-position (origin = nose tip)
    @Attribute
    def x_nose_base(self) -> float:
        return self.nose_length

    @Attribute
    def x_tail_start(self) -> float:
        return self.nose_length + self.cylindrical_length

    @Attribute
    def x_tail_tip(self) -> float:
        return self.total_length

    @Attribute
    def x_avionics_start(self) -> float:
        """Avionics bay starts immediately after the nose cone."""
        return self.x_nose_base

    @Attribute
    def x_payload_bay_start(self) -> float:
        """Payload bay starts after the avionics bay."""
        return self.x_nose_base + self.avionics_bay_length

    @Attribute
    def x_propulsion_bay_start(self) -> float:
        """Propulsion bay starts after the payload bay."""
        return self.x_payload_bay_start + self.payload_bay.required_longitudinal

    # Reference Points for Geometry
    # Reference positions
    @Attribute
    def nose_base_position(self):
        return translate(XOY, "x", self.x_nose_base)

    @Attribute
    def tail_start_position(self):
        return translate(XOY, "x", self.x_tail_start)

    @Attribute
    def tail_tip_position(self):
        return translate(XOY, "x", self.x_tail_tip)

    @Attribute
    def inner_nose_start_position(self):
        return translate(XOY, "x", 0.25 * self.nose_length)

    @Attribute
    def inner_tail_end_position(self):
        return translate(XOY, "x", self.x_tail_start + 0.65 * self.tail_length)

    n_profile_points: int = Input(40)

    def _section_coordinates(self, radius: float, x: float):
        """
        Generates a circular fuselage section in the YZ-plane at a given x-location.
        The point order is always the same, so the loft seams stay aligned.
        """
        angles = np.linspace(0, 2 * np.pi, self.n_profile_points, endpoint=True)
        return [(x,
                 radius * np.cos(theta),
                 radius * np.sin(theta))
                for theta in angles]

    # Outer geometry
    @Part
    def outer_nose_point(self):
        return FittedCurve(
            points =self._section_coordinates(0.01, 0.0)
        )
    @Part
    def outer_nose_base(self):
        return FittedCurve(
            points =self._section_coordinates(self.outer_radius, self.x_nose_base)
        )
    @Part
    def outer_tail_start(self):
        return FittedCurve(
            points =self._section_coordinates(self.outer_radius, self.x_tail_start)
        )

    @Part
    def outer_tail_tip(self):
        return FittedCurve(
            points =self._section_coordinates(0.01, self.x_tail_tip)
        )
    @Attribute
    def outer_profiles(self):
        return [self.outer_nose_point,
                self.outer_nose_base,
                self.outer_tail_start,
                self.outer_tail_tip]

    @Part
    def outer_loft(self):
        return LoftedSolid(profiles=self.outer_profiles)

    # Inner geometry
    @Part
    def inner_nose_start(self):
        return FittedCurve(
            points =self._section_coordinates(0.01, 0.25 * self.nose_length)
        )

    @Part
    def inner_nose_base(self):
        return FittedCurve(
            points =self._section_coordinates(self.inner_radius, self.x_nose_base)
        )
    @Part
    def inner_tail_start(self):
        return FittedCurve(
            points =self._section_coordinates(self.inner_radius, self.x_tail_start)
        )
    @Part
    def inner_tail_end(self):
        return FittedCurve(
            points =self._section_coordinates(0.01, self.x_tail_start + 0.65 * self.tail_length)
        )
    @Attribute
    def inner_profiles(self):
        return [self.inner_nose_start,
                self.inner_nose_base,
                self.inner_tail_start,
                self.inner_tail_end]

    @Part
    def inner_loft(self):
        return LoftedSolid(profiles=self.inner_profiles)

    @Part
    def fuselage_shell(self):
        return SubtractedSolid(shape_in=self.outer_loft,
                               tool=self.inner_loft,
                               color = "gray",
                               transparency = 0.2)

    # Payload bay reference box
    @Part
    def payload_bay_box(self):
        """
        Reference box for the payload bay usable envelope.
        Used for visual interference checking with PropulsionSystem
        and Wing. [Slide 30]
        """
        return Box(
            length=self.payload_bay.required_lateral,
            width=self.payload_bay.required_longitudinal,
            height=self.payload_bay.required_vertical,
            centered=True,
            position=Position(
                Point(
                    self.x_payload_bay_start + 0.5 * self.payload_bay.required_longitudinal,
                    0,
                    0
                )
            ),
            color="red",
            transparency=0.5
        )


if __name__ == "__main__":
    from parapy.gui import display


    bay=PayloadBay(
            payload_longitudinal=1.20,
            payload_lateral=0.2,
            payload_vertical=0.2,
            clearance=0.10
    )

    fu = Fuselage(
        label="Spaceplane Fuselage",
        payload_bay=bay,
        avionics_bay_length=0.40,
        propulsion_bay_length=1.20,
        structural_wall_depth=0.05,
        min_inner_diameter=0.80,
        nose_fineness=1.8,
        tail_fineness=2.5,
        upsweep_angle=10.0,
        popup_warnings=False,
    )

    display(fu)