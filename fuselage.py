"""
fuselage.py

Parametric fuselage model for the suborbital research spaceplane KBE application.

Engineering knowledge sourced from:
    Vos, R., Hoogreef, M.F.M., Zandbergen, B.T.C.
    "Aerospace Design and Systems Engineering Elements I – The Design of the Fuselage"
    TU Delft lecture slides (2025)

The fuselage is split into three sections:
    1. Nose cone   – from tip to max diameter (left EMPTY for optional payload)
    2. Mid section – constant-diameter cylinder housing:
                        a. NoseConePayloadBay  – inside the nose cone volume
                        b. StandardPayloadBay  – CubeSat standard (1U/3U/6U/12U)
                        c. AvionicsBay         – separated from payload by clearance
                        d. PropulsionBay       – sized by PropulsionSystem
    3. Tail cone   – boattail from max diameter to nozzle exit diameter

CubeSat Unit (1U) standard:
    10 cm × 10 cm × 10 cm   (PC/104 heritage, ECSS-E-ST-10-04C)
    1U  = 100 × 100 × 100 mm
    3U  = 100 × 100 × 340 mm
    6U  = 100 × 200 × 340 mm   (two-wide 3U)
    12U = 200 × 200 × 340 mm   (two-wide 6U stack)

Part of Team 24 KBE Assignment – Spaceplane conceptual design tool.
"""

from math import pi, sqrt
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
    window.destroy()


# ---------------------------------------------------------------------------
# CubeSat standard dimensions  (ECSS-E-ST-10-04C / CDS Rev 14)
# ---------------------------------------------------------------------------

CUBESAT_STANDARDS = {
    #  name  : (length_m, width_m, height_m)   – height = stacking axis (Z)
    "1U":  (0.100, 0.100, 0.113),   # 113 mm includes P-POD rail tolerance
    "3U":  (0.100, 0.100, 0.340),
    "6U":  (0.100, 0.226, 0.340),   # double-wide (PC/104 2×3U)
    "12U": (0.226, 0.226, 0.340),   # quad-wide (PC/104 4×3U)
}


# ---------------------------------------------------------------------------
# NoseConePayloadBay – optional volume inside the nose cone
# ---------------------------------------------------------------------------

class NoseConePayloadBay(Base):
    """
    Represents the usable cylindrical volume inside the nose cone that is
    LEFT EMPTY for optional/additional payload (e.g. science instruments,
    cameras, experiment modules).

    The bay is modelled as a cone coaxial with the fuselage, mirroring the
    tapered shape of the nose cone.  The cone tip is at the nose tip (r=0)
    and the base radius equals (nose_base_diameter/2 - wall_margin) so the
    envelope never penetrates the structural skin.  Its axial length is a
    user-defined fraction of the nose cone length.

    This object is purely a reference/envelope shape – it owns no propulsion
    or avionics and is entirely managed by Fuselage as a child Part.
    """

    #: Outer diameter of the nose cone at its base [m]
    nose_base_diameter: float = Input(0.30, validator=Positive())

    #: Total nose cone length [m]
    nose_length: float = Input(0.54, validator=Positive())

    #: Fraction of nose length available for payload (0–0.85)
    usable_fraction: float = Input(0.60, validator=Between(0.1, 0.85))

    #: Radial clearance between bay envelope and structural skin [m]
    wall_margin: float = Input(0.025, validator=Positive())

    #: X-position of the nose tip (origin of nose cone) [m]
    x_nose_tip: float = Input(0.0)

    @Attribute
    def usable_diameter(self) -> float:
        """Inner diameter of the nose cone payload envelope [m]."""
        return self.nose_base_diameter - 2.0 * self.wall_margin

    @Attribute
    def usable_radius(self) -> float:
        return 0.5 * self.usable_diameter

    @Attribute
    def usable_length(self) -> float:
        """Axial length of the usable nose-cone payload volume [m]."""
        return self.usable_fraction * self.nose_length

    @Attribute
    def usable_volume(self) -> float:
        """Conical usable nose-cone payload volume [m³].
        V_cone = (1/3) * pi * r_base^2 * h
        where r_base = usable_radius (at nose base) and h = usable_length.
        """
        return (1.0 / 3.0) * pi * self.usable_radius ** 2 * self.usable_length

    @Attribute
    def x_bay_start(self) -> float:
        """X-position where the conical envelope base sits [m].
        The cone tip is always at x_nose_tip; the base is at x_bay_start.
        x_bay_start = x_nose_tip + usable_length (= usable_fraction * nose_length).
        """
        return self.x_nose_tip + self.usable_length

    @Attribute
    def x_bay_center(self) -> float:
        return self.x_bay_start + 0.5 * self.usable_length

    @Part
    def envelope_box(self):
        """
        Conical reference envelope of the nose-cone payload bay.
        Matches the taper of the nose cone: tip at x_nose_tip (r=0),
        base at x_nose_tip + usable_length (r = usable_radius).
        Rendered as a semi-transparent yellow cone.
        The volume INSIDE this cone is reserved for optional payload.
        """
        return Cone(
            radius1=0.005,          # near-zero tip (exact zero fails OCCT)
            radius2=self.usable_radius,
            height=self.usable_length,
            position=rotate(
                translate(XOY, "x", self.x_nose_tip),
                "y", 90, deg=True
            ),
            color="yellow",
            transparency=0.55,
        )


# ---------------------------------------------------------------------------
# StandardPayloadBay – CubeSat-standard primary payload bay
# ---------------------------------------------------------------------------

class StandardPayloadBay(Base):
    """
    Primary payload bay sized to hold one CubeSat form-factor payload
    (1U, 3U, 6U, or 12U) plus a structural clearance margin on all sides.

    The required circular cross-section envelope diameter is derived from
    the CubeSat diagonal + 2×clearance, ensuring the CubeSat fits
    regardless of rotation about the fuselage axis.

    Layout convention (body axis = X):
        - CubeSat stacking axis aligned with fuselage X (nose-to-tail)
        - CubeSat height dimension = longitudinal extent in fuselage
    """

    #: CubeSat standard to accommodate
    cubesat_standard: str = Input(
        "3U",
        validator=OneOf(list(CUBESAT_STANDARDS.keys()))
    )

    #: Number of CubeSat units stacked along the fuselage axis
    #: (multiplies the longitudinal dimension only)
    n_units_stacked: int = Input(1, validator=Positive())

    #: Structural clearance between CubeSat envelope and bay walls [m]
    clearance: float = Input(0.030, validator=Positive(incl_zero=True))

    @Attribute
    def cubesat_dims(self):
        """Raw CubeSat dimensions (l, w, h) in metres."""
        return CUBESAT_STANDARDS[self.cubesat_standard]

    @Attribute
    def cs_length(self) -> float:
        """CubeSat dimension along fuselage X axis [m] (stacking axis)."""
        return self.cubesat_dims[2]  # 'height' in CDS = stacking axis

    @Attribute
    def cs_width(self) -> float:
        """CubeSat cross-section width [m]."""
        return self.cubesat_dims[0]

    @Attribute
    def cs_depth(self) -> float:
        """CubeSat cross-section depth [m]."""
        return self.cubesat_dims[1]

    @Attribute
    def required_longitudinal(self) -> float:
        """Longitudinal bay length including clearance and stacking [m]."""
        return self.cs_length * self.n_units_stacked + 2.0 * self.clearance

    @Attribute
    def required_lateral(self) -> float:
        """Required lateral (Y) bay width including clearance [m]."""
        return self.cs_width + 2.0 * self.clearance

    @Attribute
    def required_vertical(self) -> float:
        """Required vertical (Z) bay height including clearance [m]."""
        return self.cs_depth + 2.0 * self.clearance

    @Attribute
    def required_diameter(self) -> float:
        """
        Minimum circular cross-section diameter [m] that circumscribes the
        rectangular payload box, ensuring fit regardless of roll orientation.
        """
        return sqrt(self.required_lateral ** 2 + self.required_vertical ** 2)

    @Attribute
    def required_volume(self) -> float:
        """Bay box volume (clearance included) [m³]."""
        return (self.required_longitudinal
                * self.required_lateral
                * self.required_vertical)

    @Attribute
    def cubesat_volume(self) -> float:
        """Net CubeSat envelope volume (no clearance) [m³]."""
        return (self.cs_length * self.n_units_stacked
                * self.cs_width
                * self.cs_depth)


# ---------------------------------------------------------------------------
# AvionicsBay
# ---------------------------------------------------------------------------

class AvionicsBay(Base):
    """
    Avionics bay envelope that houses the flight computer, power conditioning
    unit, attitude control electronics, and RF systems.

    Clearance design rules:
        - Minimum gap to structural wall:  avionics_wall_clearance (≥ 15 mm)
        - Minimum gap to payload bay aft face:  payload_aft_clearance (≥ 30 mm)
          This separation prevents payload EMI affecting avionics and provides
          a thermal break between the science payload and electronics.
        - Minimum gap between PCBs/boxes:  internal_clearance (≥ 10 mm)

    The avionics module is sized as a rectangular box that fits inside the
    fuselage cylindrical inner cross-section with all clearances satisfied.
    """

    #: Fuselage inner diameter at the avionics bay location [m]
    fuselage_inner_diameter: float = Input(0.30, validator=Positive())

    #: Avionics box longest dimension (along fuselage axis) [m]
    avionics_box_length: float = Input(0.150, validator=Positive())

    #: Avionics box width (Y-axis) [m]
    avionics_box_width: float = Input(0.120, validator=Positive())

    #: Avionics box height (Z-axis) [m]
    avionics_box_height: float = Input(0.080, validator=Positive())

    #: Clearance between avionics box and inner fuselage wall [m]
    avionics_wall_clearance: float = Input(0.020, validator=Positive())

    #: Clearance between payload bay aft face and avionics bay fwd face [m]
    payload_aft_clearance: float = Input(0.040, validator=Positive())

    #: Additional internal clearance for wiring harness routing [m]
    wiring_clearance: float = Input(0.015, validator=Positive(incl_zero=True))

    @Attribute
    def total_bay_length(self) -> float:
        """
        Total axial length occupied by avionics bay [m].
        = payload separation + box + wiring margin
        """
        return (self.payload_aft_clearance
                + self.avionics_box_length
                + self.wiring_clearance)

    @Attribute
    def checked_wall_clearance(self) -> float:
        """
        Verify avionics box fits inside fuselage with required wall clearance.
        The diagonal of the avionics box cross-section must not exceed the
        fuselage inner diameter minus 2*wall_clearance.
        """
        max_allowed_diag = (self.fuselage_inner_diameter
                            - 2.0 * self.avionics_wall_clearance)
        actual_diag = sqrt(self.avionics_box_width ** 2
                           + self.avionics_box_height ** 2)
        if actual_diag > max_allowed_diag:
            msg = (
                f"Avionics box diagonal ({actual_diag * 1000:.1f} mm) exceeds "
                f"allowed envelope ({max_allowed_diag * 1000:.1f} mm) at "
                f"wall clearance {self.avionics_wall_clearance * 1000:.0f} mm. "
                "Reduce avionics box size or increase fuselage diameter."
            )
            warnings.warn(msg)
        return actual_diag

    @Attribute
    def envelope_diameter(self) -> float:
        """Circular cross-section envelope for the avionics bay [m]."""
        return self.fuselage_inner_diameter - 2.0 * self.avionics_wall_clearance


# ---------------------------------------------------------------------------
# Fuselage
# ---------------------------------------------------------------------------

class Fuselage(Base):
    """
    Rule-based parametric fuselage for a suborbital research spaceplane.

    Designed inside-out (payload first, then structure around it), following
    the standard conceptual design procedure.  Payload envelope drives
    inner diameter, wall depth gives outer diameter and fineness ratios
    set nose and tail lengths.

    Section layout (nose → tail):
        1. Nose cone              – empty / optional nose-cone payload
        2. Standard payload bay   – CubeSat primary science payload
        3. Avionics bay           – flight computer + electronics (with clearance)
        4. Propulsion bay         – tanks, lines, engine

    Design is driven by:
        - CubeSat standard payload requirements (1U / 3U / 6U / 12U)
        - Aerodynamic considerations (fineness ratios)
        - Structural considerations (wall depth)
        - Ground handling

    The three-section layout (nose / mid / tail) follows the standard
    top-view decomposition described in [Slide 51, Slide 86].
    """

    # ------------------------------------------------------------------
    # Payload
    # ------------------------------------------------------------------

    payload_bay: StandardPayloadBay = Input(StandardPayloadBay())

    # ------------------------------------------------------------------
    # Avionics
    # ------------------------------------------------------------------

    avionics: AvionicsBay = Input(AvionicsBay())

    # ------------------------------------------------------------------
    # Propulsion bay
    # ------------------------------------------------------------------

    #: Propulsion bay length [m] – overridden by PropulsionSystem output
    propulsion_bay_length: float = Input(1.20, validator=GreaterThan(0))

    # ------------------------------------------------------------------
    # Structure
    # ------------------------------------------------------------------

    structural_wall_depth: float = Input(0.05, validator=Between(0.02, 0.15))
    min_inner_diameter: float = Input(0.30)

    # ------------------------------------------------------------------
    # Nose and tail shaping
    # ------------------------------------------------------------------

    #: Nose fineness ratio: nose_length / outer_diameter  ≥ 1.5 for low wave drag
    nose_fineness: float = Input(1.8)

    #: Tail fineness ratio: tail_length / outer_diameter  ≥ 1.0 for no separation
    tail_fineness: float = Input(3.0)

    #: Show Tk pop-ups for soft-rule violations?
    popup_warnings: bool = Input(False)

    # ------------------------------------------------------------------
    # Inner / outer diameters
    # ------------------------------------------------------------------

    @Attribute
    def inner_diameter(self) -> float:
        """
        Inner (usable) fuselage diameter [m].
        Driven by payload diagonal, then checked against min_inner_diameter.
        """
        return max(self.payload_bay.required_diameter, self.min_inner_diameter)

    @Attribute
    def outer_diameter(self) -> float:
        """Outer fuselage diameter [m] = inner + 2 × wall layers."""
        return self.inner_diameter + 2.0 * self.structural_wall_depth

    @Attribute
    def inner_radius(self) -> float:
        return 0.5 * self.inner_diameter

    @Attribute
    def outer_radius(self) -> float:
        return 0.5 * self.outer_diameter

    # ------------------------------------------------------------------
    # Section lengths
    # ------------------------------------------------------------------

    @Attribute
    def cylindrical_length(self) -> float:
        """
        Total length of the constant-diameter mid-section [m].
        = payload_bay + avionics_bay + propulsion_bay
        """
        return (self.payload_bay.required_longitudinal
                + self.avionics.total_bay_length
                + self.propulsion_bay_length)

    @Attribute
    def nose_length(self) -> float:
        """Nose cone length [m] = nose_fineness × outer_diameter. [Slide 57]"""
        return self.nose_fineness * self.outer_diameter

    @Attribute
    def tail_length(self) -> float:
        """Tail cone length [m] = tail_fineness × outer_diameter. [Slide 56]"""
        return self.tail_fineness * self.outer_diameter

    @Attribute
    def total_length(self) -> float:
        """Total fuselage length [m]. [Slide 51]"""
        return self.nose_length + self.cylindrical_length + self.tail_length

    # ------------------------------------------------------------------
    # Soft-rule checks
    # ------------------------------------------------------------------

    @Attribute
    def checked_nose_fineness(self) -> float:
        if self.nose_fineness < 1.5:
            msg = (
                f"nose_fineness ({self.nose_fineness:.2f}) < 1.5 — blunt nose "
                "will have elevated wave drag at transonic speed. "
                "Recommend nose_fineness >= 1.5."
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Nose fineness warning", msg)
        return self.nose_fineness

    @Attribute
    def checked_tail_fineness(self) -> float:
        if self.tail_fineness < 1.0:
            msg = (
                f"tail_fineness ({self.tail_fineness:.2f}) < 1.0 — steep boattail "
                "will cause flow separation and high base drag. "
                "Recommend tail_fineness >= 1.0"
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Tail fineness warning", msg)
        return self.tail_fineness

    @Attribute
    def slenderness_ratio(self) -> float:
        """
        Fineness ratio: total_length / outer_diameter.
        Soft warning when outside the transonic spaceplane target of 20–25.
        """
        sr = self.total_length / self.outer_diameter
        if sr < 20.0:
            msg = (
                f"Fineness ratio {sr:.2f} < 20 — below the transonic spaceplane "
                "target (20-25). Pressure drag will be elevated."
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Slenderness ratio warning", msg)
        elif sr > 25.0:
            msg = (
                f"Fineness ratio {sr:.2f} > 25 — structural bending loads will "
                "be critical. [Slide 12]"
            )
            warnings.warn(msg)
        return sr

    # ------------------------------------------------------------------
    # X-positions (origin = nose tip)
    # ------------------------------------------------------------------

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
        """Primary (CubeSat) payload bay starts immediately after nose cone."""
        return self.x_nose_base

    @Attribute
    def x_avionics_start(self) -> float:
        """
        Avionics bay starts after the payload bay.
        The 'payload_aft_clearance' inside AvionicsBay already provides
        the required separation gap between payload and electronics.
        """
        return self.x_payload_bay_start + self.payload_bay.required_longitudinal

    @Attribute
    def x_propulsion_bay_start(self) -> float:
        """Propulsion bay starts after the avionics bay."""
        return self.x_avionics_start + self.avionics.total_bay_length

    # ------------------------------------------------------------------
    # Volumes
    # ------------------------------------------------------------------

    @Attribute
    def internal_cylindrical_volume(self) -> float:
        """Total inner volume of the cylindrical mid-section [m³]."""
        return pi * self.inner_radius ** 2 * self.cylindrical_length

    # ------------------------------------------------------------------
    # Profile-section helper
    # ------------------------------------------------------------------

    n_profile_points: int = Input(40)

    def _section_coordinates(self, radius: float, x: float):
        """Circular fuselage section in the YZ-plane at x-location."""
        angles = np.linspace(0, 2 * np.pi, self.n_profile_points, endpoint=True)
        return [(x,
                 radius * np.cos(theta),
                 radius * np.sin(theta))
                for theta in angles]

    # ------------------------------------------------------------------
    # Outer loft profiles
    # ------------------------------------------------------------------

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
        return [self.outer_nose_point,
                self.outer_nose_base,
                self.outer_tail_start,
                self.outer_tail_mid,
                self.outer_tail_tip]

    @Part
    def outer_loft(self):
        return LoftedSolid(profiles=self.outer_profiles)

    # ------------------------------------------------------------------
    # Inner loft profiles
    # ------------------------------------------------------------------

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
        return [self.inner_nose_start,
                self.inner_nose_base,
                self.inner_tail_start,
                self.inner_tail_mid,
                self.inner_tail_end]

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

    # ------------------------------------------------------------------
    # Nose-cone payload bay (EMPTY – optional additional payload)
    # ------------------------------------------------------------------

    @Part
    def nose_cone_payload_bay(self):
        """
        Optional payload volume inside the nose cone.
        LEFT EMPTY in the baseline – can host cameras, science instruments,
        or additional CubeSat units.  Rendered in yellow.
        """
        return NoseConePayloadBay(
            nose_base_diameter=self.outer_diameter,
            nose_length=self.nose_length,
            usable_fraction=0.60,
            wall_margin=self.structural_wall_depth + 0.010,
            x_nose_tip=0.0,
        )

    # ------------------------------------------------------------------
    # Primary CubeSat payload bay reference box
    # ------------------------------------------------------------------

    @Part
    def payload_bay_box(self):
        """
        Reference box for the CubeSat primary payload envelope.
        Used for visual interference checking with PropulsionSystem and Wing.
        Rendered in red (semi-transparent).
        """
        return Box(
            length=self.payload_bay.required_lateral,
            width=self.payload_bay.required_longitudinal,
            height=self.payload_bay.required_vertical,
            centered=True,
            position=Position(
                Point(
                    self.x_payload_bay_start
                    + 0.5 * self.payload_bay.required_longitudinal,
                    0,
                    0,
                )
            ),
            color="red",
            transparency=0.5,
        )

    # ------------------------------------------------------------------
    # Avionics bay reference box
    # ------------------------------------------------------------------

    @Part
    def avionics_bay_box(self):
        """
        Reference box for the avionics bay envelope.
        The box starts at x_avionics_start + payload_aft_clearance so the
        clearance gap between payload and avionics is visible in the GUI.
        Rendered in yellow (semi-transparent).
        """
        return Box(
            length=self.avionics.avionics_box_width,
            width=self.avionics.avionics_box_length,
            height=self.avionics.avionics_box_height,
            centered=True,
            position=Position(
                Point(
                    # Centre of the avionics box:
                    # x_avionics_start + payload_aft_clearance + half box length
                    self.x_avionics_start
                    + self.avionics.payload_aft_clearance
                    + 0.5 * self.avionics.avionics_box_length,
                    0,
                    0,
                )
            ),
            color="yellow",
            transparency=0.4,
        )

    @Part
    def avionics_clearance_gap(self):
        """
        Visual indicator of the clearance zone between payload aft face
        and the start of the avionics box.  Rendered as a thin orange box.
        """
        return Box(
            length=self.inner_diameter * 0.9,
            width=self.avionics.payload_aft_clearance,
            height=self.inner_diameter * 0.9,
            centered=True,
            position=Position(
                Point(
                    self.x_avionics_start
                    + 0.5 * self.avionics.payload_aft_clearance,
                    0,
                    0,
                )
            ),
            color="orange",
            transparency=0.7,
        )

    # ------------------------------------------------------------------
    # Summary attribute
    # ------------------------------------------------------------------

    @Attribute
    def summary(self) -> dict:
        return {
            # Diameters
            "inner_diameter_m":         round(self.inner_diameter, 3),
            "outer_diameter_m":         round(self.outer_diameter, 3),
            # Lengths
            "nose_length_m":            round(self.nose_length, 3),
            "cylindrical_length_m":     round(self.cylindrical_length, 3),
            "tail_length_m":            round(self.tail_length, 3),
            "total_length_m":           round(self.total_length, 3),
            # X-positions
            "x_payload_bay_start_m":    round(self.x_payload_bay_start, 3),
            "x_avionics_start_m":       round(self.x_avionics_start, 3),
            "x_propulsion_bay_start_m": round(self.x_propulsion_bay_start, 3),
            # Payload
            "cubesat_standard":         self.payload_bay.cubesat_standard,
            "payload_bay_volume_m3":    round(self.payload_bay.required_volume, 4),
            # Avionics
            "avionics_bay_length_m":    round(self.avionics.total_bay_length, 3),
            "avionics_payload_gap_m":   round(self.avionics.payload_aft_clearance, 3),
            # Slenderness
            "slenderness_ratio":        round(self.slenderness_ratio, 2),
        }


# ---------------------------------------------------------------------------
# Stand-alone entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from parapy.gui import display

    # 3U CubeSat payload example
    cs_bay = StandardPayloadBay(
        cubesat_standard="3U",
        n_units_stacked=1,
        clearance=0.030,
    )

    avi = AvionicsBay(
        fuselage_inner_diameter=0.30,
        avionics_box_length=0.150,
        avionics_box_width=0.120,
        avionics_box_height=0.080,
        avionics_wall_clearance=0.020,
        payload_aft_clearance=0.040,
        wiring_clearance=0.015,
    )

    fu = Fuselage(
        label="Spaceplane Fuselage (3U CubeSat)",
        payload_bay=cs_bay,
        avionics=avi,
        propulsion_bay_length=1.20,
        structural_wall_depth=0.05,
        min_inner_diameter=0.30,
        nose_fineness=1.8,
        tail_fineness=2.5,
        popup_warnings=False,
    )

    print("\n=== Fuselage Summary ===")
    for k, v in fu.summary.items():
        print(f"  {k:<40} {v}")

    display(fu)