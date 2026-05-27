"""
propulsion_system.py

Conceptual propulsion sizing AND tank geometry for the suborbital research
spaceplane, inspired by the Dawn Aerospace Mk-II Aurora.

All three classes live here because they form one physical subsystem:
    PropellantTank   – one cylindrical tank (volume → geometry)
    PropulsionSystem – sizing + mass budget + owns the two tanks as @Parts

tanks.py is no longer needed as a separate file.

The key output for Fuselage integration is:
    PropulsionSystem.tank_system_length  →  Fuselage.propulsion_bay_length

Delta-V budget (three phases):
    1. Powered ascent  – runway/pad to burnout altitude
    2. Ballistic zoom  – unpowered coast to apogee
    3. Descent burn    – small correction for runway glide approach
       Landing is ALWAYS an unpowered glide to a runway (spaceplane).

Sources:
    Sutton & Biblarz, "Rocket Propulsion Elements", 9th ed., Table 5-5
    Humble, Henry & Larson, "Space Propulsion Analysis and Design", Ch. 4
    Dawn Aerospace Mk-II Aurora public data (2023–2025)

Part of Team 24 KBE Assignment
Authors: Yasmine Mafoutsis, Paul-Ionut Huciu
"""

from math import exp, sqrt, pi
import warnings

from parapy.core import *
from parapy.core.validate import *
from parapy.geom import *


def generate_warning(header: str, msg: str):
    """Show a modal Tk warning dialog and wait for the user to dismiss it."""
    from tkinter import Tk, messagebox
    window = Tk()
    window.withdraw()
    messagebox.showwarning(header, msg)
    window.deiconify()
    window.destroy()
    window.quit()


# ---------------------------------------------------------------------------
# Propellant database – single source of truth
# ---------------------------------------------------------------------------

PROPELLANT_DB = {
    # Dawn Aerospace Mk-II Aurora baseline – self-pressurising, no helium needed
    "N2O_PROPYLENE": dict(
        isp=310.0,
        mixture_ratio=7.2,
        oxidizer_density=1220.0,   # liquid N2O at ~20 °C [kg/m³]
        fuel_density=614.0,        # liquid propylene at ~20 °C [kg/m³]
        self_pressurising=True,
    ),
    # Used in early Mk-II Aurora rocket-powered tests (March 2023)
    "HTP_KEROSENE": dict(
        isp=319.0,
        mixture_ratio=7.3,
        oxidizer_density=1440.0,
        fuel_density=806.0,
        self_pressurising=False,
    ),
    # Classical high-performance option for trade studies
    "LOX_KEROSENE": dict(
        isp=311.0,
        mixture_ratio=2.56,
        oxidizer_density=1140.0,
        fuel_density=806.0,
        self_pressurising=False,
    ),
}


# ---------------------------------------------------------------------------
# PropellantTank
# ---------------------------------------------------------------------------

class PropellantTank(Base):
    """
    One propellant tank with a cylindrical midsection and two blended
    hemispherical end caps (a "capsule" or "stadium" pressure vessel).

    Volume accounting:
        V_required = V_cylinder  +  V_two_caps
                   = π r² L_cyl  +  (4/3)π r³
    → L_cyl = (V_required − (4/3)π r³) / (π r²)

    The total envelope length is: L_total = L_cyl + 2 · R_outer
    (each cap contributes one outer radius of axial length).

    Owned as a @Part by PropulsionSystem – never instantiated standalone.
    """

    required_volume:    float = Input(1000.05,  validator=Positive())
    max_outer_diameter: float = Input(0.10,  validator=Positive())
    wall_thickness:     float = Input(0.003, validator=Positive())
    x_start:            float = Input(0.0)
    color:              str   = Input("orange")
    popup_warnings:     bool  = Input(False)

    @Attribute
    def outer_diameter(self):
        return self.max_outer_diameter

    @Attribute
    def inner_diameter(self):
        value = self.outer_diameter - 2.0 * self.wall_thickness
        if value <= 0:
            raise ValueError(
                f"Wall thickness ({self.wall_thickness} m) too large for "
                f"outer diameter ({self.outer_diameter} m)."
            )
        return value

    @Attribute
    def inner_radius(self):
        return 0.5 * self.inner_diameter

    @Attribute
    def outer_radius(self):
        return 0.5 * self.outer_diameter

    # ------------------------------------------------------------------
    # Capsule volume accounting
    # ------------------------------------------------------------------

    @Attribute
    def cap_volume(self):
        """Volume of both hemispherical end caps combined (= one full sphere)
        using the inner radius – this is the propellant-carrying volume."""
        return (4.0 / 3.0) * pi * self.inner_radius ** 3

    @Attribute
    def cylindrical_length(self):
        """
        Length of the cylindrical midsection [m].
        Sized so that V_caps + V_cylinder = required_volume exactly.
            V_caps   = (4/3)π r_i³  (two hemispheres = one sphere)
            L_cyl    = (V_required − V_caps) / (π r_i²)
        """
        net = self.required_volume - self.cap_volume
        if net < 0:
            msg = (
                f"Tank diameter too large: cap volume "
                f"({self.cap_volume * 1e3:.3f} L) exceeds required volume "
                f"({self.required_volume * 1e3:.3f} L) for outer_diameter="
                f"{self.outer_diameter * 1e3:.1f} mm. "
                "Reduce tank_diameter_fraction (try 0.40 or less)."
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Tank diameter too large", msg)
            return 0.0
        return net / (pi * self.inner_radius ** 2)

    @Attribute
    def total_length(self):
        """Full tank envelope length including both hemispherical caps [m].
        = L_cyl + 2 · R_outer  (each cap protrudes one outer radius)."""
        return self.cylindrical_length + 2.0 * self.outer_radius

    @Attribute
    def x_center(self):
        return self.x_start + 0.5 * self.total_length

    @Attribute
    def checked_aspect_ratio(self):
        """
        Tank L/D soft check (total length / outer diameter). Typical range 0.5–5.0.
        Very slender tanks (L/D > 5) have high bending loads;
        very stubby tanks (L/D < 0.5) are hard to seal and integrate.
        """
        ld = self.total_length / self.outer_diameter
        if ld > 5.0:
            msg = (f"Tank L/D = {ld:.2f} > 5.0 — very slender. "
                   "Consider increasing tank diameter or splitting into two tanks.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Tank aspect ratio", msg)
        elif ld < 0.5:
            msg = f"Tank L/D = {ld:.2f} < 0.5 — very stubby. Check volume and diameter inputs."
            warnings.warn(msg)
        return ld

    # ------------------------------------------------------------------
    # Geometry positions for the three capsule components
    # ------------------------------------------------------------------

    @Attribute
    def cylinder_position(self):
        """Start of the cylindrical midsection: offset one outer radius from x_start
        to leave room for the forward hemispherical cap."""
        return Position(Point(self.x_start + self.outer_radius, 0, 0))

    @Attribute
    def left_cap_center(self):
        """Centre of the forward (left) hemispherical cap.
        Coincides with the left face of the cylinder midsection."""
        return Position(Point(self.x_start + self.outer_radius, 0, 0))

    @Attribute
    def right_cap_center(self):
        """Centre of the aft (right) hemispherical cap.
        Coincides with the right face of the cylinder midsection."""
        return Position(Point(
            self.x_start + self.outer_radius + self.cylindrical_length, 0, 0
        ))

    # ------------------------------------------------------------------
    # Capsule geometry – building blocks (suppressed from model tree)
    # ------------------------------------------------------------------

    @Part(in_tree=False)
    def _tank_cylinder(self):
        """Cylindrical midsection (internal building block)."""
        return Cylinder(
            radius=self.outer_radius,
            height=self.cylindrical_length,
            position=rotate(self.cylinder_position, "y", 90, deg=True),
        )

    @Part(in_tree=False)
    def _left_sphere(self):
        """Full sphere at the forward end – only the outer half is exposed."""
        return Sphere(
            radius=self.outer_radius,
            position=self.left_cap_center,
        )

    @Part(in_tree=False)
    def _right_sphere(self):
        """Full sphere at the aft end – only the outer half is exposed."""
        return Sphere(
            radius=self.outer_radius,
            position=self.right_cap_center,
        )

    @Part(in_tree=False)
    def _cylinder_with_left_cap(self):
        """Intermediate Boolean union: cylinder ∪ forward cap."""
        return Fused(
            shape_in=self._tank_cylinder,
            tool=self._left_sphere,
        )

    # ------------------------------------------------------------------
    # Final capsule solid (replaces the old plain cylinder @Part)
    # ------------------------------------------------------------------

    @Part
    def cylinder(self):
        """
        Capsule-shaped tank body: cylindrical midsection with fully blended
        hemispherical end caps.  Boolean union of cylinder + two spheres
        centred at each end face, giving a smooth, seam-free solid.
        """
        return Fused(
            shape_in=self._cylinder_with_left_cap,
            tool=self._right_sphere,
            color=self.color,
            transparency=0.3,
        )

    @Attribute
    def summary(self):
        return {
            "required_volume_m3":   round(self.required_volume, 4),
            "outer_diameter_m":     round(self.outer_diameter, 3),
            "inner_diameter_m":     round(self.inner_diameter, 3),
            "cap_volume_m3":        round(self.cap_volume, 4),
            "cylindrical_length_m": round(self.cylindrical_length, 3),
            "total_length_m":       round(self.total_length, 3),
            "tank_LD_ratio":        round(self.checked_aspect_ratio, 2),
            "x_start_m":            round(self.x_start, 3),
            "x_center_m":           round(self.x_center, 3),
        }


# ---------------------------------------------------------------------------
# PropulsionSystem  (owns PropellantTank parts internally)
# ---------------------------------------------------------------------------

class PropulsionSystem(Base):
    """
    Conceptual propulsion sizing for a reusable uncrewed suborbital spaceplane.

    Owns the oxidiser and fuel tanks directly as @Parts, so the full
    propulsion subsystem (sizing + geometry) lives in one class.

    Key outputs for Fuselage integration:
        tank_system_length   →  Fuselage.propulsion_bay_length  (sizing loop)
        max_tank_diameter    →  set from Fuselage.inner_diameter (via Spaceplane)
        x_tanks_start        →  set from Fuselage.x_propulsion_bay_start (via Spaceplane)
        gross_mass           →  Spaceplane summary / MassCGAnalysis
        thrust               →  T/W check
    """

    # ------------------------------------------------------------------
    # Propulsion inputs
    # ------------------------------------------------------------------

    propulsion_type: str = Input(
        "N2O_PROPYLENE",
        validator=OneOf(list(PROPELLANT_DB.keys()))
    )

    #: Payload mass [kg] – flows in from Spaceplane
    payload_mass: float = Input(4.0, validator=Positive())

    #: Target apogee altitude [m] – flows in from Spaceplane
    target_apogee: float = Input(100e3, validator=Positive())

    #: Required Mach number at burnout – governs zoom_delta_v
    max_burnout_mach: float = Input(3.5, validator=Between(1.0, 5.0))

    #: Liftoff thrust-to-weight ratio [-]
    thrust_to_weight: float = Input(3.0, validator=Between(1.3, 6.0))

    #: Launch mode – affects structural fraction and T/W soft-check bounds
    #: Both modes return via unpowered glide to a runway (spaceplane).
    #: "horizontal": wings sized for takeoff lift + landing → struct_frac = 0.28
    #: "vertical":   wings sized for landing only → struct_frac = 0.22
    launch_mode: str = Input(
        "horizontal",
        validator=OneOf(["horizontal", "vertical"])
    )

    # ------------------------------------------------------------------
    # Tank geometry inputs  (formerly in TankSystem)
    # Set by Spaceplane from fuselage geometry – not free user inputs
    # ------------------------------------------------------------------

    #: Maximum tank outer diameter [m] = fraction × fuselage inner diameter
    max_tank_diameter: float = Input(0.20, validator=Positive())

    #: Tank wall thickness [m]
    tank_wall_thickness: float = Input(0.003, validator=Positive())

    #: Gap between oxidiser and fuel tank [m]
    intertank_spacing: float = Input(0.05, validator=Positive(incl_zero=True))

    #: X-position where the tank stack starts [m] = fuselage.x_propulsion_bay_start
    x_tanks_start: float = Input(0.0)

    popup_warnings: bool = Input(False)

    # ------------------------------------------------------------------
    # Physical constants (never Inputs)
    # ------------------------------------------------------------------

    _G0         = 9.80665
    _SOUND_30KM = 300.0    # ISA speed of sound at ~30 km [m/s]

    # ------------------------------------------------------------------
    # Embedded engineering knowledge
    # ------------------------------------------------------------------

    BURNOUT_ALTITUDE    = 30_000.0  # [m]
    DRAG_LOSS_ASCENT    = 200.0     # [m/s]
    GRAVITY_LOSS_ASCENT = 200.0     # [m/s]
    DESCENT_DV          = 80.0      # [m/s]  glide approach correction burn
    PROPELLANT_MARGIN   = 0.05      # [-]

    ULLAGE_SELF_PRESS    = 0.02
    ULLAGE_NON_SELFPRESS = 0.05

    # Structural fraction by launch mode
    _STRUCT_FRACTION = {"horizontal": 0.28, "vertical": 0.22}

    # Recommended T/W range by launch mode (soft check)
    _TW_RANGE = {"horizontal": (1.3, 2.5), "vertical": (2.0, 5.0)}

    # ------------------------------------------------------------------
    # Launch-mode derived quantities
    # ------------------------------------------------------------------

    @Attribute
    def structural_fraction(self):
        """
        Structural mass fraction [-] by launch mode.

        Horizontal: 0.28 – wing sized for takeoff lift + landing glide.
        Vertical:   0.22 – wing sized for landing glide only (smaller wing).
        Both modes land via unpowered glide to a runway.
        """
        return self._STRUCT_FRACTION[self.launch_mode]

    @Attribute
    def checked_thrust_to_weight(self):
        """
        Soft check: T/W vs launch mode.
        Horizontal: 1.3–2.5 (aerodynamic lift assists early climb).
        Vertical:   2.0–5.0 (must clear pad from rest under thrust alone).
        """
        lo, hi = self._TW_RANGE[self.launch_mode]
        if not (lo <= self.thrust_to_weight <= hi):
            msg = (
                f"thrust_to_weight ({self.thrust_to_weight:.2f}) is outside "
                f"the recommended range [{lo}, {hi}] for {self.launch_mode} takeoff."
            )
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("T/W warning", msg)
        return self.thrust_to_weight

    # ------------------------------------------------------------------
    # Propellant property lookup
    # ------------------------------------------------------------------

    @Attribute
    def _props(self):
        return PROPELLANT_DB[self.propulsion_type]

    @Attribute
    def isp(self):
        """Vacuum Isp [s]. Source: Sutton & Biblarz Table 5-5."""
        return self._props["isp"]

    @Attribute
    def mixture_ratio(self):
        """Oxidiser-to-fuel mass ratio (O/F) [-]."""
        return self._props["mixture_ratio"]

    @Attribute
    def oxidizer_density(self):
        return self._props["oxidizer_density"]

    @Attribute
    def fuel_density(self):
        return self._props["fuel_density"]

    @Attribute
    def is_self_pressurising(self):
        return self._props["self_pressurising"]

    @Attribute
    def ullage_fraction(self):
        return (self.ULLAGE_SELF_PRESS if self.is_self_pressurising
                else self.ULLAGE_NON_SELFPRESS)

    # ------------------------------------------------------------------
    # Delta-V budget
    # ------------------------------------------------------------------

    @Attribute
    def burnout_speed(self):
        """Speed at burnout to meet max_burnout_mach [m/s]."""
        return self.max_burnout_mach * self._SOUND_30KM

    @Attribute
    def zoom_delta_v(self):
        """
        Speed at burnout needed to coast to apogee [m/s].
        Governed by whichever is larger: Mach target or altitude target.
        Energy conservation: v = sqrt(2 * g0 * delta_h).
        """
        dh = self.target_apogee - self.BURNOUT_ALTITUDE
        if dh <= 0:
            warnings.warn(
                f"target_apogee ({self.target_apogee/1e3:.1f} km) <= "
                f"burnout altitude ({self.BURNOUT_ALTITUDE/1e3:.0f} km)."
            )
            return self.burnout_speed
        return max(sqrt(2.0 * self._G0 * dh), self.burnout_speed)

    @Attribute
    def ascent_delta_v_ideal(self):
        return self.zoom_delta_v + self.DRAG_LOSS_ASCENT + self.GRAVITY_LOSS_ASCENT

    @Attribute
    def required_delta_v(self):
        """
        Total mission delta-V [m/s].
        Ascent + descent correction + margin (applied once).
        No landing delta-V – landing is always an unpowered glide.
        """
        return (self.ascent_delta_v_ideal + self.DESCENT_DV) * (1.0 + self.PROPELLANT_MARGIN)

    # ------------------------------------------------------------------
    # Mass sizing – gross_mass is an OUTPUT
    # ------------------------------------------------------------------

    @Attribute
    def mass_ratio(self):
        return exp(self.required_delta_v / (self._G0 * self.isp))

    @Attribute
    def propellant_fraction(self):
        return 1.0 - 1.0 / self.mass_ratio

    @Attribute
    def gross_mass(self):
        """
        Gross liftoff mass [kg] – DERIVED, never a user input.
        gross = payload / (1 - propellant_fraction - structural_fraction)
        """
        denom = 1.0 - self.propellant_fraction - self.structural_fraction
        if denom <= 0:
            raise ValueError(
                f"Vehicle not feasible: propellant fraction "
                f"({self.propellant_fraction:.3f}) + structural fraction "
                f"({self.structural_fraction:.3f}) >= 1.0. "
                "Reduce apogee, burnout Mach, or choose higher-Isp propellant."
            )
        return self.payload_mass / denom

    @Attribute
    def structural_mass(self):
        return self.structural_fraction * self.gross_mass

    @Attribute
    def propellant_mass(self):
        return self.propellant_fraction * self.gross_mass

    # ------------------------------------------------------------------
    # Propellant split
    # ------------------------------------------------------------------

    @Attribute
    def fuel_mass(self):
        return self.propellant_mass / (1.0 + self.mixture_ratio)

    @Attribute
    def oxidizer_mass(self):
        return self.propellant_mass - self.fuel_mass

    @Attribute
    def fuel_volume(self):
        """Fuel tank volume including ullage [m³]."""
        return (self.fuel_mass / self.fuel_density) / (1.0 - self.ullage_fraction)

    @Attribute
    def oxidizer_volume(self):
        """Oxidiser tank volume including ullage [m³]."""
        return (self.oxidizer_mass / self.oxidizer_density) / (1.0 - self.ullage_fraction)

    @Attribute
    def total_propellant_volume(self):
        return self.fuel_volume + self.oxidizer_volume

    @Attribute
    def thrust(self):
        return self.checked_thrust_to_weight * self.gross_mass * self._G0

    # ------------------------------------------------------------------
    # Tank x-positions (derived internally – no external TankSystem needed)
    # ------------------------------------------------------------------

    @Attribute
    def x_oxidizer_tank(self):
        """Oxidiser tank starts at the front of the propulsion bay."""
        return self.x_tanks_start

    @Attribute
    def x_fuel_tank(self):
        """Fuel tank starts after oxidiser tank + intertank gap."""
        return (self.x_oxidizer_tank
                + self.oxidizer_tank.total_length
                + self.intertank_spacing)

    @Attribute
    def tank_system_length(self):
        """
        Total length of the tank stack [m].
        This is the key output that sets Fuselage.propulsion_bay_length.
        Change propellant type → different volumes → different tank lengths
        → different fuselage length.  All automatic via ParaPy dependency graph.
        """
        return (self.x_fuel_tank
                + self.fuel_tank.total_length
                - self.x_tanks_start)

    # ------------------------------------------------------------------
    # Tank Parts  (formerly in TankSystem / tanks.py)
    # ------------------------------------------------------------------

    @Part
    def oxidizer_tank(self):
        """
        Oxidiser tank – placed forward in the propulsion bay.
        Forward placement of the denser oxidiser keeps the CG closer to
        the vehicle centre, improving longitudinal stability margin.
        Sized directly from oxidizer_volume.
        """
        return PropellantTank(
            required_volume=self.oxidizer_volume,
            max_outer_diameter=self.max_tank_diameter,
            wall_thickness=self.tank_wall_thickness,
            x_start=self.x_oxidizer_tank,
            color="blue",
            popup_warnings=self.popup_warnings,
        )

    @Part
    def fuel_tank(self):
        """
        Fuel tank – placed aft of the oxidiser tank.
        Sized directly from fuel_volume.
        """
        return PropellantTank(
            required_volume=self.fuel_volume,
            max_outer_diameter=self.max_tank_diameter,
            wall_thickness=self.tank_wall_thickness,
            x_start=self.x_fuel_tank,
            color="green",
            popup_warnings=self.popup_warnings,
        )

    # ------------------------------------------------------------------
    # Soft cross-parameter checks
    # ------------------------------------------------------------------

    @Attribute
    def checked_payload_fraction(self):
        """Payload fraction. Dawn Mk-II B: ~4/280 ≈ 1.4 %."""
        pf = self.payload_mass / self.gross_mass
        if pf < 0.01:
            msg = (f"Payload fraction {pf:.2%} < 1 % — reduce apogee, "
                   "burnout Mach, or use higher-Isp propellant.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Low payload fraction", msg)
        elif pf > 0.10:
            msg = (f"Payload fraction {pf:.2%} > 10 % — verify structural "
                   "fraction assumption for a reusable vehicle.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("High payload fraction", msg)
        return pf

    @Attribute
    def checked_propellant_fraction(self):
        pf = self.propellant_fraction
        if pf > 0.65:
            msg = (f"Propellant fraction {pf:.3f} > 0.65 — heavy for a "
                   "reusable spaceplane. Consider higher-Isp propellant or "
                   "lower mission targets.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("High propellant fraction", msg)
        return pf

    @Attribute
    def checked_burnout_mach(self):
        m = self.zoom_delta_v / self._SOUND_30KM
        if m < 1.0:
            msg = (f"Effective burnout Mach {m:.2f} < 1.0 — vehicle subsonic "
                   "at burnout. Increase apogee or burnout Mach target.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Subsonic burnout", msg)
        return m

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------

    @Attribute
    def summary(self):
        return {
            # Configuration
            "launch_mode":                  self.launch_mode,
            "landing_mode":                 "unpowered glide → runway (both modes)",
            "propulsion_type":              self.propulsion_type,
            "isp_s":                        round(self.isp, 1),
            "mixture_ratio_OF":             round(self.mixture_ratio, 2),
            "self_pressurising":            self.is_self_pressurising,
            # Delta-V
            "max_burnout_mach":             round(self.max_burnout_mach, 2),
            "burnout_speed_m_s":            round(self.burnout_speed, 1),
            "zoom_delta_v_m_s":             round(self.zoom_delta_v, 1),
            "required_delta_v_m_s":         round(self.required_delta_v, 1),
            # Mass
            "structural_fraction":          round(self.structural_fraction, 3),
            "propellant_fraction":          round(self.checked_propellant_fraction, 3),
            "gross_mass_kg":                round(self.gross_mass, 1),
            "structural_mass_kg":           round(self.structural_mass, 1),
            "propellant_mass_kg":           round(self.propellant_mass, 1),
            "fuel_mass_kg":                 round(self.fuel_mass, 1),
            "oxidizer_mass_kg":             round(self.oxidizer_mass, 1),
            # Tank volumes and geometry
            "fuel_volume_m3":               round(self.fuel_volume, 4),
            "oxidizer_volume_m3":           round(self.oxidizer_volume, 4),
            "total_propellant_volume_m3":   round(self.total_propellant_volume, 4),
            "tank_system_length_m":         round(self.tank_system_length, 3),
            "max_tank_diameter_m":          round(self.max_tank_diameter, 3),
            # Thrust
            "thrust_N":                     round(self.thrust, 1),
            "thrust_to_weight":             round(self.thrust_to_weight, 2),
            "payload_fraction":             round(self.checked_payload_fraction, 4),
        }


if __name__ == "__main__":
    from parapy.gui import display

    prop = PropulsionSystem(
        label="Propulsion System",
        launch_mode="horizontal",
        propulsion_type="N2O_PROPYLENE",
        payload_mass=15.0,
        target_apogee=100e3,
        max_burnout_mach=3.5,
        thrust_to_weight=1.5,
        max_tank_diameter=0.09,
        tank_wall_thickness=0.003,
        intertank_spacing=0.05,
        x_tanks_start=0.0,
    )

    print("\n=== Propulsion System Summary ===")
    for k, v in prop.summary.items():
        print(f"  {k:<42} {v}")
    print(f"\n  oxidizer_tank: {prop.oxidizer_tank.summary}")
    print(f"  fuel_tank:     {prop.fuel_tank.summary}")

    display(prop)