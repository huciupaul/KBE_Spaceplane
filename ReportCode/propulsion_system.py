"""
propulsion_system.py

Conceptual propulsion sizing AND tank geometry for the suborbital research
spaceplane, inspired by the Dawn Aerospace Mk-II Aurora.

Launch mode: HORIZONTAL only (runway takeoff and landing glide).

Tank splitting rule:
    If a tank's L/D would exceed max_tank_ld (default 5.0), the total
    volume is automatically split across n_tanks equal sub-tanks so that
    each sub-tank's L/D is within the acceptable range.
    n_tanks is chosen as the smallest integer (1–4) that brings L/D <= max_tank_ld.
    Sub-tanks are placed consecutively with the same intertank_spacing between
    each pair.  The oxidiser stack (blue) sits forward; the fuel stack (green)
    sits aft.  The spacing between the two stacks (oxidiser aft face → fuel
    forward face) is always exactly intertank_spacing, regardless of how many
    sub-tanks each stack contains.


Part of Team 24 KBE Assignment
"""

from math import exp, sqrt, pi, ceil
import warnings

from parapy.core import *
from parapy.core.validate import *
from parapy.geom import *


def generate_warning(header: str, msg: str):
    from tkinter import Tk, messagebox
    window = Tk()
    window.withdraw()
    messagebox.showwarning(header, msg)
    window.deiconify()
    window.destroy()
    window.quit()


# ---------------------------------------------------------------------------
# Propellant database
# ---------------------------------------------------------------------------

PROPELLANT_DB = {
    "N2O_PROPYLENE": dict(
        isp=310.0, mixture_ratio=7.2,
        oxidizer_density=1220.0, fuel_density=614.0,
        self_pressurising=True,
    ),
    "HTP_KEROSENE": dict(
        isp=319.0, mixture_ratio=7.3,
        oxidizer_density=1440.0, fuel_density=806.0,
        self_pressurising=False,
    ),
    "LOX_KEROSENE": dict(
        isp=311.0, mixture_ratio=2.56,
        oxidizer_density=1140.0, fuel_density=806.0,
        self_pressurising=False,
    ),
}


# ---------------------------------------------------------------------------
# PropellantTank  – one physical tank unit
# ---------------------------------------------------------------------------

class PropellantTank(Base):
    """
    One propellant tank: cylindrical midsection + two hemispherical end caps.

    Diameter auto-optimisation:
        Starts at max_outer_diameter, steps down by 1 % of
        fuselage_inner_diameter until cylindrical_length >= min_cylindrical_length.

   """

    required_volume:         float = Input(0.001,  validator=Positive())
    max_outer_diameter:      float = Input(0.10,   validator=Positive())
    fuselage_inner_diameter: float = Input(0.20,   validator=Positive())
    wall_thickness:          float = Input(0.001,  validator=Positive()) #for manufacturability purposes
    x_start:                 float = Input(0.0)
    color:                   str   = Input("orange")
    popup_warnings:          bool  = Input(False)
    min_cylindrical_length:  float = Input(0.020,  validator=Positive())
    q_max:                   float = Input(50e3,   validator=Positive())
    sigma_allow:             float = Input(276e6,  validator=Positive())
    rho_wall:                float = Input(2700.0, validator=Positive())
    factor_of_safety:        float = Input(1.5,    validator=Between(1.0, 3.0))
    propellant_mass_in_tank: float = Input(1.0,    validator=Positive())

    # ── Diameter auto-optimisation ────────────────────────────────────

    @Attribute
    def actual_outer_diameter(self):
        """
        Largest diameter where cylindrical_length >= min_cylindrical_length.
        Steps down by 1 % of fuselage_inner_diameter per iteration.
        """
        step = 0.01 * self.fuselage_inner_diameter
        d = self.max_outer_diameter
        while d > 0.10 * self.fuselage_inner_diameter:
            r_i = d / 2.0 - self.wall_thickness
            if r_i <= 0:
                d -= step
                continue
            v_caps = (4.0 / 3.0) * pi * r_i ** 3
            v_cyl  = self.required_volume - v_caps
            if v_cyl > 0 and v_cyl / (pi * r_i ** 2) >= self.min_cylindrical_length:
                if d < self.max_outer_diameter:
                    msg = (
                        f"Tank diameter reduced from "
                        f"{self.max_outer_diameter * 1e3:.1f} mm to "
                        f"{d * 1e3:.1f} mm (cylindrical section "
                        f"{v_cyl / (pi * r_i**2) * 1e3:.1f} mm >= "
                        f"{self.min_cylindrical_length * 1e3:.0f} mm min)."
                    )
                    warnings.warn(msg)
                    if self.popup_warnings:
                        generate_warning("Tank diameter reduced", msg)
                return d
            d -= step
        msg = (f"No valid diameter found for volume "
               f"{self.required_volume * 1e3:.3f} L. Using minimum.")
        warnings.warn(msg)
        return max(d, 0.10 * self.fuselage_inner_diameter)

    @Attribute
    def outer_diameter(self):
        return self.actual_outer_diameter

    @Attribute
    def inner_diameter(self):
        v = self.outer_diameter - 2.0 * self.wall_thickness
        if v <= 0:
            raise ValueError(
                f"Wall thickness ({self.wall_thickness} m) too large for "
                f"outer diameter ({self.outer_diameter} m).")
        return v

    @Attribute
    def inner_radius(self):
        return 0.5 * self.inner_diameter

    @Attribute
    def outer_radius(self):
        return 0.5 * self.outer_diameter

    @Attribute
    def cap_volume(self):
        return (4.0 / 3.0) * pi * self.inner_radius ** 3

    @Attribute
    def cylindrical_length(self):
        net = self.required_volume - self.cap_volume
        if net < 0:
            warnings.warn("Cap volume exceeds required volume after optimisation.")
            return self.min_cylindrical_length
        return net / (pi * self.inner_radius ** 2)

    @Attribute
    def total_length(self):
        return self.cylindrical_length + 2.0 * self.outer_radius

    @Attribute
    def x_center(self):
        return self.x_start + 0.5 * self.total_length

    @Attribute
    def ld_ratio(self):
        return self.total_length / self.outer_diameter

    # ── Structural mass ───────────────────────────────────────────────

    @Attribute
    def t_wall_hoop(self):
        p_design = self.q_max * self.factor_of_safety
        return max(p_design * self.inner_radius / self.sigma_allow,
                   self.wall_thickness)

    @Attribute
    def structural_mass(self):
        t       = self.t_wall_hoop
        lateral = pi * self.inner_diameter * self.cylindrical_length
        caps    = 4.0 * pi * self.inner_radius ** 2
        return self.rho_wall * t * (lateral + caps)


    # ── Geometry positions ────────────────────────────────────────────

    @Attribute
    def cylinder_position(self):
        return Position(Point(self.x_start + self.outer_radius, 0, 0))

    @Attribute
    def left_cap_center(self):
        return Position(Point(self.x_start + self.outer_radius, 0, 0))

    @Attribute
    def right_cap_center(self):
        return Position(Point(
            self.x_start + self.outer_radius + self.cylindrical_length, 0, 0))

    @Part(in_tree=False)
    def _tank_cylinder(self):
        return Cylinder(
            radius=self.outer_radius,
            height=self.cylindrical_length,
            position=rotate(self.cylinder_position, "y", 90, deg=True),
        )

    @Part(in_tree=False)
    def _left_sphere(self):
        return Sphere(radius=self.outer_radius, position=self.left_cap_center)

    @Part(in_tree=False)
    def _right_sphere(self):
        return Sphere(radius=self.outer_radius, position=self.right_cap_center)

    @Part(in_tree=False)
    def _cylinder_with_left_cap(self):
        return Fused(shape_in=self._tank_cylinder, tool=self._left_sphere)

    @Part
    def cylinder(self):
        """Capsule tank: cylinder fused with two hemispherical end caps."""
        return Fused(
            shape_in=self._cylinder_with_left_cap,
            tool=self._right_sphere,
            color=self.color,
            transparency=0.3,
        )

    @Attribute
    def summary(self):
        return {
            "required_volume_L":        round(self.required_volume * 1e3, 3),
            "actual_outer_diameter_mm": round(self.actual_outer_diameter * 1e3, 1),
            "inner_diameter_mm":        round(self.inner_diameter * 1e3, 1),
            "cylindrical_length_mm":    round(self.cylindrical_length * 1e3, 1),
            "total_length_mm":          round(self.total_length * 1e3, 1),
            "ld_ratio":                 round(self.ld_ratio, 2),
            "t_wall_hoop_mm":           round(self.t_wall_hoop * 1e3, 2),
            "structural_mass_kg":       round(self.structural_mass, 3),
            "x_start_m":                round(self.x_start, 3),
        }


# ---------------------------------------------------------------------------
# TankStack  – one propellant type, auto-split into 1-4 sub-tanks
# ---------------------------------------------------------------------------

class TankStack(Base):
    """
    Stack of 1–4 equal sub-tanks for one propellant.

    Splitting rule:
        Compute n_tanks = smallest integer in [1, max_tanks] such that
        each sub-tank's L/D <= max_tank_ld.  All sub-tanks are identical
        (equal volume, same diameter).  They are placed consecutively with
        intertank_spacing between each pair.

    The x_start of this stack is the forward face of sub-tank[0].
    The aft face of the last sub-tank defines the stack end (x_end).
    """

    total_volume:            float = Input(0.001, validator=Positive())
    total_propellant_mass:   float = Input(1.0,   validator=Positive())
    max_outer_diameter:      float = Input(0.10,  validator=Positive())
    fuselage_inner_diameter: float = Input(0.20,  validator=Positive())
    wall_thickness:          float = Input(0.003, validator=Positive())
    x_start:                 float = Input(0.0)
    color:                   str   = Input("blue")
    intertank_spacing:       float = Input(0.050, validator=Positive(incl_zero=True))
    min_cylindrical_length:  float = Input(0.020, validator=Positive())
    max_tank_ld:             float = Input(5.0,   validator=Positive())
    max_tanks:               int   = Input(4,     validator=Positive())
    q_max:                   float = Input(50e3,  validator=Positive())
    sigma_allow:             float = Input(276e6, validator=Positive())
    rho_wall:                float = Input(2700.0, validator=Positive())
    factor_of_safety:        float = Input(1.5,   validator=Between(1.0, 3.0))
    popup_warnings:          bool  = Input(False)

    # ── Splitting decision ────────────────────────────────────────────

    @Attribute
    def n_tanks(self):
        """
        Minimum number of sub-tanks (1-max_tanks) so that each sub-tank's
        L/D <= max_tank_ld.

        For each candidate n, a temporary PropellantTank is instantiated
        (off-tree, volume = total_volume / n) and its L/D is checked.
        The first n that satisfies the L/D limit is used.
        If even max_tanks is not sufficient a warning is issued and
        max_tanks is returned.
        """
        for n in range(1, self.max_tanks + 1):
            sub_vol = self.total_volume / n
            sub_mass = self.total_propellant_mass / n
            # Instantiate a temporary tank to get the actual L/D
            # (diameter optimisation may reduce the diameter)
            tmp = PropellantTank(
                required_volume=sub_vol,
                max_outer_diameter=self.max_outer_diameter,
                fuselage_inner_diameter=self.fuselage_inner_diameter,
                wall_thickness=self.wall_thickness,
                min_cylindrical_length=self.min_cylindrical_length,
                q_max=self.q_max,
                sigma_allow=self.sigma_allow,
                rho_wall=self.rho_wall,
                factor_of_safety=self.factor_of_safety,
                propellant_mass_in_tank=sub_mass,
            )
            if tmp.ld_ratio <= self.max_tank_ld:
                if n > 1:
                    msg = (
                        f"{self.color} tank split into {n} sub-tanks "
                        f"(single-tank L/D would exceed {self.max_tank_ld:.1f}). "
                        f"Each sub-tank: volume={sub_vol * 1e3:.3f} L, "
                        f"L/D={tmp.ld_ratio:.2f}."
                    )
                    warnings.warn(msg)
                    if self.popup_warnings:
                        generate_warning("Tank split", msg)
                return n
        msg = (
            f"{self.color} tank: even {self.max_tanks} sub-tanks cannot "
            f"achieve L/D <= {self.max_tank_ld:.1f}. Using {self.max_tanks}."
        )
        warnings.warn(msg)
        return self.max_tanks

    @Attribute
    def sub_volume(self):
        """Volume of each individual sub-tank [m³]."""
        return self.total_volume / self.n_tanks

    @Attribute
    def sub_propellant_mass(self):
        return self.total_propellant_mass / self.n_tanks

    @Attribute
    def _sub_x_starts(self):
        """
        X-start positions of all sub-tanks.
        Computed from the first sub-tank's L/D geometry and the spacing.
        Uses a representative tank to get the single-sub-tank total_length.
        """
        ref = PropellantTank(
            required_volume=self.sub_volume,
            max_outer_diameter=self.max_outer_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.wall_thickness,
            min_cylindrical_length=self.min_cylindrical_length,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            propellant_mass_in_tank=self.sub_propellant_mass,
        )
        sub_len = ref.total_length
        starts = []
        x = self.x_start
        for _ in range(self.n_tanks):
            starts.append(x)
            x += sub_len + self.intertank_spacing
        return starts

    @Attribute
    def stack_length(self):
        """
        Total axial length of this stack [m].
        = n_tanks * sub_tank_length + (n_tanks - 1) * intertank_spacing
        """
        ref = PropellantTank(
            required_volume=self.sub_volume,
            max_outer_diameter=self.max_outer_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.wall_thickness,
            min_cylindrical_length=self.min_cylindrical_length,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            propellant_mass_in_tank=self.sub_propellant_mass,
        )
        sub_len = ref.total_length
        return (self.n_tanks * sub_len
                + (self.n_tanks - 1) * self.intertank_spacing)

    @Attribute
    def x_end(self):
        return self.x_start + self.stack_length

    @Attribute
    def structural_mass(self):
        """Total structural mass of all sub-tanks [m]."""
        ref = PropellantTank(
            required_volume=self.sub_volume,
            max_outer_diameter=self.max_outer_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.wall_thickness,
            min_cylindrical_length=self.min_cylindrical_length,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            propellant_mass_in_tank=self.sub_propellant_mass,
        )
        return ref.structural_mass * self.n_tanks

    # ── Sub-tank Parts (up to max_tanks, suppressed if not used) ─────

    @Part
    def tank_1(self):
        return PropellantTank(
            required_volume=self.sub_volume,
            max_outer_diameter=self.max_outer_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.wall_thickness,
            x_start=self._sub_x_starts[0],
            color=self.color,
            min_cylindrical_length=self.min_cylindrical_length,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            propellant_mass_in_tank=self.sub_propellant_mass,
            popup_warnings=self.popup_warnings,
        )

    @Part
    def tank_2(self):
        return PropellantTank(
            required_volume=self.sub_volume,
            max_outer_diameter=self.max_outer_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.wall_thickness,
            x_start=self._sub_x_starts[1] if self.n_tanks >= 2
                    else self._sub_x_starts[0],
            color=self.color,
            min_cylindrical_length=self.min_cylindrical_length,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            propellant_mass_in_tank=self.sub_propellant_mass,
            popup_warnings=self.popup_warnings,
            suppress=self.n_tanks < 2,
        )

    @Part
    def tank_3(self):
        return PropellantTank(
            required_volume=self.sub_volume,
            max_outer_diameter=self.max_outer_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.wall_thickness,
            x_start=self._sub_x_starts[2] if self.n_tanks >= 3
                    else self._sub_x_starts[0],
            color=self.color,
            min_cylindrical_length=self.min_cylindrical_length,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            propellant_mass_in_tank=self.sub_propellant_mass,
            popup_warnings=self.popup_warnings,
            suppress=self.n_tanks < 3,
        )

    @Part
    def tank_4(self):
        return PropellantTank(
            required_volume=self.sub_volume,
            max_outer_diameter=self.max_outer_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.wall_thickness,
            x_start=self._sub_x_starts[3] if self.n_tanks >= 4
                    else self._sub_x_starts[0],
            color=self.color,
            min_cylindrical_length=self.min_cylindrical_length,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            propellant_mass_in_tank=self.sub_propellant_mass,
            popup_warnings=self.popup_warnings,
            suppress=self.n_tanks < 4,
        )


# ---------------------------------------------------------------------------
# PropulsionSystem
# ---------------------------------------------------------------------------

class PropulsionSystem(Base):
    """
    Horizontal-takeoff spaceplane propulsion sizing.

    Owns an oxidiser TankStack (blue) and a fuel TankStack (green).
    Each stack auto-splits into 1-4 sub-tanks if L/D > max_tank_ld.
    The gap between the two stacks is always exactly intertank_spacing.
    """

    # ── Propulsion inputs ─────────────────────────────────────────────
    propulsion_type: str = Input(
        "N2O_PROPYLENE", validator=OneOf(list(PROPELLANT_DB.keys())))
    payload_mass:       float = Input(4.0,   validator=Positive())
    target_apogee:      float = Input(100e3, validator=Positive())
    max_burnout_mach:   float = Input(1.2,   validator=Between(0.9, 3.5))
    thrust_to_weight:   float = Input(1.0,   validator=Between(0.6, 1.2))

    # ── Tank geometry ─────────────────────────────────────────────────
    max_tank_diameter:       float = Input(0.20,  validator=Positive())
    fuselage_inner_diameter: float = Input(0.20,  validator=Positive())
    tank_wall_thickness:     float = Input(0.003, validator=Positive())
    intertank_spacing:       float = Input(0.050, validator=Positive(incl_zero=True))
    x_tanks_start:           float = Input(0.0)
    min_cylindrical_length:  float = Input(0.020, validator=Positive())
    #: L/D above which a tank is split
    max_tank_ld:             float = Input(5.0,   validator=Positive())
    #: Maximum number of sub-tanks per propellant
    max_tanks_per_propellant: int  = Input(4,     validator=Positive())

    # ── Tank structural inputs ────────────────────────────────────────
    q_max:            float = Input(50e3,  validator=Positive())
    sigma_allow_tank: float = Input(276e6, validator=Positive())
    rho_wall:         float = Input(2700.0, validator=Positive())
    factor_of_safety: float = Input(1.5,   validator=Between(1.0, 3.0))

    popup_warnings: bool = Input(False)

    # ── Physical constants ────────────────────────────────────────────
    _G0         = 9.80665
    _SOUND_30KM = 300.0

    BURNOUT_ALTITUDE    = 30_000.0
    DRAG_LOSS_ASCENT    = 200.0
    GRAVITY_LOSS_ASCENT = 200.0
    DESCENT_DV          = 80.0
    PROPELLANT_MARGIN   = 0.05

    ULLAGE_SELF_PRESS    = 0.02
    ULLAGE_NON_SELFPRESS = 0.05


    TW_LO = 1.3
    TW_HI = 2.5


    @Attribute
    def checked_thrust_to_weight(self):
        if not (self.TW_LO <= self.thrust_to_weight <= self.TW_HI):
            msg = (f"thrust_to_weight ({self.thrust_to_weight:.2f}) outside "
                   f"recommended range [{self.TW_LO}, {self.TW_HI}] "
                   f"for horizontal takeoff spaceplane.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("T/W warning", msg)
        return self.thrust_to_weight

    # ── Propellant properties ─────────────────────────────────────────

    @Attribute
    def _props(self):
        return PROPELLANT_DB[self.propulsion_type]

    @Attribute
    def isp(self):
        return self._props["isp"]

    @Attribute
    def mixture_ratio(self):
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

    # ── Delta-V budget ────────────────────────────────────────────────

    @Attribute
    def burnout_speed(self):
        return self.max_burnout_mach * self._SOUND_30KM

    @Attribute
    def zoom_delta_v(self):
        dh = self.target_apogee - self.BURNOUT_ALTITUDE
        if dh <= 0:
            warnings.warn("target_apogee <= burnout altitude.")
            return self.burnout_speed
        return max(sqrt(2.0 * self._G0 * dh), self.burnout_speed)

    @Attribute
    def ascent_delta_v_ideal(self):
        return self.zoom_delta_v + self.DRAG_LOSS_ASCENT + self.GRAVITY_LOSS_ASCENT

    @Attribute
    def required_delta_v(self):
        return (self.ascent_delta_v_ideal + self.DESCENT_DV) * (1.0 + self.PROPELLANT_MARGIN)

    # ── Mass sizing (two-phase iterative) ─────────────────────────────

    @Attribute
    def mass_ratio(self):
        return exp(self.required_delta_v / (self._G0 * self.isp))

    @Attribute
    def propellant_fraction(self):
        return 1.0 - 1.0 / self.mass_ratio

    #: Science payload [kg]
    mass_payload: float = Input(4.0, validator=Positive())
    #: Fuselage shell (nose + barrel + boat-tail skins) [kg]
    mass_fuselage: float = Input(25.0, validator=Positive(incl_zero=True))
    #: Wings + fins structural mass [kg]  (placeholder: Raymer Eq. 15.25)
    mass_wings: float = Input(3.5, validator=Positive(incl_zero=True))
    #: Landing gear [kg]  (~3 % of MTOW is typical for retractable gear)
    mass_landing_gear: float = Input(2.0, validator=Positive(incl_zero=True))
    #: Avionics + EPS [kg]  (flight computer, IMU, GPS, RF, batteries)
    mass_avionics: float = Input(2.5, validator=Positive())
    #: Tail / empennage [kg]
    mass_tail: float = Input(1.5, validator=Positive(incl_zero=True))
    #: Tank structural mass estimate [kg] — initial seed; corrected by
    #: tank_wall_mass from the actual PropellantTank geometry.
    mass_tanks_est: float = Input(1.0, validator=Positive(incl_zero=True))

    @Attribute
    def dry_mass_without_tanks(self):
        return (
                self.mass_payload
                + self.mass_fuselage
                + self.mass_wings
                + self.mass_landing_gear
                + self.mass_avionics
                + self.mass_tail
        )

    @Attribute
    def mass_ratio(self):
        return exp(self.required_delta_v / (self._G0 * self.isp))

    @Attribute
    def _mass_solution(self):
        """
        Iterative mass solution.

        Returns a dictionary with:
            gross_mass
            dry_mass
            propellant_mass
            fuel_mass
            oxidizer_mass
            fuel_volume
            oxidizer_volume
            tank_mass
        """

        dry_without_tanks = self.dry_mass_without_tanks
        mass_ratio = self.mass_ratio

        dry = dry_without_tanks + self.mass_tanks_est
        gross = dry * mass_ratio

        for _ in range(30):
            propellant = gross - dry

            fuel = propellant / (1.0 + self.mixture_ratio)
            oxidizer = propellant - fuel

            fuel_volume = (fuel / self.fuel_density) / (1.0 - self.ullage_fraction)
            oxidizer_volume = (oxidizer / self.oxidizer_density) / (1.0 - self.ullage_fraction)

            ox_stack = TankStack(
                total_volume=oxidizer_volume,
                total_propellant_mass=oxidizer,
                max_outer_diameter=self.max_tank_diameter,
                fuselage_inner_diameter=self.fuselage_inner_diameter,
                wall_thickness=self.tank_wall_thickness,
                x_start=self.x_tanks_start,
                color="blue",
                intertank_spacing=self.intertank_spacing,
                min_cylindrical_length=self.min_cylindrical_length,
                max_tank_ld=self.max_tank_ld,
                max_tanks=self.max_tanks_per_propellant,
                q_max=self.q_max,
                sigma_allow=self.sigma_allow_tank,
                rho_wall=self.rho_wall,
                factor_of_safety=self.factor_of_safety,
                popup_warnings=False,
            )

            fuel_stack = TankStack(
                total_volume=fuel_volume,
                total_propellant_mass=fuel,
                max_outer_diameter=self.max_tank_diameter,
                fuselage_inner_diameter=self.fuselage_inner_diameter,
                wall_thickness=self.tank_wall_thickness,
                x_start=ox_stack.x_end + self.intertank_spacing,
                color="green",
                intertank_spacing=self.intertank_spacing,
                min_cylindrical_length=self.min_cylindrical_length,
                max_tank_ld=self.max_tank_ld,
                max_tanks=self.max_tanks_per_propellant,
                q_max=self.q_max,
                sigma_allow=self.sigma_allow_tank,
                rho_wall=self.rho_wall,
                factor_of_safety=self.factor_of_safety,
                popup_warnings=False,
            )

            tank_mass = ox_stack.structural_mass + fuel_stack.structural_mass

            dry_new = dry_without_tanks + tank_mass
            gross_new = dry_new * mass_ratio

            if abs(gross_new - gross) / gross < 1e-6:
                return {
                    "gross_mass": gross_new,
                    "dry_mass": dry_new,
                    "propellant_mass": gross_new - dry_new,
                    "fuel_mass": fuel,
                    "oxidizer_mass": oxidizer,
                    "fuel_volume": fuel_volume,
                    "oxidizer_volume": oxidizer_volume,
                    "tank_mass": tank_mass,
                }

            dry = dry_new
            gross = gross_new

        warnings.warn("gross_mass iteration did not converge.")

        propellant = gross - dry
        fuel = propellant / (1.0 + self.mixture_ratio)
        oxidizer = propellant - fuel

        fuel_volume = (fuel / self.fuel_density) / (1.0 - self.ullage_fraction)
        oxidizer_volume = (oxidizer / self.oxidizer_density) / (1.0 - self.ullage_fraction)

        return {
            "gross_mass": gross,
            "dry_mass": dry,
            "propellant_mass": propellant,
            "fuel_mass": fuel,
            "oxidizer_mass": oxidizer,
            "fuel_volume": fuel_volume,
            "oxidizer_volume": oxidizer_volume,
            "tank_mass": dry - dry_without_tanks,
        }

    @Attribute
    def gross_mass(self):
        return self._mass_solution["gross_mass"]

    @Attribute
    def wet_mass(self):
        """
        Wet mass = vehicle mass with propellant loaded.
        Same as gross mass at liftoff.
        """
        return self.gross_mass

    @Attribute
    def dry_mass(self):
        """
        Dry mass = all non-propellant mass.
        Includes payload, structure, avionics, landing gear, tail, and tanks.
        """
        return self._mass_solution["dry_mass"]

    @Attribute
    def propellant_mass(self):
        return self._mass_solution["propellant_mass"]

    @Attribute
    def fuel_mass(self):
        return self._mass_solution["fuel_mass"]

    @Attribute
    def oxidizer_mass(self):
        return self._mass_solution["oxidizer_mass"]

    @Attribute
    def fuel_volume(self):
        return self._mass_solution["fuel_volume"]

    @Attribute
    def oxidizer_volume(self):
        return self._mass_solution["oxidizer_volume"]

    @Attribute
    def tank_wall_mass(self):
        return self._mass_solution["tank_mass"]

    @Attribute
    def total_propellant_volume(self):
        return self.fuel_volume + self.oxidizer_volume

    @Attribute
    def thrust(self):
        return self.checked_thrust_to_weight * self.gross_mass * self._G0

    # ── Tank stack x-positions ────────────────────────────────────────

    @Attribute
    def x_oxidizer_stack_start(self):
        """Oxidiser stack starts at the front of the propulsion bay."""
        return self.x_tanks_start

    @Attribute
    def x_fuel_stack_start(self):
        """
        Fuel stack starts exactly intertank_spacing after the oxidiser stack.
        The spacing between the two stacks (ox aft face → fuel fwd face)
        is always intertank_spacing regardless of how many sub-tanks each
        stack contains.
        """
        return self.oxidizer_stack.x_end + self.intertank_spacing

    @Attribute
    def tank_system_length(self):
        """
        Total propulsion bay length [m]: oxidiser stack + gap + fuel stack.
        Key output → Fuselage.propulsion_bay_length.
        """
        return (self.fuel_stack.x_end - self.x_tanks_start)

    # ── TankStack Parts ───────────────────────────────────────────────

    @Part
    def oxidizer_stack(self):
        """
        Oxidiser tank stack (blue). Forward placement keeps CG near centre.
        At N2O/propylene O/F=7.2 this holds most of the propellant mass
        and typically fits in 1 tank; at lower O/F ratios it may split.
        """
        return TankStack(
            label="oxidizer_stack",
            total_volume=self.oxidizer_volume,
            total_propellant_mass=self.oxidizer_mass,
            max_outer_diameter=self.max_tank_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.tank_wall_thickness,
            x_start=self.x_oxidizer_stack_start,
            color="blue",
            intertank_spacing=self.intertank_spacing,
            min_cylindrical_length=self.min_cylindrical_length,
            max_tank_ld=self.max_tank_ld,
            max_tanks=self.max_tanks_per_propellant,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow_tank,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            popup_warnings=self.popup_warnings,
        )

    @Part
    def fuel_stack(self):
        """
        Fuel tank stack (green). At high O/F ratios fuel volume is small;
        diameter optimiser may reduce diameter or splitting may occur if
        the reduced diameter still gives L/D > max_tank_ld.
        """
        return TankStack(
            label="fuel_stack",
            total_volume=self.fuel_volume,
            total_propellant_mass=self.fuel_mass,
            max_outer_diameter=self.max_tank_diameter,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            wall_thickness=self.tank_wall_thickness,
            x_start=self.x_fuel_stack_start,
            color="green",
            intertank_spacing=self.intertank_spacing,
            min_cylindrical_length=self.min_cylindrical_length,
            max_tank_ld=self.max_tank_ld,
            max_tanks=self.max_tanks_per_propellant,
            q_max=self.q_max,
            sigma_allow=self.sigma_allow_tank,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            popup_warnings=self.popup_warnings,
        )

    # ── Soft checks ───────────────────────────────────────────────────

    @Attribute
    def tank_wall_mass(self):
        return self.oxidizer_stack.structural_mass + self.fuel_stack.structural_mass

    @Attribute
    def checked_payload_fraction(self):
        pf = self.payload_mass / self.gross_mass
        if pf < 0.01:
            msg = (f"Payload fraction {pf:.2%} < 1 % — reduce apogee, "
                   "burnout Mach, or use higher-Isp propellant.")
            warnings.warn(msg)
        elif pf > 0.40:
            msg = f"Payload fraction {pf:.2%} > 40 % — verify structural fraction."
            warnings.warn(msg)
        return pf

    @Attribute
    def checked_propellant_fraction(self):
        pf = self.propellant_fraction
        if pf > 0.65:
            warnings.warn(f"Propellant fraction {pf:.3f} > 0.65 — heavy vehicle.")
        return pf

    # ── Summary ───────────────────────────────────────────────────────

    @Attribute
    def summary(self):
        return {
            "launch_mode":              "horizontal",
            "propulsion_type":          self.propulsion_type,
            "isp_s":                    round(self.isp, 1),
            "mixture_ratio_OF":         round(self.mixture_ratio, 2),
            "required_delta_v_m_s":     round(self.required_delta_v, 1),
            "propellant_fraction":      round(self.checked_propellant_fraction, 3),
            "fuel_volume_L":            round(self.fuel_volume * 1e3, 3),
            "oxidizer_volume_L":        round(self.oxidizer_volume * 1e3, 3),
            "n_oxidizer_tanks":         self.oxidizer_stack.n_tanks,
            "n_fuel_tanks":             self.fuel_stack.n_tanks,
            "ox_stack_length_mm":       round(self.oxidizer_stack.stack_length * 1e3, 1),
            "fu_stack_length_mm":       round(self.fuel_stack.stack_length * 1e3, 1),
            "tank_system_length_mm":    round(self.tank_system_length * 1e3, 1),
            "thrust_N":                 round(self.thrust, 1),
            "thrust_to_weight":         round(self.thrust_to_weight, 2),
            "payload_fraction":         round(self.checked_payload_fraction, 4),

            "wet_mass_kg":          round(self.wet_mass, 1),
            "gross_mass_kg":        round(self.gross_mass, 1),
            "dry_mass_kg":          round(self.dry_mass, 1),
            "propellant_mass_kg":   round(self.propellant_mass, 1),
            "fuel_mass_kg":         round(self.fuel_mass, 1),
            "oxidizer_mass_kg":     round(self.oxidizer_mass, 1),
            "tank_wall_mass_kg":    round(self.tank_wall_mass, 3),
        }


if __name__ == "__main__":
    from parapy.gui import display

    prop = PropulsionSystem(
        label="Propulsion System",
        propulsion_type="N2O_PROPYLENE",
        payload_mass=15,
        target_apogee=100e3,
        max_burnout_mach=3.4,
        thrust_to_weight=1.0,
        max_tank_diameter=0.120,
        fuselage_inner_diameter=0.300,
        tank_wall_thickness=0.003,
        intertank_spacing=0.050,
        x_tanks_start=0.0,
        max_tank_ld=5.0,
        max_tanks_per_propellant=4,
    )

    print("\n=== Propulsion System Summary ===")
    for k, v in prop.summary.items():
        print(f"  {k:<42} {v}")

    display(prop)