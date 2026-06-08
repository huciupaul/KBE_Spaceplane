"""
main.py

Top-level integration module for the suborbital research spaceplane KBE application.

Sizing loop:
    PropulsionSystem.tank_system_length  →  Fuselage.propulsion_bay_length
"""

import warnings

from parapy.core import *
from parapy.core.validate import *
from parapy.geom import *

from fuselage import Fuselage, StandardPayloadBay, AvionicsBay, CUBESAT_STANDARDS
from propulsion_system import PropulsionSystem


class Spaceplane(Base):
    """Root assembly for the suborbital research spaceplane."""

    # ── Payload ───────────────────────────────────────────────────────
    cubesat_standard:  str   = Input("3U",  validator=OneOf(list(CUBESAT_STANDARDS.keys())))
    n_units_stacked:   int   = Input(1,     validator=Positive())
    payload_clearance: float = Input(0.030, validator=Positive(incl_zero=True))

    # ── Avionics box (exact dimensions) ──────────────────────────────
    avionics_box_length: float = Input(0.150, validator=Positive())
    avionics_box_width:  float = Input(0.120, validator=Positive())
    avionics_box_height: float = Input(0.080, validator=Positive())

    # ── Fuselage structure ────────────────────────────────────────────
    structural_wall_depth: float = Input(0.050, validator=Between(0.02, 0.15))
    min_inner_diameter:    float = Input(0.30)
    nose_fineness:         float = Input(1.8)
    tail_fineness:         float = Input(2.5)
    engine_exit_diameter:  float = Input(0.080, validator=Positive())
    n_nose_sects:          int   = Input(8)

    # ── Mission / propulsion ──────────────────────────────────────────
    propulsion_type:        str   = Input("N2O_PROPYLENE")
    payload_mass:           float = Input(4.0,   validator=Positive())
    target_apogee:          float = Input(100e3, validator=Positive())
    max_burnout_mach:       float = Input(3.5,   validator=Between(1.0, 5.0))
    thrust_to_weight:       float = Input(1.5,   validator=Between(1.3, 6.0))
    tank_wall_thickness:    float = Input(0.003, validator=Positive())
    intertank_spacing:      float = Input(0.050, validator=Positive(incl_zero=True))
    tank_diameter_fraction:    float = Input(0.40,  validator=Between(0.10, 0.90))
    tanks_avionics_margin:     float = Input(0.050, validator=Positive(incl_zero=True))
    #: L/D above which a tank is split into multiple sub-tanks
    max_tank_ld:               float = Input(5.0,  validator=Positive())
    #: Maximum sub-tanks per propellant (1-4)
    max_tanks_per_propellant:  int   = Input(4,    validator=Positive())

    # ── Tank structural / q_max inputs ───────────────────────────────────
    q_max:            float = Input(50e3,  validator=Positive())
    sigma_allow_tank: float = Input(345e6, validator=Positive())
    rho_wall:         float = Input(2840.0, validator=Positive())
    factor_of_safety: float = Input(1.5,   validator=Between(1.0, 3.0))
    k_tank:           float = Input(0.10,  validator=Positive())

    popup_warnings: bool = Input(False)

    # ── Derived sizing ────────────────────────────────────────────────

    @Attribute
    def _payload_bay(self):
        return StandardPayloadBay(
            cubesat_standard=self.cubesat_standard,
            n_units_stacked=self.n_units_stacked,
            clearance=self.payload_clearance,
        )

    @Attribute
    def fuselage_inner_diameter(self) -> float:
        return max(self._payload_bay.required_diameter, self.min_inner_diameter)

    @Attribute
    def max_tank_diameter(self) -> float:
        return self.tank_diameter_fraction * self.fuselage_inner_diameter

    @Attribute
    def x_tanks_start(self) -> float:
        """
        Tanks start position computed from inputs only (no circular dependency).
        = nose_length + payload_bay_length + avionics_length + margin
        """
        outer_d = self.fuselage_inner_diameter + 2.0 * self.structural_wall_depth
        nose_l  = self.nose_fineness * outer_d
        pay_l   = self._payload_bay.required_longitudinal
        avi_l   = self.avionics_box_length
        margin  = self.tanks_avionics_margin
        return nose_l + pay_l + avi_l + margin

    # ── Propulsion Part ───────────────────────────────────────────────

    @Part
    def propulsion(self):
        return PropulsionSystem(
            label="Propulsion System",
            propulsion_type=self.propulsion_type,
            payload_mass=self.payload_mass,
            target_apogee=self.target_apogee,
            max_burnout_mach=self.max_burnout_mach,
            thrust_to_weight=self.thrust_to_weight,
            max_tank_diameter=self.max_tank_diameter,
            tank_wall_thickness=self.tank_wall_thickness,
            intertank_spacing=self.intertank_spacing,
            x_tanks_start=self.x_tanks_start,
            fuselage_inner_diameter=self.fuselage_inner_diameter,
            max_tank_ld=self.max_tank_ld,
            max_tanks_per_propellant=self.max_tanks_per_propellant,
            q_max=self.q_max,
            sigma_allow_tank=self.sigma_allow_tank,
            rho_wall=self.rho_wall,
            factor_of_safety=self.factor_of_safety,
            k_tank=self.k_tank,
            popup_warnings=self.popup_warnings,
        )

    # ── Fuselage Part ─────────────────────────────────────────────────

    @Part
    def fuselage(self):
        """
        Fuselage Part. Inputs are passed flat (no sub-object instances)
        so the @Part parser can validate them against Fuselage._inputs.
        Both flat-input and sub-object-input versions of fuselage.py are
        supported: the keys match whichever local version is in use.
        """
        return Fuselage(
            label=f"Fuselage ({self.cubesat_standard})",
            cubesat_standard=self.cubesat_standard,
            n_units_stacked=self.n_units_stacked,
            clearance=self.payload_clearance,
            avionics_box_length=self.avionics_box_length,
            avionics_box_width=self.avionics_box_width,
            avionics_box_height=self.avionics_box_height,
            propulsion_bay_length=(self.propulsion.tank_system_length
                                   + self.tanks_avionics_margin),
            structural_wall_depth=self.structural_wall_depth,
            min_inner_diameter=self.min_inner_diameter,
            nose_fineness=self.nose_fineness,
            tail_fineness=self.tail_fineness,
            engine_exit_diameter=self.engine_exit_diameter,
            n_nose_sects=self.n_nose_sects,
            popup_warnings=self.popup_warnings,
        )

    # ── Summary ───────────────────────────────────────────────────────

    @Attribute
    def summary(self) -> dict:
        f = self.fuselage
        p = self.propulsion
        return {
            "total_length_m":          round(f.total_length, 3),
            "outer_diameter_m":        round(f.outer_diameter, 3),
            "inner_diameter_m":        round(f.inner_diameter, 3),
            "slenderness_ratio":       round(f.slenderness_ratio, 2),
            "nose_cone_length_m":      round(f.nose_length, 3),
            "payload_bay_length_m":    round(f.payload_bay.required_longitudinal, 3),
            "avionics_bay_length_m":   round(f.avionics.total_bay_length, 3),
            "tanks_avionics_margin_m": round(self.tanks_avionics_margin, 3),
            "propulsion_bay_length_m": round(f.propulsion_bay_length, 3),
            "tail_length_m":           round(f.tail_length, 3),
            "cubesat_standard":        f.payload_bay.cubesat_standard,
            "payload_volume_m3":       round(f.payload_bay.required_volume, 5),
            "propulsion_type":         p.propulsion_type,
            "required_delta_v_m_s":    round(p.required_delta_v, 1),
            "gross_mass_kg":           round(p.gross_mass, 1),
            "propellant_mass_kg":      round(p.propellant_mass, 1),
            "thrust_N":                round(p.thrust, 1),
            "tank_system_length_m":    round(p.tank_system_length, 3),
            "max_tank_diameter_m":     round(p.max_tank_diameter, 3),
            "n_oxidizer_tanks":        p.oxidizer_stack.n_tanks,
            "n_fuel_tanks":            p.fuel_stack.n_tanks,
            "tank_wall_mass_kg":       round(p.tank_wall_mass, 3),
        }

    def print_summary(self):
        print("\n" + "=" * 60)
        print("  SPACEPLANE SUMMARY")
        print("=" * 60)
        for k, v in self.summary.items():
            print(f"  {k:<44} {v}")
        print("=" * 60)
        f = self.fuselage
        print("\n  FUSELAGE X-POSITIONS (origin = nose tip)")
        print(f"  {'Nose tip':<38} x = 0.000 m")
        print(f"  {'Nose base':<38} x = {f.x_nose_base:.3f} m")
        print(f"  {'Payload bay start':<38} x = {f.x_payload_bay_start:.3f} m")
        print(f"  {'Avionics bay start':<38} x = {f.x_avionics_start:.3f} m")
        print(f"  {'Propulsion bay start':<38} x = {f.x_propulsion_bay_start:.3f} m")
        print(f"  {'Tail start':<38} x = {f.x_tail_start:.3f} m")
        print(f"  {'Total length':<38} x = {f.total_length:.3f} m")
        print("=" * 60)


if __name__ == "__main__":
    from parapy.gui import display

    vehicle = Spaceplane(
        label="Suborbital Research Spaceplane",

        # Payload
        cubesat_standard="6U",
        n_units_stacked=1,
        payload_clearance=0.030,
        payload_mass=4.0,

        # Avionics
        avionics_box_length=0.150,
        avionics_box_width=0.120,
        avionics_box_height=0.080,

        # Fuselage
        structural_wall_depth=0.050,
        min_inner_diameter=0.30,
        nose_fineness=1.8,
        tail_fineness=2.5,

        # Propulsion
        propulsion_type="N2O_PROPYLENE",
        target_apogee=100e3,
        max_burnout_mach=3.5,
        thrust_to_weight=1.5,
        tank_wall_thickness=0.003,
        intertank_spacing=0.050,
        tank_diameter_fraction=0.40,
        tanks_avionics_margin=0.050,
        max_tank_ld=5.0,
        max_tanks_per_propellant=4,
        engine_exit_diameter=0.080,
        n_nose_sects=8,
        q_max=50e3,
        sigma_allow_tank=345e6,
        rho_wall=2840.0,
        factor_of_safety=1.5,
        k_tank=0.10,
    )

    vehicle.print_summary()
    display(vehicle)