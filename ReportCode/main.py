"""
main.py

Top-level integration module for the suborbital research spaceplane KBE application.

Sizing loop:
    PropulsionSystem.tank_system_length  →  Fuselage.propulsion_bay_length
"""
from math import radians
from mass_breakdown import plot_mass_breakdown
from parapy.core import *
from parapy.core.validate import *
from parapy.geom import (GeomBase, translate, rotate, ProjectedCurve,
                         MirroredShape, Rectangle, SubtractedSolid, Subtracted,
                         Fused, FusedSolid, rotate90,XOY)

from fuselage import Fuselage, StandardPayloadBay, CUBESAT_STANDARDS, FUSELAGE_MATERIALS
from propulsion_system import PropulsionSystem
from wing import Wing
from tail import TailSection
from ref_frame import Frame



class Spaceplane(GeomBase):
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
    min_inner_diameter:    float = Input(0.30)
    nose_fineness:         float = Input(1.8)
    tail_fineness:         float = Input(2.5)
    engine_exit_diameter:  float = Input(0.080, validator=Positive())
    n_nose_sects:          int   = Input(8)
    fuselage_material: str = Input("Al-6061-T6",
                                   validator=OneOf(list(FUSELAGE_MATERIALS.keys())))
    skin_thickness: float = Input(0.003, validator=Positive())

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

    # Individual Masses  ───────────────────────────────────
    mass_landing_gear: float = Input(5.0, validator=Positive())
    mass_avionics: float = Input(2.5, validator=Positive())
    mass_tail: float = Input(8.0, validator=Positive())
    mass_wings: float = Input(20.0, validator=Positive())

    # ── Tank structural / q_max inputs ───────────────────────────────────
    q_max:            float = Input(50e3,  validator=Positive())
    sigma_allow_tank: float = Input(345e6, validator=Positive())
    rho_wall:         float = Input(2840.0, validator=Positive())
    factor_of_safety: float = Input(1.5,   validator=Between(1.0, 3.0))

    # ─────────────────────── Wing ───────────────────────────────────
    airfoil_root_name: str = Input("whitcomb")
    airfoil_kink_name: str = Input("whitcomb")
    airfoil_tip_name: str = Input("whitcomb")
    w_c_root: float = Input()
    w_c_kink: float = Input()
    w_c_tip: float = Input()

    w_t_factor_root: float = Input(1)
    w_t_factor_kink: float = Input(1)
    w_t_factor_tip: float = Input(1)

    w_semi_span: float = Input()
    w_kink_span: float = Input()
    w_sweep_kink: float = Input(0)  # at leading edge, in degrees
    w_sweep_tip: float = Input(0)
    w_twist: float = Input(0)
    w_dihedral: float = Input(0)

    w_pos_rel_x: float = Input(0.4) # wing root LE position as fraction of fuselage length
    w_pos_rel_z: float = Input(0.8) # wing root LE position as fraction of fuselage radius, vertically

    # ─────────────────────Tail sections ───────────────────────────────────

    vt_long: float = Input(0.8) # VT root LE position as fraction of fuselage length
    vt_taper: float = Input(0.4)
    vt_chord_perc: float = Input(0.75)




    mesh_deflection: float = Input(1e-4)
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
        outer_d = self.fuselage_inner_diameter + 2.0 * self.skin_thickness
        nose_l  = self.nose_fineness * outer_d
        pay_l   = self._payload_bay.required_longitudinal
        avi_l   = self.avionics_box_length
        margin  = self.tanks_avionics_margin
        return nose_l + pay_l + avi_l + margin

    preliminary_propulsion_bay_length: float = Input(1.20, validator=Positive())

    @Attribute
    def preliminary_fuselage(self):
        return Fuselage(
            label="Preliminary Fuselage",
            cubesat_standard=self.cubesat_standard,
            n_units_stacked=self.n_units_stacked,
            clearance=self.payload_clearance,
            avionics_box_length=self.avionics_box_length,
            avionics_box_width=self.avionics_box_width,
            avionics_box_height=self.avionics_box_height,
            propulsion_bay_length=self.preliminary_propulsion_bay_length,
            min_inner_diameter=self.min_inner_diameter,
            nose_fineness=self.nose_fineness,
            tail_fineness=self.tail_fineness,
            engine_exit_diameter=self.engine_exit_diameter,
            n_nose_sects=self.n_nose_sects,
            fuselage_material=self.fuselage_material,
            skin_thickness=self.skin_thickness,
            popup_warnings=self.popup_warnings,
        )

    @Attribute
    def preliminary_fuselage_mass(self):
        return self.preliminary_fuselage.fuselage_structural_mass

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
            mass_payload=self.payload_mass,
            mass_fuselage=self.preliminary_fuselage_mass,
            mass_wings=self.mass_wings,
            mass_landing_gear=self.mass_landing_gear,
            mass_avionics=self.mass_avionics,
            mass_tail=self.mass_tail,
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
            min_inner_diameter=self.min_inner_diameter,
            nose_fineness=self.nose_fineness,
            tail_fineness=self.tail_fineness,
            engine_exit_diameter=self.engine_exit_diameter,
            n_nose_sects=self.n_nose_sects,

            fuselage_material=self.fuselage_material,
            skin_thickness=self.skin_thickness,

            popup_warnings=self.popup_warnings,
        )

    # ── Wing Part ─────────────────────────────────────────────────

    @Part
    def aircraft_frame(self):
        """This helps visualise the wing local reference frame"""
        return Frame(pos=self.position)

    @Part
    def wing(self):
        return Wing(pass_down=['airfoil_root_name', 'airfoil_kink_name', 'airfoil_tip_name'],
                    semi_span=self.w_semi_span,
                    kink_span=self.w_kink_span,
                    sweep_kink=self.w_sweep_kink,
                    sweep_tip=self.w_sweep_tip,
                    twist=self.w_twist,
                    dihedral=self.w_dihedral,
                    c_root=self.w_c_root,
                    c_kink=self.w_c_kink,
                    c_tip=self.w_c_tip,
                    t_factor_root=self.w_t_factor_root,
                    t_factor_kink=self.w_t_factor_kink,
                    t_factor_tip=self.w_t_factor_tip,
                    position=translate(self.position,
                                       'x', self.w_pos_rel_x * self.fuselage.total_length,
                                       'z', self.w_pos_rel_z * -self.fuselage.outer_diameter/2),
                    mesh_deflection=self.mesh_deflection)

    # # ── VT and HT parts ───────────────────────────────────────────────────────

    @Part
    def vert_tail(self):
        return TailSection(c_root=self.w_c_root * self.vt_chord_perc,
                           c_tip=self.w_c_root * self.vt_taper,
                           airfoil_root_name="whitcomb",
                           airfoil_tip_name="whitcomb",
                           t_factor_root=0.9 * self.w_t_factor_root,
                           t_factor_tip=0.8 * self.w_t_factor_tip,
                           semi_span=self.w_semi_span / 3,
                           sweep=45,
                           twist=0,
                           position=rotate(translate(self.position,
                                                     "x", self.vt_long * self.fuselage.total_length,
                                                     "z", self.fuselage.outer_diameter/2 * 0.7),
                                           "x", radians(90)),
                           mesh_deflection=self.mesh_deflection)

    @Part
    def h_tail_right(self):
        return TailSection(c_root=self.w_c_root / 1.5,
                           c_tip=self.w_c_tip / 2,
                           airfoil_root_name="whitcomb",
                           airfoil_tip_name="whitcomb",
                           t_factor_root=0.9 * self.w_t_factor_root,
                           t_factor_tip=0.7 * child.spact_factor_root,
                           semi_span=self.w_semi_span / 2.5,
                           sweep=self.w_sweep_tip * 1.2,
                           twist=0,
                           position=rotate(translate
                                           (self.position, "x", self.fuselage.total_length - self.w_c_root),
                                           "x", radians(self.w_dihedral + 4)),
                           mesh_deflection=self.mesh_deflection)

    @Part
    def h_tail_left(self):
        return MirroredShape(shape_in=self.h_tail_right,
                             reference_point=self.position,
                             # Two vectors and a point to define the mirror plane
                             vector1=self.position.Vz,
                             vector2=self.position.Vx,
                             mesh_deflection=self.mesh_deflection)

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
            "thrust_N":                round(p.thrust, 1),
            "tank_system_length_m":    round(p.tank_system_length, 3),
            "max_tank_diameter_m":     round(p.max_tank_diameter, 3),
            "n_oxidizer_tanks":        p.oxidizer_stack.n_tanks,
            "n_fuel_tanks":            p.fuel_stack.n_tanks,
            "wet_mass_kg":             round(p.wet_mass, 1),
            "gross_mass_kg":           round(p.gross_mass, 1),
            "dry_mass_kg":             round(p.dry_mass, 1),
            "propellant_mass_kg":      round(p.propellant_mass, 1),
            "fuel_mass_kg":            round(p.fuel_mass, 1),
            "oxidizer_mass_kg":        round(p.oxidizer_mass, 1),
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
        cubesat_standard="12U",
        n_units_stacked=1,
        payload_clearance=0.050,
        payload_mass=10.0,

        # Avionics
        avionics_box_length=0.150,
        avionics_box_width=0.120,
        avionics_box_height=0.080,

        # Fuselage
        min_inner_diameter=0.30,
        nose_fineness=1.8,
        tail_fineness=2.5,
        fuselage_material="Al-6061-T6",
        skin_thickness=0.002,

        # Propulsion
        propulsion_type="N2O_PROPYLENE",
        target_apogee=100e3,
        max_burnout_mach=3.5,
        thrust_to_weight=1.5,
        tank_wall_thickness=0.003,
        intertank_spacing=0.050,
        tank_diameter_fraction=0.850,
        tanks_avionics_margin=0.050,
        max_tank_ld=5.0,
        max_tanks_per_propellant=4,
        engine_exit_diameter=0.080,
        n_nose_sects=8,
        q_max=50e3,
        sigma_allow_tank=345e6,
        rho_wall=2840.0,
        factor_of_safety=1.5,

        # Wing
        w_semi_span=5,
        w_kink_span=2,
        w_sweep_kink=20,
        w_sweep_tip=25,
        w_twist=0,
        w_dihedral=4,
        w_c_root=1.5,
        w_c_kink=1,
        w_c_tip=0.5,
        w_t_factor_root=1,
        w_t_factor_kink=0.75,
        w_t_factor_tip=0.5,
        airfoil_root_name='whitcomb',
        airfoil_kink_name='whitcomb',
        airfoil_tip_name='whitcomb',
        mesh_deflection=1e-5,
        #position=XOY.rotate(x=0.7).translate(x=5, y=10)
    )

    vehicle.print_summary()
    plot_mass_breakdown(vehicle)
    display(vehicle)