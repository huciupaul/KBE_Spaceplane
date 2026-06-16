"""
main.py

Top-level integration module for the suborbital research spaceplane KBE application.

Sizing loop:
    PropulsionSystem.tank_system_length  →  Fuselage.propulsion_bay_length
"""
from math import radians, sqrt
from mass_breakdown import plot_mass_breakdown
from parapy.core import *
from parapy.core.validate import *
from parapy.geom import (GeomBase, translate, rotate, ProjectedCurve,
                         MirroredShape, Rectangle, SubtractedSolid, Subtracted,
                         Fused, FusedSolid, rotate90,XOY)

from fuselage import Fuselage, StandardPayloadBay, CUBESAT_STANDARDS, FUSELAGE_MATERIALS
from propulsion_system import PropulsionSystem
#from wing import Wing
from tail import TailSection
from ref_frame import Frame
from wing_v2 import Wing
from kbeutils import avl



def calc_span(area, aspect_ratio):
    return sqrt(area * aspect_ratio)

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
    max_burnout_mach:       float = Input(1.0,   validator=Between(0.9, 3.5))
    thrust_to_weight:       float = Input(1.0,   validator=Between(0.6, 1.2))
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
    sigma_allow_tank: float = Input(276e6, validator=Positive())
    rho_wall:         float = Input(2700.0, validator=Positive())
    factor_of_safety: float = Input(1.5,   validator=Between(1.0, 3.0))

    # ─────────────────────── Wing ───────────────────────────────────
    '''airfoil_root_name: str = Input("whitcomb")
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
    w_pos_rel_z: float = Input(0.8) # wing root LE position as fraction of fuselage radius, vertically'''

    wing_location: float = Input()  # longitudinal wing location, as % of fuselage length
    wing_area: float = Input()  # planform area of the total wing (right + left wing)
    wing_aspect_ratio: float = Input()  # square root (wing span**2/ wing area)
    wing_taper_ratio: float = Input()  # chord_tip / chord_root
    wing_le_sweep: float = Input()  # sweep angle measured at leading edge, in degrees
    wing_twist: float = Input()
    wing_airfoil_name: str = Input()  # Name of the NACA airfoil (4 or 5 digits according to designation)
    elevator_hinge: float = Input()  # Chordwise location of the elevator hinge

    # ─────────────────────Tail sections ───────────────────────────────────

    '''vt_long: float = Input(0.8) # VT root LE position as fraction of fuselage length
    vt_taper: float = Input(0.4)
    vt_chord_perc: float = Input(0.75)'''

    tail_area: float = Input()
    tail_aspect_ratio: float = Input()
    tail_taper_ratio: float = Input()
    tail_airfoil_name: str = Input()
    rudder_hinge: float = Input()

    # ─────────────────────AVL inputs ───────────────────────────────────

    cl_cr: float = Input()  # cruise lift coefficient
    mach_cr: float = Input()  # Cruise Mach number (used for AVL analysis)

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

    ''''# ── Wing Part ─────────────────────────────────────────────────

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
                                                     "z", 0),
                                           "x", radians(90)),
                           mesh_deflection=self.mesh_deflection)

    @Part
    def h_tail_right(self):
        return TailSection(c_root=self.w_c_root / 1.5,
                           c_tip=self.w_c_tip / 2,
                           airfoil_root_name="whitcomb",
                           airfoil_tip_name="whitcomb",
                           t_factor_root=0.9 * self.w_t_factor_root,
                           t_factor_tip=0.7 * self.w_t_factor_root,
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
                             mesh_deflection=self.mesh_deflection)'''

    # ────────────────────────────── Wing + vt ───────────────────────────────────

    @Attribute
    def tail_le_sweep(self):
        return self.wing_le_sweep + 5  # engineering rule: to allow the tail a higher critical mach than for the wing

    @Attribute
    def wing_position(self):  # wing reference system. Same orientation as fuselage's, but with origin on wing LE @ root
        return self.position.translate('x', self.fuselage.total_length * self.wing_location,
                                       'z', -self.fuselage.outer_diameter/4)

    @Part
    def wing(self):
        return Wing(name="wing",
                    span=calc_span(self.wing_area, self.wing_aspect_ratio),  # call to function calc_span
                    # span=sqrt(self.wing_area * self.wing_aspect_ratio), # worse alternative to the use of the function
                    aspect_ratio=self.wing_aspect_ratio,
                    taper_ratio=self.wing_taper_ratio,
                    le_sweep=self.wing_le_sweep,
                    twist=self.wing_twist,
                    airfoil_name=self.wing_airfoil_name,
                    control_name='elevator',
                    control_hinge_loc=self.elevator_hinge,
                    duplicate_sign=1,
                    position=self.wing_position)

    @Attribute
    def tail_position(self):
        return rotate(translate(self.position,
                                "x", self.fuselage.total_length - self.tail.chord_root,
                                "z", 0),
                                "x", radians(90))

    @Part
    def tail(self):
        return Wing(name="tail",
                    span=calc_span(self.tail_area, self.tail_aspect_ratio),
                    aspect_ratio=self.tail_aspect_ratio,
                    taper_ratio=self.tail_taper_ratio,
                    le_sweep=self.tail_le_sweep,
                    twist=0,
                    airfoil_name=self.tail_airfoil_name,
                    control_name='rudder',
                    control_hinge_loc=self.rudder_hinge,
                    is_mirrored=False,
                    position=self.tail_position
                    )

    # ── AVL ───────────────────────────────────────────────────────

    @Attribute(in_tree=True)
    def avl_surfaces(self):  # a list of all AVL surfaces in the aircraft:
        return self.find_children(lambda o: isinstance(o, avl.Surface))
        # This automatically scans the product tree and collects all
        # instances of the avl.Surface class.
        # (if you don't know what `lambda` is doing there: that's a somewhat
        # more advanced feature of functional programming, but it's not
        # required knowledge for this course. Just use it as provided above,
        # and you'll be fine. Feel free to check out the
        # Python documentation about it, though, if you're curious.)

        # Otherwise, you can of course also manually specify the surfaces you
        # want to include, like this:
        # return [self.wing.avl_surface, self.tail.avl_surface]
        # (but make sure you forget no surface that needs to be included in the
        # model!)

    @Part
    def avl_configuration(self):
        """Configurations are made separately for each Mach number that is provided."""
        return avl.Configuration(name='cruise analysis',
                                 reference_area=self.wing.planform_area,
                                 reference_span=self.wing.span,
                                 reference_chord=self.wing.mac,
                                 reference_point=self.position.point,
                                 surfaces=self.avl_surfaces,
                                 mach=self.mach_cr)

    @Attribute
    def avl_settings(self):
        """
        Format for AVL inputs:
            dict(<parameter to define>: <value>)
            value can be defined either by a number or with `avl.Parameter`:
            avl.Parameter(name=<var to adjust>,
                          setting=<var for which to achieve a given value>
                          value=<value to achieve>)
        These need to be defined either in Input or in a separate Attribute, in
        order to allow ParaPy to trace dependencies (defining this dictionary
        in an argument for avl.Interface() or avl.Case fails for that reason)
        """
        return {'alpha': avl.Parameter(name='alpha',
                                       setting='CL',
                                       value=self.cl_cr)}

    @Part
    def avl_case(self):
        """avl case definition using the avl_settings dictionary defined above"""
        return avl.Case(name='fixed_cl',  # name _must_ correspond to type of case
                        settings=self.avl_settings)

    @Part
    def avl_analysis(self):
        return avl.Interface(configuration=self.avl_configuration,
                             # note: AVL always expects a list of cases!
                             cases=[self.avl_case])

    @Attribute
    def l_over_d(self):
        """process AVL results and compute L/D"""
        # Since AVL always receives a list of cases, but produces a dictionary of
        # results (using the name supplied to avl.Case as key)
        # each result is itself a nested dictionary, containing a lot of
        # information so it's a good idea to extract the relevant numbers
        # as @Attributes
        return {result['Name']: result['Totals']['CLtot'] / result['Totals']['CDtot']
                for case_name, result in self.avl_analysis.results.items()}
        # The above is a bit more complicated than needed since there's only
        # a single case name etc., but addressing it "by name" means we'd need
        # to update the definition above every time we change something in the
        # analysis.

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
        max_burnout_mach=3.4,
        thrust_to_weight=0.9,
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
        # w_semi_span=5,
        # w_kink_span=2,
        # w_sweep_kink=20,
        # w_sweep_tip=25,
        # w_twist=0,
        # w_dihedral=4,
        # w_c_root=1.5,
        # w_c_kink=1,
        # w_c_tip=0.5,
        # w_t_factor_root=1,
        # w_t_factor_kink=0.75,
        # w_t_factor_tip=0.5,
        # airfoil_root_name='whitcomb',
        # airfoil_kink_name='whitcomb',
        # airfoil_tip_name='whitcomb',
        # mesh_deflection=1e-5,
        #position=XOY.rotate(x=0.7).translate(x=5, y=10)

        # Wing
        wing_location=0.5,
        wing_area=10,
        wing_aspect_ratio=2.5,
        wing_taper_ratio=0.2,
        wing_le_sweep=46,
        wing_twist=-5,
        wing_airfoil_name='23008',
        elevator_hinge=0.8,
        # Tail
        tail_area=2,
        tail_aspect_ratio=2,
        tail_taper_ratio=0.2,
        tail_airfoil_name='0010',
        rudder_hinge=0.6,
        # AVL
        mach_cr=0.3,
        cl_cr=0.4
    )

    vehicle.print_summary()
    plot_mass_breakdown(vehicle)
    display(vehicle)