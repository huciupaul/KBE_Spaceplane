"""
main.py

Top-level integration module for the suborbital research spaceplane KBE application.

Combines:
    - Fuselage (nose cone, CubeSat payload bay, avionics bay, propulsion bay)
    - PropulsionSystem (delta-V sizing, tank geometry, thrust)

Design philosophy:
    The Spaceplane class is the single root object displayed in ParaPy GUI.
    It wires the PropulsionSystem output (tank_system_length) back into the
    Fuselage propulsion_bay_length, creating a closed sizing loop:

        CubeSat standard  →  fuselage inner diameter
        Mission delta-V   →  propellant mass → tank volumes → tank lengths
        tank_system_length → propulsion_bay_length → fuselage total length

All geometry is placed in a common coordinate system with origin at the
nose tip (X = 0).

Part of Team 24 KBE Assignment – Spaceplane conceptual design tool.
Authors: Team 24
"""

import warnings

from parapy.core import *
from parapy.core.validate import *
from parapy.geom import *

from fuselage import (
    Fuselage,
    StandardPayloadBay,
    AvionicsBay,
    CUBESAT_STANDARDS,
)
from propulsion_system import PropulsionSystem


# ---------------------------------------------------------------------------
# Spaceplane  – root assembly
# ---------------------------------------------------------------------------

class Spaceplane(Base):
    """
    Root assembly for the suborbital research spaceplane.

    Sizing loop (all driven by ParaPy dependency graph):
        1.  User selects cubesat_standard  →  payload_bay.required_diameter
            →  fuselage.inner_diameter  →  fuselage.outer_diameter
        2.  User sets mission targets (target_apogee, max_burnout_mach)
            →  propulsion.required_delta_v → propulsion.gross_mass
            →  propulsion.oxidizer_volume + fuel_volume
            →  propulsion.tank_system_length
        3.  propulsion.tank_system_length → fuselage.propulsion_bay_length
            →  fuselage.cylindrical_length  →  fuselage.total_length
        4.  fuselage.inner_diameter   →  propulsion.max_tank_diameter
            (tanks must fit inside the fuselage)
        5.  fuselage.x_propulsion_bay_start → propulsion.x_tanks_start
            (tanks are placed in the correct fuselage section)
    """

    # ------------------------------------------------------------------
    # Payload inputs
    # ------------------------------------------------------------------

    #: CubeSat form-factor standard
    cubesat_standard: str = Input(
        "3U",
        validator=OneOf(list(CUBESAT_STANDARDS.keys()))
    )

    #: Number of CubeSat units stacked along the fuselage axis
    n_units_stacked: int = Input(1, validator=Positive())

    #: Structural clearance between CubeSat envelope and bay walls [m]
    payload_clearance: float = Input(0.030, validator=Positive(incl_zero=True))

    # ------------------------------------------------------------------
    # Avionics inputs
    # ------------------------------------------------------------------

    avionics_box_length: float = Input(0.150, validator=Positive())
    avionics_box_width: float = Input(0.120, validator=Positive())
    avionics_box_height: float = Input(0.080, validator=Positive())

    #: Clearance between avionics box and fuselage inner wall [m]
    avionics_wall_clearance: float = Input(0.020, validator=Positive())

    #: Axial separation between payload bay aft face and avionics box [m]
    avionics_payload_gap: float = Input(0.040, validator=Positive())

    #: Wiring harness routing margin aft of avionics box [m]
    avionics_wiring_clearance: float = Input(0.015, validator=Positive(incl_zero=True))

    # ------------------------------------------------------------------
    # Fuselage structural inputs
    # ------------------------------------------------------------------

    structural_wall_depth: float = Input(0.050, validator=Between(0.02, 0.15))
    min_inner_diameter: float = Input(0.30)
    nose_fineness: float = Input(1.8)
    tail_fineness: float = Input(2.5)

    # ------------------------------------------------------------------
    # Mission / propulsion inputs
    # ------------------------------------------------------------------

    propulsion_type: str = Input("N2O_PROPYLENE")
    payload_mass: float = Input(4.0, validator=Positive())
    target_apogee: float = Input(100e3, validator=Positive())
    max_burnout_mach: float = Input(3.5, validator=Between(1.0, 5.0))
    thrust_to_weight: float = Input(1.5, validator=Between(1.3, 6.0))
    launch_mode: str = Input("horizontal", validator=OneOf(["horizontal", "vertical"]))
    tank_wall_thickness: float = Input(0.003, validator=Positive())
    intertank_spacing: float = Input(0.050, validator=Positive(incl_zero=True))

    #: Fraction of inner diameter available to tanks (keeps clearance to pipe runs)
    tank_diameter_fraction: float = Input(0.80, validator=Between(0.5, 0.95))

    popup_warnings: bool = Input(False)

    # ------------------------------------------------------------------
    # Intermediate sizing attributes (coupling payload → structure → propulsion)
    # ------------------------------------------------------------------

    @Attribute
    def _payload_bay(self) -> StandardPayloadBay:
        """
        Standalone StandardPayloadBay instance used ONLY to compute the
        required fuselage inner diameter before the Fuselage Part exists.
        This breaks the circular dependency between fuselage geometry
        and avionics sizing.
        """
        return StandardPayloadBay(
            cubesat_standard=self.cubesat_standard,
            n_units_stacked=self.n_units_stacked,
            clearance=self.payload_clearance,
        )

    @Attribute
    def fuselage_inner_diameter(self) -> float:
        """
        Fuselage inner diameter driven by CubeSat payload diagonal [m].
        Used to size avionics and tanks before the Part tree is evaluated.
        """
        return max(self._payload_bay.required_diameter, self.min_inner_diameter)

    @Attribute
    def max_tank_diameter(self) -> float:
        """
        Maximum tank outer diameter [m].
        = tank_diameter_fraction × fuselage inner diameter
        """
        return self.tank_diameter_fraction * self.fuselage_inner_diameter

    # ------------------------------------------------------------------
    # Propulsion Part  (sized first – its tank_system_length drives fuselage)
    # ------------------------------------------------------------------

    @Part
    def propulsion(self) -> PropulsionSystem:
        """
        Propulsion subsystem: delta-V sizing, mass budget, tank geometry.

        The x_tanks_start and max_tank_diameter are updated once the
        Fuselage geometry is known (see fuselage Part below).
        To avoid a circular dependency they are set to provisional values
        here; the Fuselage Part consumes tank_system_length to set
        propulsion_bay_length.
        """
        return PropulsionSystem(
            label="Propulsion System",
            propulsion_type=self.propulsion_type,
            payload_mass=self.payload_mass,
            target_apogee=self.target_apogee,
            max_burnout_mach=self.max_burnout_mach,
            thrust_to_weight=self.thrust_to_weight,
            launch_mode=self.launch_mode,
            max_tank_diameter=self.max_tank_diameter,
            tank_wall_thickness=self.tank_wall_thickness,
            intertank_spacing=self.intertank_spacing,
            # Tanks start at x=0 provisionally; Fuselage sets the real value
            # via x_tanks_start below (dependency tracked by ParaPy)
            x_tanks_start=self.fuselage.x_propulsion_bay_start,
            popup_warnings=self.popup_warnings,
        )

    # ------------------------------------------------------------------
    # Fuselage Part  (propulsion_bay_length driven by PropulsionSystem)
    # ------------------------------------------------------------------

    @Part
    def fuselage(self) -> Fuselage:
        """
        Parametric fuselage with:
            - NoseConePayloadBay  (empty, optional)
            - StandardPayloadBay  (CubeSat standard, primary)
            - AvionicsBay         (with clearance from payload)
            - PropulsionBay       (length = propulsion.tank_system_length)
        """
        return Fuselage(
            label=f"Fuselage ({self.cubesat_standard})",
            payload_bay=StandardPayloadBay(
                cubesat_standard=self.cubesat_standard,
                n_units_stacked=self.n_units_stacked,
                clearance=self.payload_clearance,
            ),
            avionics=AvionicsBay(
                fuselage_inner_diameter=self.fuselage_inner_diameter,
                avionics_box_length=self.avionics_box_length,
                avionics_box_width=self.avionics_box_width,
                avionics_box_height=self.avionics_box_height,
                avionics_wall_clearance=self.avionics_wall_clearance,
                payload_aft_clearance=self.avionics_payload_gap,
                wiring_clearance=self.avionics_wiring_clearance,
            ),
            # KEY COUPLING: PropulsionSystem.tank_system_length sets the bay length
            propulsion_bay_length=self.propulsion.tank_system_length,
            structural_wall_depth=self.structural_wall_depth,
            min_inner_diameter=self.min_inner_diameter,
            nose_fineness=self.nose_fineness,
            tail_fineness=self.tail_fineness,
            popup_warnings=self.popup_warnings,
        )

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------

    @Attribute
    def summary(self) -> dict:
        f = self.fuselage
        p = self.propulsion
        return {
            # ── Geometry ──────────────────────────────────────────────
            "total_length_m":               round(f.total_length, 3),
            "outer_diameter_m":             round(f.outer_diameter, 3),
            "inner_diameter_m":             round(f.inner_diameter, 3),
            "slenderness_ratio":            round(f.slenderness_ratio, 2),
            # ── Section breakdown ─────────────────────────────────────
            "nose_cone_length_m":           round(f.nose_length, 3),
            "payload_bay_length_m":         round(f.payload_bay.required_longitudinal, 3),
            "avionics_bay_length_m":        round(f.avionics.total_bay_length, 3),
            "avionics_payload_gap_m":       round(f.avionics.payload_aft_clearance, 3),
            "propulsion_bay_length_m":      round(f.propulsion_bay_length, 3),
            "tail_length_m":                round(f.tail_length, 3),
            # ── Payload ───────────────────────────────────────────────
            "cubesat_standard":             f.payload_bay.cubesat_standard,
            "cubesat_dims_mm":              str(
                tuple(int(d * 1000) for d in f.payload_bay.cubesat_dims)
            ),
            "payload_volume_m3":            round(f.payload_bay.required_volume, 5),
            "nose_cone_bay_volume_m3":      round(
                f.nose_cone_payload_bay.usable_volume, 5
            ),
            # ── Propulsion ────────────────────────────────────────────
            "propulsion_type":              p.propulsion_type,
            "gross_mass_kg":                round(p.gross_mass, 1),
            "propellant_mass_kg":           round(p.propellant_mass, 1),
            "thrust_N":                     round(p.thrust, 1),
            "required_delta_v_m_s":         round(p.required_delta_v, 1),
            "tank_system_length_m":         round(p.tank_system_length, 3),
            "max_tank_diameter_m":          round(p.max_tank_diameter, 3),
        }

    def print_summary(self):
        print("\n" + "=" * 60)
        print("  SPACEPLANE SUMMARY")
        print("=" * 60)
        for k, v in self.summary.items():
            print(f"  {k:<44} {v}")
        print("=" * 60)
        print("\n  FUSELAGE SECTION X-POSITIONS (origin = nose tip)")
        print(f"  {'Nose tip':<40} x = 0.000 m")
        f = self.fuselage
        print(f"  {'Nose base (mid section start)':<40} x = {f.x_nose_base:.3f} m")
        print(f"  {'Payload bay start':<40} x = {f.x_payload_bay_start:.3f} m")
        print(f"  {'Avionics bay start (incl. gap)':<40} x = {f.x_avionics_start:.3f} m")
        print(f"  {'Propulsion bay start':<40} x = {f.x_propulsion_bay_start:.3f} m")
        print(f"  {'Tail start':<40} x = {f.x_tail_start:.3f} m")
        print(f"  {'Tail tip (total length)':<40} x = {f.total_length:.3f} m")
        print("=" * 60)


# ---------------------------------------------------------------------------
# Stand-alone entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from parapy.gui import display

    # ── Default configuration: 3U CubeSat, 100 km apogee, N2O/Propylene ──

    vehicle = Spaceplane(
        label="Suborbital Research Spaceplane",

        # Payload
        cubesat_standard="6U",
        n_units_stacked=1,
        payload_clearance=0.030,
        payload_mass=20.0,

        # Avionics
        avionics_box_length=0.150,
        avionics_box_width=0.120,
        avionics_box_height=0.080,
        avionics_wall_clearance=0.020,
        avionics_payload_gap=0.040,
        avionics_wiring_clearance=0.015,

        # Fuselage structure
        structural_wall_depth=0.050,
        min_inner_diameter=0.30,
        nose_fineness=1.8,
        tail_fineness=2.5,

        # Propulsion
        propulsion_type="N2O_PROPYLENE",
        target_apogee=100e3,
        max_burnout_mach=3.5,
        thrust_to_weight=1.5,
        launch_mode="horizontal",
        tank_wall_thickness=0.003,
        intertank_spacing=0.050,
        tank_diameter_fraction=0.60,
    )

    vehicle.print_summary()

    display(vehicle)