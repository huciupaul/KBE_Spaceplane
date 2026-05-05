# This is the main file for running the KBE App
"""
Main file to test the conceptual spaceplane KBE modules.

Integrates:
- payload bay
- fuselage
- propulsion system
- tank system


"""

from parapy.core import *
from parapy.geom import *
from parapy.gui import display

from fuselage import Fuselage, PayloadBay
from propulsion_system import PropulsionSystem
from tanks import TankSystem


class Spaceplane(Base):
    """
    Top-level integration class for the conceptual suborbital spaceplane.
    """

    # Inputs
    payload_longitudinal = Input(1.20)
    payload_lateral = Input(0.80)
    payload_vertical = Input(0.80)
    payload_clearance = Input(0.10)

    target_apogee = Input(100e3)
    payload_mass = Input(60.0)
    propulsion_type = Input("LOX_METHANE")
    thrust_to_weight = Input(1.5)

    fuselage_wall_depth = Input(0.05)
    fuselage_min_inner_diameter = Input(0.40)
    fuselage_nose_fineness = Input(1.8)
    fuselage_tail_fineness = Input(3.0)
    fuselage_upsweep_angle = Input(7.0)

    tank_wall_thickness = Input(0.01)
    intertank_spacing = Input(0.15)
    tank_diameter_fraction = Input(0.9)


    # Payload
    @Part
    def payload_bay(self):
        return PayloadBay(
            payload_longitudinal=self.payload_longitudinal,
            payload_lateral=self.payload_lateral,
            payload_vertical=self.payload_vertical,
            clearance=self.payload_clearance
        )


    # Propulsion
    @Part
    def propulsion_system(self):
        return PropulsionSystem(
            propulsion_type=self.propulsion_type,
            payload_mass=self.payload_mass,
            target_apogee=self.target_apogee,
            thrust_to_weight=self.thrust_to_weight
        )


    # Fuselage
    @Part
    def fuselage(self):
        return Fuselage(
            payload_bay=self.payload_bay,
            structural_wall_depth=self.fuselage_wall_depth,
            min_inner_diameter=self.fuselage_min_inner_diameter,
            nose_fineness=self.fuselage_nose_fineness,
            tail_fineness=self.fuselage_tail_fineness,
            upsweep_angle=self.fuselage_upsweep_angle,
            popup_warnings=False
        )


    # Tank system
    @Attribute
    def max_tank_diameter(self):
        return self.tank_diameter_fraction * self.fuselage.inner_diameter

    @Attribute
    def tank_x_start(self):
        """
        Place tanks in the aft fuselage, starting after the payload bay.
        """
        return self.fuselage.x_propulsion_bay_start

    @Part
    def tank_system(self):
        return TankSystem(
            propulsion_system=self.propulsion_system,
            max_tank_diameter=self.max_tank_diameter,
            wall_thickness=self.tank_wall_thickness,
            intertank_spacing=self.intertank_spacing,
            x_start=self.tank_x_start
        )


    # Summary
    @Attribute
    def summary(self):
        return {
            "fuselage_total_length_m": round(self.fuselage.total_length, 3),
            "fuselage_inner_diameter_m": round(self.fuselage.inner_diameter, 3),
            "fuselage_outer_diameter_m": round(self.fuselage.outer_diameter, 3),
            "payload_required_volume_m3": round(self.payload_bay.required_volume, 3),
            "propulsion_type": self.propulsion_system.propulsion_type,
            "required_delta_v_m_s": round(self.propulsion_system.required_delta_v, 1),
            "propellant_mass_kg": round(self.propulsion_system.propellant_mass, 1),
            "fuel_volume_m3": round(self.propulsion_system.fuel_volume, 3),
            "oxidizer_volume_m3": round(self.propulsion_system.oxidizer_volume, 3),
            "tank_system_length_m": round(self.tank_system.total_system_length, 3),
        }


if __name__ == "__main__":
    sp = Spaceplane(
        label="Conceptual Suborbital Spaceplane",
        payload_longitudinal=1.20,
        payload_lateral=0.80,
        payload_vertical=0.80,
        payload_clearance=0.10,
        target_apogee=1000e3,
        payload_mass=60.0,
        propulsion_type="LOX_METHANE",
        thrust_to_weight=1.5,
        fuselage_wall_depth=0.05,
        fuselage_min_inner_diameter=0.40,
        fuselage_nose_fineness=1.8,
        fuselage_tail_fineness=3.0,
        fuselage_upsweep_angle=7.0,
        tank_wall_thickness=0.01,
        intertank_spacing=0.15,
        tank_diameter_fraction=0.9
    )

    print("\n=== Spaceplane Summary ===")
    for key, val in sp.summary.items():
        print(f"  {key:<30} {val}")

    display(sp)