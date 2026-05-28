"""
tanks.py

Propellant tank geometry for the suborbital research spaceplane

PropellantTank: sizes one cylindrical tak given required volume
    and max diameter
TankSystem: places oxidiser + fuel tanks sequential in the propulsion bay

Part of Team 24 KBE Assignment
Authors: Yasmine Mafoutsis, Paul-Ionut Huciu
"""

from math import pi
import warnings
from parapy.core import *
from parapy.core.validate import *
from parapy.geom import *

from propulsion_system import PropulsionSystem, generate_warning




class PropellantTank(Base):
    """
    Sizes one propellant tank as a pure cylinder.

    Inputs:
    - required_volume: required tank volume [m^3]
    - max_outer_diameter: maximum allowable outer diameter [m]
    - wall_thickness: tank wall thickness [m]
    """

    required_volume = Input(0.5, validator=Positive())
    max_outer_diameter = Input(0.8, validator=Positive())
    wall_thickness = Input(0.003, validator=Positive())
    x_start = Input(0.0)
    color = Input("orange")
    popup_warnings: bool = Input(False)

    @Attribute
    def outer_diameter(self):
        return self.max_outer_diameter

    @Attribute
    def inner_diameter(self):
        value = self.outer_diameter - 2.0 * self.wall_thickness
        if value <= 0:
            raise ValueError(
                "Tank wall thickness is too large for the selected diameter."
            )
        return value

    @Attribute
    def inner_radius(self):
        return 0.5 * self.inner_diameter

    @Attribute
    def outer_radius(self):
        return 0.5 * self.outer_diameter

    @Attribute
    def cylindrical_length(self):
        return self.required_volume / (pi * self.inner_radius ** 2)

    @Attribute
    def total_length(self):
        return self.cylindrical_length

    @Attribute
    def x_center(self):
        return self.x_start + 0.5 * self.total_length

    @Attribute
    def cylinder_position(self):
        return Position(Point(self.x_start, 0, 0))

    @Part
    def cylinder(self):
        return Cylinder(
            radius=self.outer_radius,
            height=self.cylindrical_length,
            position=rotate(self.cylinder_position, "y", 90, deg=True),
            color=self.color,
            transparency=0.3
        )

    @Attribute
    def summary(self):
        return {
            "required_volume_m3": round(self.required_volume, 3),
            "outer_diameter_m": round(self.outer_diameter, 3),
            "inner_diameter_m": round(self.inner_diameter, 3),
            "cylindrical_length_m": round(self.cylindrical_length, 3),
            "total_length_m": round(self.total_length, 3),
            "x_start_m": round(self.x_start, 3),
            "x_center_m": round(self.x_center, 3),
        }


class TankSystem(Base):
    """
    Tank system for the spaceplane propulsion subsystem.

    Uses the PropulsionSystem outputs:
    - fuel_volume
    - oxidizer_volume

    Sizes:
    - one oxidizer tank: palaced forward closer to cg
    - one fuel tank: placed behind oxidiser tank
        * sequentially along the x-axis. *
    """

    propulsion_system = Input(PropulsionSystem())

    max_tank_diameter = Input(0.8, validator=Positive())
    wall_thickness = Input(0.01, validator=Positive())
    intertank_spacing = Input(0.10, validator=Positive(incl_zero=True))
    x_start = Input(0.0)

    @Attribute
    def oxidizer_tank_x_start(self):
        return self.x_start

    @Attribute
    def fuel_tank_x_start(self):
        return self.oxidizer_tank_x_start + self.oxidizer_tank.total_length + self.intertank_spacing

    @Attribute
    def total_system_length(self):
        return self.fuel_tank_x_start + self.fuel_tank.total_length - self.x_start

    @Part
    def oxidizer_tank(self):
        return PropellantTank(
            required_volume=self.propulsion_system.oxidizer_volume,
            max_outer_diameter=self.max_tank_diameter,
            wall_thickness=self.wall_thickness,
            x_start=self.oxidizer_tank_x_start,
            color="blue"
        )

    @Part
    def fuel_tank(self):
        return PropellantTank(
            required_volume=self.propulsion_system.fuel_volume,
            max_outer_diameter=self.max_tank_diameter,
            wall_thickness=self.wall_thickness,
            x_start=self.fuel_tank_x_start,
            color="green"
        )

    @Attribute
    def summary(self):
        return {
            "oxidizer_volume_m3": round(self.propulsion_system.oxidizer_volume, 3),
            "fuel_volume_m3": round(self.propulsion_system.fuel_volume, 3),
            "max_tank_diameter_m": round(self.max_tank_diameter, 3),
            "tank_system_length_m": round(self.total_system_length, 3),
            "oxidizer_tank": self.oxidizer_tank.summary,
            "fuel_tank": self.fuel_tank.summary,
        }


if __name__ == "__main__":
    from parapy.gui import display

    prop = PropulsionSystem(
        label="Propulsion System",
        propulsion_type="N2O_PROPYLENE",
        payload_mass=60.0,
        target_apogee=100e3,
        thrust_to_weight=3.5,
        popup_warnings=False,
    )

    tanks = TankSystem(
        propulsion_system=prop,
        max_tank_diameter=0.7,
        wall_thickness=0.01,
        intertank_spacing=0.15,
        x_start=0.0,
    )

    print("\n=== Tank System Summary ===")
    for key, val in tanks.summary.items():
        print(f"{key}: {val}")

    display(tanks)