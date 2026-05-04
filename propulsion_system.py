#estimate required deltav propellant needed from mission inputs--> target altitude ascent profile, cruise/peak velocity
#landing mode, payload mass

#required proplennat mass and proppelant volume and select propellant type (drop down menu)


from parapy.core import *
from parapy.core.validate import *
from math import sqrt, exp
import warnings

class PropulsionSystem(Base):
    """
    Conceptual propulsion sizing for suborbital research spaceplane .

    This class selects propellant properties and estimates
    1. Thrust
    2. Propellant mass
    3. Oxidizer/fuel split
    4. Corresponding fluid volumes

    Tank geometry handled separately in tanks.py
    """

    propulsion_type = Input(
        "LOX_METHANE",
        validator=OneOf(["LOX_KEROSENE", "HTP_KEROSENE", "LOX_METHANE"])
    )

    gross_mass = Input(2500, validator = Positive()) #mass logic? Payload is not an input
    target_orbit_altitude = Input(185e3, validator=Positive())
    payload_mass = Input(15.0, validator = Positive(incl_zero=True))

    ullage_fraction = Input(0.05, validator = Positive(incl_zero=True))
    propellant_margin = Input(0.05, validator = Positive(incl_zero=True))

    # Constants
    mu = Input(3.986004418e14)
    re = Input(6371e3)

    @Attribute
    def mixture_ratio(self):
        """Oxidizer-to-fuel mass ratio."""
        if self.propulsion_type == "LOX_KEROSENE":
            return 2.56
        elif self.propulsion_type == "HTP_KEROSENE":
            return 7.07
        elif self.propulsion_type == "LOX_METHANE":
            return 3.50
        else:
            raise ValueError(f"Unsupported propulsion_type: {self.propulsion_type}")

    @Attribute
    def oxidizer_density(self):
        """Oxidizer density [kg/m3]."""
        if self.propulsion_type == "LOX_KEROSENE":
            return 1140.0
        elif self.propulsion_type == "HTP_KEROSENE":
            return 1300.0
        elif self.propulsion_type == "LOX_METHANE":
            return 1140.0
        else:
            raise ValueError(f"Unsupported propulsion_type: {self.propulsion_type}")

    @Attribute
    def fuel_density(self):
        """Fuel density [kg/m3]."""
        if self.propulsion_type == "LOX_KEROSENE":
            return 806.0
        elif self.propulsion_type == "HTP_KEROSENE":
            return 806.0
        elif self.propulsion_type == "LOX_METHANE":
            return 440.0
        else:
            raise ValueError(f"Unsupported propulsion_type: {self.propulsion_type}")

    @Attribute
    def isp(self):
        """Vacuum Isp [s]"""
        if self.propulsion_type == "LOX_KEROSENE":
            return 301.0
        elif self.propulsion_type == "HTP_KEROSENE":
            return 276.0
        elif self.propulsion_type == "LOX_METHANE":
            return 325.0

    @Attribute
    def required_delta_v(self):
        """Equivalent delta-V from specific-energy increase."""
        eps_initial = -self.mu / self.re
        eps_target = -self.mu / (2 * (self.re + self.target_orbit_altitude))
        return sqrt(2 * (eps_target - eps_initial))

    @Attribute
    def propellant_fraction(self):
        g0 = 9.81
        mf = 1.0 - 1.0 / exp(self.required_delta_v / (g0 * self.isp))
        return mf * (1.0 + self.propellant_margin)

    @Attribute
    def propellant_mass(self):
        return self.gross_mass * self.propellant_fraction

    @Attribute
    def fuel_mass(self):
        mr = self.mixture_ratio
        return self.propellant_mass / (1.0 + mr)

    @Attribute
    def oxidizer_mass(self):
        return self.propellant_mass - self.fuel_mass

    @Attribute
    def fuel_volume(self):
        return self.fuel_mass / self.fuel_density

    @Attribute
    def oxidizer_volume(self):
        return self.oxidizer_mass / self.oxidizer_density

    @Attribute
    def total_propellant_volume(self):
        return (self.fuel_volume + self.oxidizer_volume) / (1.0 - self.ullage_fraction)

    @Attribute
    def thrust(self):
        """Conceptual thrust using target T/W = 1.3"""
        return self.gross_mass * 9.81 * 1.3

    @Attribute
    def summary(self):
        return {
            "propulsion_type": self.propulsion_type,
            "mixture_ratio": round(self.mixture_ratio, 2),
            "isp_s": round(self.isp, 1),
            "required_delta_v_m_s": round(self.required_delta_v, 1),
            "propellant_mass_kg": round(self.propellant_mass, 1),
            "fuel_mass_kg": round(self.fuel_mass, 1),
            "oxidizer_mass_kg": round(self.oxidizer_mass, 1),
            "fuel_volume_m3": round(self.fuel_volume, 3),
            "oxidizer_volume_m3": round(self.oxidizer_volume, 3),
            "total_propellant_volume_m3": round(self.total_propellant_volume, 3),
            "thrust_N": round(self.thrust, 1),
        }


if __name__ == "__main__":
    prop = PropulsionSystem(
        propulsion_type="LOX_METHANE",
        gross_mass=2500.0,
        propellant_margin=0.05,
        ullage_fraction=0.05,
        target_orbit_altitude=185e3
    )

    print(prop.summary)





