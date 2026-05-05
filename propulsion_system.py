from parapy.core import *
from parapy.core.validate import *
from math import sqrt, exp
import warnings

def generate_warning(header: str, msg: str):
    """Show a modal warning dialog and wait for the user to dismiss it."""
    from tkinter import Tk, messagebox
    window = Tk()
    window.withdraw()
    messagebox.showwarning(header, msg)
    window.destroy()

PROPELLANT_DB = {
    "LOX_KEROSENE": dict(isp=311.0, mixture_ratio=2.56,
                             oxidizer_density=1140.0, fuel_density=806.0),
    "LOX_METHANE": dict(isp=363.0, mixture_ratio=3.60,
                            oxidizer_density=1140.0, fuel_density=424.0),
    "HTP_KEROSENE": dict(isp=319.0, mixture_ratio=7.30,
                             oxidizer_density=1440.0, fuel_density=806.0),
}

class PropulsionSystem(Base):
    """
    Conceptual propulsion sizing for suborbital research spaceplane .
    The user provides propellant combination, payload mass, target apogee, thrust to weight ratio
    This class selects propellant properties and estimates
    1. Thrust
    2. Propellant mass
    3. Oxidizer/fuel split
    4. Corresponding fluid volumes

    Tank geometry handled separately in tanks.py
    """
    # User Inputs
    # Propellant combination
    propulsion_type: str = Input(
        "LOX_METHANE",
        validator=OneOf(list(PROPELLANT_DB.keys()))
    )

    # Payload mass [kg]
    payload_mass: float = Input(15.0, validator=Positive())

    # Target apogee altitude [m]
    target_apogee = Input(100e3, validator = Positive())

    # Target liftoff thrust to weight ratio [-]
    # Below 1.3 implies a slow ascent with high gravity losses
    # Above 2.0 implies high structural loads and a heavier engine
    thrust_to_weight: float = Input(1.5, validator=Between(1.2, 2.5))

    # Show Tk pop-up critical soft-rule warnings
    popup_warnings: bool = Input(False)

    # Constants
    MU = Input(3.986004418e14)
    RE = Input(6371e3)
    G0= Input(9.81)

    # Embedded knowledge

    BURNOUT_ALTITUDE = 30000 #[m]
    DRAG_LOSS = 200 #[m/s]
    GRAVITY_LOSS = 200.0 #[m/s]
    STRUCTURAL_FRACTION = 0.15
    PROPELLANT_MARGIN = 0.05 #reserve
    ULLAGE_FRACTION = 0.05

    @Attribute
    def _props(self):
        """Raw propellant property dict looked up from PROPELLANT_DB."""
        return PROPELLANT_DB[self.propulsion_type]

    @Attribute
    def isp(self):
        """Vacuum specific impulse [s].  Source: Sutton & Biblarz Table 5-5."""
        return self._props["isp"]

    @Attribute
    def mixture_ratio(self):
        """Oxidiser-to-fuel mass ratio (O/F) [-]."""
        return self._props["mixture_ratio"]

    @Attribute
    def oxidizer_density(self):
        """Oxidiser bulk density [kg/m³]."""
        return self._props["oxidizer_density"]

    @Attribute
    def fuel_density(self):
        """Fuel bulk density [kg/m³]."""
        return self._props["fuel_density"]

    # Delta V budget
    @Attribute
    def zoom_delta_v(self):
        """
        Vertical speed needed to coast from burnout altitude to apogee
        Derived from energy conservation, so for a purely vertical trajectory.
        Actual value is slightly higher due to the launch angle.
        """
        dh = self.target_apogee - self.BURNOUT_ALTITUDE
        if dh <= 0:
            warnings.warn(
                f"target_apogee ({self.target_apogee:.0f} m) is at or below "
                f"burnout altitude ({self.BURNOUT_ALTITUDE:.0f} m). "
                "zoom_delta_v set to zero."
            )
            return 0.0
        return sqrt(2.0 * self.G0 * dh)

    @Attribute
    def required_delta_v(self):
        """
        Total delta-V required for the mission [m/s]
        """
        dv_ideal = self.zoom_delta_v + self.DRAG_LOSS + self.GRAVITY_LOSS
        return dv_ideal * (1.0 + self.PROPELLANT_MARGIN)

    @Attribute
    def mass_ratio(self):
        """
        Tsiolkovsky mass ratio: m_initial / m_final = exp(dV / (g0 * Isp)).
        """
        return exp(self.required_delta_v / (self.G0 * self.isp))

    @Attribute
    def propellant_fraction(self):
        """
        Propellant mass fraction of gross mass [-]
        From the rocket equation: mf = 1 - 1/mass_ratio
        """
        return 1.0 - 1.0 / self.mass_ratio

    @Attribute
    def gross_mass(self):
        """
        Gross lift-off mass [kg]
        gross_mass = payload_mass + propellant_mass + structural_mass
        propellant_mass  = propellant_fraction * gross_mass
        structural_mass  = STRUCTURAL_FRACTION * gross_mass

        Solving for gross_mass:
            gross_mass * (1 - propellant_fraction - STRUCTURAL_FRACTION)= payload_mass
            gross_mass = payload_mass
                / (1 - propellant_fraction - _STRUCTURAL_FRACTION)

            Hard check: denominator must be positive (i.e. payload fraction > 0).
            If not, the vehicle is not feasible at this Isp / apogee combination.
            """
        denominator = 1.0 - self.propellant_fraction - self.STRUCTURAL_FRACTION
        if denominator <= 0:
            raise ValueError(
                f"Vehicle is not feasible: propellant fraction "
                f"({self.propellant_fraction:.3f}) + structural fraction "
                f"({self.STRUCTURAL_FRACTION:.3f}) >= 1.0. "
                "Reduce target_apogee, switch to a higher-Isp propellant, "
                "or reduce structural fraction."
            )
        return self.payload_mass / denominator

    @Attribute
    def structural_mass(self):
        """Structural dry mass [kg] (skin, frames, landing gear, avionics)."""
        return self.STRUCTURAL_FRACTION * self.gross_mass

    @Attribute
    def propellant_mass(self):
        """Total propellant mass including reserves [kg]."""
        return self.propellant_fraction * self.gross_mass

    # Propellant split
    @Attribute
    def fuel_mass(self):
        """Fuel mass [kg].  From O/F ratio: m_fuel = m_prop / (1 + O/F)."""
        return self.propellant_mass / (1.0 + self.mixture_ratio)

    @Attribute
    def oxidizer_mass(self):
        """Oxidiser mass [kg]."""
        return self.propellant_mass - self.fuel_mass

    @Attribute
    def fuel_volume(self):
        """Fuel tank volume including ullage [m³]."""
        return (self.fuel_mass / self.fuel_density) / (1.0 - self.ULLAGE_FRACTION)

    @Attribute
    def oxidizer_volume(self):
        """Oxidiser tank volume including ullage [m³]."""
        return (self.oxidizer_mass / self.oxidizer_density) / (1.0 - self.ULLAGE_FRACTION)

    @Attribute
    def thrust(self):
        """
        Required liftoff thrust [N].
        T = T/W * gross_mass * g0.
        T/W is a user input; typical range 1.2–2.5 for suborbital vehicles.
        """
        return self.thrust_to_weight * self.gross_mass * self.G0

 # Soft cross parameter checks

    @Attribute
    def checked_payload_fraction(self):
        """
        Payload fraction = payload_mass / gross_mass.
        Typical suborbital research vehicles: 2–8 %.
        Below 2 % the vehicle is very inefficient for the mission.
        """
        pf = self.payload_mass / self.gross_mass
        if pf < 0.02:
            msg = (f"Payload fraction {pf:.1%} < 2 % — vehicle is very large "
                   "relative to payload. Consider reducing target_apogee or "
                   "switching to a higher-Isp propellant.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("Low payload fraction", msg)
        elif pf > 0.12:
            msg = (f"Payload fraction {pf:.1%} > 12 % — unusually high; "
                   "verify structural fraction assumption is realistic.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("High payload fraction", msg)
        return pf

    @Attribute
    def checked_propellant_fraction(self):
        """
        Warns when propellant fraction is very high (> 0.70), meaning the
        vehicle is propellant-dominated and structurally very challenging.
        """
        pf = self.propellant_fraction
        if pf > 0.70:
            msg = (f"Propellant fraction {pf:.2f} > 0.70 — structurally "
                   "challenging. Consider a higher-Isp propellant or "
                   "reducing target_apogee.")
            warnings.warn(msg)
            if self.popup_warnings:
                generate_warning("High propellant fraction", msg)
        return pf

    @Attribute
    def summary(self):
        return {
            "propulsion_type": self.propulsion_type,
            "isp_s": round(self.isp, 1),
            "mixture_ratio": round(self.mixture_ratio, 2),
            "required_delta_v_m_s": round(self.required_delta_v, 1),
            "zoom_delta_v_m_s": round(self.zoom_delta_v, 1),
            "mass_ratio": round(self.mass_ratio, 3),
            "propellant_fraction": round(self.propellant_fraction, 3),
            "gross_mass_kg": round(self.gross_mass, 1),
            "structural_mass_kg": round(self.structural_mass, 1),
            "propellant_mass_kg": round(self.propellant_mass, 1),
            "fuel_mass_kg": round(self.fuel_mass, 1),
            "oxidizer_mass_kg": round(self.oxidizer_mass, 1),
            "fuel_volume_m3": round(self.fuel_volume, 3),
            "oxidizer_volume_m3": round(self.oxidizer_volume, 3),
            "thrust_N": round(self.thrust, 1),
            "thrust_to_weight": round(self.thrust_to_weight, 2),
            "payload_fraction": round(self.checked_payload_fraction, 4),
        }


if __name__ == "__main__":
    from parapy.gui import display

    prop = PropulsionSystem(
        label="Propulsion System",
        propulsion_type="LOX_METHANE",
        payload_mass=60.0,
        target_apogee=100e3,
        thrust_to_weight=1.5,
        popup_warnings=False,
    )

    print("\n=== Propulsion System Summary ===")
    for key, val in prop.summary.items():
        print(f"  {key:<32} {val}")
