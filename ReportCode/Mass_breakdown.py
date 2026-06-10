import matplotlib.pyplot as plt


def plot_mass_breakdown(vehicle):
    """
    Plot a pie chart of the vehicle mass breakdown.

    Parameters
    ----------
    vehicle : Spaceplane
    """

    p = vehicle.propulsion
    f = vehicle.fuselage

    labels = [
        "Fuselage",
        "Wings",
        "Tail",
        "Avionics",
        "Payload",
        "Tank walls",
        "Propellant",
        "Landing gear",
    ]

    masses = [
        f.fuselage_structural_mass,
        p.wings_mass,
        p.avionics_mass,
        p.payload_mass,
        p.tank_wall_mass,
        p.propellant_mass,
    ]

    plt.figure(figsize=(8, 8))

    plt.pie(
        masses,
        labels=labels,
        autopct="%1.1f%%",
        startangle=90,
    )

    plt.title("Spaceplane Mass Breakdown")

    plt.axis("equal")

    plt.show()