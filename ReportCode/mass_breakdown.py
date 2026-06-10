import matplotlib.pyplot as plt


def plot_mass_breakdown(vehicle):
    """
    Plot a pie chart of the vehicle mass breakdown.
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
        p.mass_wings,
        p.mass_tail,
        p.mass_avionics,
        p.mass_payload,
        p.tank_wall_mass,
        p.propellant_mass,
        p.mass_landing_gear,
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