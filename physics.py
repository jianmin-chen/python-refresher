# Constants
g = 9.81
density_water = 1000


def calculate_buoyancy(V: float, density_fluid: float):
    """
    Calculates the buoyancy force exerted on a object submerged in water.

        Parameters:
            V (float): The volume of the object in cubic meters.
            density_fluid (float): The density of the fluid in kg/m^3.
        Returns:
            buoyancy_force (float): Buoyancy float in Newtons
    """
    if V < 0 or density_fluid <= 0:
        raise ValueError("Volume and/or density of fluid should be > 0")

    return density_fluid * V * g


def will_it_float(V: float, mass: float):
    """
    Determines whether an object will float or sink in water.

        Parameters:
            V (float): The volume of the object in cubic meters.
            mass (float): The mass of the object in kilograms.
        Returns:
            will_it_float (bool): True if the object will float and False if the object will sink
    """

    if V <= 0 or mass <= 0:
        raise ValueError("Volume and/or mass should be > 0")

    # mass / V = density; if density < density_water, it will float, otherwise will sink
    if mass / V < density_water:
        return True
    return False


def calculate_pressure(depth: float):
    """
    Calculates the pressure at a given depth in water.

        Parameters:
            depth (float): The depth in meters.
        Returns:
            pressure (float): Pressure in Pascals.
    """

    if depth <= 0:
        raise ValueError("Depth should be > 0 meters")

    return density_water * g * depth


if __name__ == "__main__":
    print(calculate_buoyancy(60, density_water))
    print(will_it_float(8, 10000))
