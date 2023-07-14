from numpy import sin, cos, pi

# Constants
g = 9.81
density_water = 1000
surface_pressure = 101325


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

    buoyancy_force = density_fluid * V * g
    return buoyancy_force


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
    density = mass / V
    if density == density_water:
        return None
    elif density < density_water:
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

    pressure = density_water * g * depth + surface_pressure
    return pressure


def calculate_acceleration(F: float, m: float) -> float:
    """
    Calculates the acceleration of an object given the force applied to it and its mass

        Parameters:
            F (float): The force applied to the object in Newtons
            m (float): The mass of the object in kg.

        Returns:
            a (float): The acceleration of the object.
    """

    if m <= 0:
        raise ValueError("Mass should be > 0 kilograms")
    if F < 0:
        raise ValueError("Force should be >= 0 Newtons")

    a = F / m
    return a


def calculate_angular_acceleration(tau: float, I: float) -> float:
    """
    Calculates the angular acceleration of an object given the torque applied to it and its moment of inertia.

        Parameters:
            tau (float): The torque applied to the object in Newton-meters
            I (float): the moment of inertia of the object in kg * m^2.

        Returns:
            angular_acceleration (float)
    """

    if I <= 0:
        raise ValueError("Moment of inertia should be > 0")
    if tau < 0:
        raise ValueError("Torque should be > 0")

    # angular_acceleration = r / I
    angular_acceleration = tau / I
    return angular_acceleration


def calculate_torque(F_magnitude: float, F_direction: float, r: float) -> float:
    """
    Calculates the torque applied to an object given the force applied to it and the stance from the axis of rotation to the point where the force is applied.

        Parameters:
            F_magnitude (float): The magnitude of the force applied to the object in Newtons
            F_direction (float): The direction of the force applied to the object in degrees
            r (float): The distance from the axis of rotation to the point where the force is applied in meters

        Returns:
            torque (float)
    """

    # r = r * F
    force = F_magnitude * sin(F_direction * (pi / 180))
    torque = r * force
    return torque


def calculate_moment_of_inertia(m: float, r: float) -> float:
    """
    Calculates the moment of inertia of an object given its mass and the distance from the axis of rotation to the center of mass of the object.

        Parameters:
            m (float): The mass of the object in kilograms.
            r (float): The distance from the axis of rotation to the center of mass of the object in meters.

        Returns:
            moment_of_inertia (float)
    """

    moment_of_inertia = m * (r**2)
    return moment_of_inertia


def calculate_auv_acceleration(
    F_magnitude: float,
    F_angle: float,
    mass: float = 100,
    volume: float = 0.1,
    thruster_distance: float = 0.5,
):
    """
    Calculates the acceleration of the AUV in the 2D plane.

        Parameters:
            F_magnitude (float): The magnitude of force applied by the thruster in Newtons
            F_angle (float): The angle of the force applied by the thruster in radians. The angle is measured from the x-axis. Positive angles are measured from the x-axis. Positive angles are measured in the counter-clockwise direction.
            mass (float): The mass of the AUV in kilograms.
            volume (float): The volume of the AUV in cubic meters.
            thruster_distance (float): The distance from the center of mass of the AUV to the thruster in meters.

        Returns:
            acceleration (float): Acceleration of the AUV
    """

    # a = F / m
    acceleration = F_magnitude / mass
    return acceleration


def calculate_auv_angular_acceleration(F_magnitude: float, F_angle: float) -> float:
    """
    Calculates the angular acceleration of the AUV.

        Parameters:
            F_magnitude (float)
    """

    pass


if __name__ == "__main__":
    print(calculate_buoyancy(60, density_water))
    print(will_it_float(8, 10000))
