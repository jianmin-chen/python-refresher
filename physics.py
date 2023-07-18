import numpy as np
import pprint

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


"""
Functions for part 2 start here
"""


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
    force = F_magnitude * np.sin(F_direction * (np.pi / 180))
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

    moment_of_inertia = m * np.power(r, 2)
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

    acceleration_x = calculate_acceleration(F_magnitude * np.cos(F_angle), mass)
    acceleration_y = calculate_acceleration(F_magnitude * np.sin(F_angle), mass)
    return np.array([acceleration_x, acceleration_y])


def calculate_auv_angular_acceleration(
    F_magnitude: float, F_angle: float, inertia: float, thruster_distance: float
) -> float:
    """
    Calculates the angular acceleration of the AUV.

        Parameters:
            F_magnitude (float): The magnitude of force applied by the thruster in Newtons.
            F_angle (float): The angle of the force applied by the thruster in radians.
            inertia (float): The moment of inertia of the AUV in kg * m^2.
            thruster_distance (float): The distance from the center of the mass of the AUV to the thruster in meters.
    """

    torque = calculate_torque(F_magnitude, np.deg2rad(F_angle), thruster_distance)
    angular_acceleration = calculate_angular_acceleration(
        torque, inertia
    )  # Rearrange r = rF
    return angular_acceleration


def gen_rotation_matrix(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]])


def calculate_auv2_acceleration(T: np.ndarray, alpha: float, mass: float = 100):
    """
    Calculates the acceleration of the AUV in the 2D plane.

        Parameters:
            T (np.ndarray): An `np.ndarray` of the magnitudes of the forces applied by the thrusters in Newtons.
            alpha (float): The angle of the thrusters in radians.
            mass (float): The mass of the AUV in kilograms.
    """

    if mass <= 0:
        raise ValueError("Mass should be > 0 kilograms")

    # Do the same thing described yesterday
    # F = ma
    # Find all the forces, then divide by mass to get acceleration
    # T should account for a variable length???
    direction_matrix = np.array(
        [
            [np.cos(alpha), np.cos(alpha), -np.cos(alpha), -np.cos(alpha)],
            [np.sin(alpha), -np.sin(alpha), -np.sin(alpha), np.sin(alpha)],
        ]
    )
    F_prime = np.matmul(direction_matrix, T)
    rotation_matrix = gen_rotation_matrix(alpha)
    (Fx, Fy) = np.matmul(rotation_matrix, F_prime)
    acceleration = np.array([Fx / mass, Fy / mass])
    return acceleration


def calculate_auv2_angular_acceleration(
    T: np.ndarray, alpha: float, L: float, l: float, inertia: float = 100
):
    """
    Calculates the angular acceleration of the AUV.

        Parameters:
            T (np.ndarray): An `np.ndarray` of the magnitudes of the forces applied by the thrusters in Newtons.
            alpha (float): The angle of the thrusters in radians.
            L (float): The distance from the center of mass of the AUV to the thrusters in meters.
            l (float): The distance from the center of mass of the AUV to the thrusters in meters.
            inertia (float): The moment of inertia of the AUV in kg * m^2
    """

    # Do the same thing described yesterday
    # r = I(alpha)
    # Find alpha, which is the angular acceleration
    # alpha = r / I
    # I is inertia
    # r = rF
    # Wait no T is the array of magnitudes
    # F = ma
    """
    force = F_magnitude * np.sin(F_direction * (np.pi / 180))
    torque = r * force
    return torque
    torque = calculate_torque(F_magnitude, np.deg2rad(F_angle), thruster_distance)
    angular_acceleration = calculate_angular_acceleration(
        torque, inertia
    )  # Rearrange r = rF
    return angular_acceleration
    """

    # ???
    r = np.sqrt(np.power(l, 2) + np.power(L, 2))

    beta = np.arctan(L / l)
    new_force = T[0] + -T[1] + T[2] - T[3]
    return np.sin(alpha + beta) * r * new_force / inertia


def simulate_auv2_motion(
    T: np.ndarray,
    alpha: float,
    L: float,
    l: float,
    mass: float = 100,
    inertia: float = 100,
    dt: float = 0.1,
    t_final: float = 10,
    x0: float = 0,
    y0: float = 0,
    theta0: float = 0,
):
    """
    Simulates the motion of the AUV in the 2D plane.

        Parameters:
            T (np.ndarray): An `np.ndarray` of the magnitudes of the forces applied by the thrusters in Newtons.
            alpha (float): The angle of the thrusters in radians.
            L (float): The disance from the center of mass of the AUV to the thrusters in meters.
            l (float): The distance from the center of mass of the AUV to the thrusters in meters.
            mass (float): The mass of the AUV in kg. The default value is 100kg.
            inertia (float): the moment of inertia of the AUV in kg * m^2. Default value  is 100kg * m^2.
            dt (float): The time step of the simulation in seconds. The default value is 0.1s.
            t_final (float): The final time of the simulation in seconds. The default value is 10s.
            x0 (float): The initial x-position of the AUV in meters. The default value is 0m.
            y0 (float): The initial y-position of the AUV in radians. The default value is 0m.
            theta0 (float): The initial angle of the AUV in radians. The default value is 0rad.

        Returns:
            t (np.ndarray): The time steps of the simulation in seconds. :check
            x (np.ndarray): An np.ndarray of the x-positions of the AUV in meters.
            y (np.ndarray): An np.ndarray of the y-positions of the AUV in meters.
            theta (np.ndarray): An np.ndarray of the angles of the AUV in radians.
            v (np.ndarray): An np.ndarray of the velocities of the AUV in meters per second.
            omega (np.ndarray): An np.ndarray of the angular velocities of the AUV in radians per second.
            a (np.ndarray): An np.ndarray of the accelerations of the AUV in meters per second squared.
    """

    data = {
        "t": np.arange(0, t_final, dt),
        "x": np.array([x0]),
        "y": np.array([y0]),
        "theta": np.array([theta0]),
        "v": np.array([0]),
        "omega": np.array([0]),
        "a": np.array([calculate_auv2_acceleration(T, alpha, mass)]),
    }

    angular_acceleration = calculate_auv2_angular_acceleration(T, alpha, L, l, inertia)

    t = dt
    while t < t_final:
        angular_velocity = angular_acceleration * dt + data["omega"][-1]

        robot_angle = data["omega"][-1] * dt + data["theta"][-1]

        robot_acceleration = calculate_auv2_acceleration(T, robot_angle, mass)

        robot_velocity = data["a"][-1] * dt + data["v"][-1]

        robot_x = data["v"][-1] * dt + data["x"][-1]
        robot_y = data["v"][-1] * dt + data["y"][-1]

        data["omega"] = np.append(data["omega"], angular_velocity)

        data["theta"] = np.append(data["theta"], robot_angle)

        data["a"] = np.append(data["a"], robot_acceleration)

        data["v"] = np.append(data["v"], robot_velocity)

        data["x"] = np.append(data["x"], robot_x)
        data["y"] = np.append(data["y"], robot_y)

        data["t"] = np.append(data["t"], t)
        t += dt

    return data


if __name__ == "__main__":
    res = simulate_auv2_motion(np.array([4, 4, -2, -2]), np.pi / 4, 4, 4)
    print(res["t"].size, res["x"].size, res["y"].size)
