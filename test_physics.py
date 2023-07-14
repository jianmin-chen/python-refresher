import unittest
import physics


class TestPhysics(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(physics.g, 9.81)
        self.assertEqual(physics.density_water, 1000)
        self.assertEqual(physics.surface_pressure, 101325)

    def test_calculate_buoyancy(self):
        self.assertRaises(ValueError, physics.calculate_buoyancy(-5, 6))
        self.assertRaises(ValueError, physics.calculate_buoyancy(6, -5))
        self.assertEqual(physics.calculate_buoyancy(1, physics.density_water), 9810)
        self.assertEqual(physics.calculate_buoyancy(60, physics.density_water), 588600)

    def test_will_it_float(self):
        self.assertRaises(
            ValueError,
            physics.will_it_float(-5, 5),
        )
        self.assertRaises(ValueError, physics.will_it_float(5, -5))
        self.assertTrue(physics.will_it_float(50, 5))
        self.assertFalse(physics.will_it_float(9, 10000))
        self.assertIsNone(
            physics.will_it_float(10, 10000)
        )  # = density water, so neither floats nor sinks

    def test_calculate_pressure(self):
        self.assertRaises(ValueError, physics.calculate_pressure(-10))
        self.assertRaises(ValueError, physics.calculate_pressure(0))
        self.assertEqual(physics.calculate_pressure(1), 9810 + physics.surface_pressure)
        self.assertEqual(
            physics.calculate_pressure(3657), 35875170 + physics.surface_pressure
        )

    def test_calculate_acceleration(self):
        self.assertRaises(ValueError, physics.calculate_acceleration(0, 0))  # m != 0
        self.assertRaises(ValueError, physics.calculate_acceleration(0, -1))  # m != < 0
        self.assertRaises(ValueError, physics.calculate_acceleration(-1, 1))  # F != < 0
        self.assertEqual(physics.calculate_acceleration(0, 5), 0)
        self.assertEqual(physics.calculate_acceleration(10, 2), 5)

    def test_calculate_angular_acceleration(self):
        self.assertRaises(ValueError, physics.calculate_angular_acceleration(1, 0))
        self.assertRaises(ValueError, physics.calculate_angular_acceleration(1, -1))
        self.assertRaises(ValueError, physics.calculate_angular_acceleration(-1, 1))
        self.assertEqual()

    def test_calculate_torque(self):
        pass

    def test_calculate_moment_of_inertia(self):
        pass

    def test_calculate_auv_acceleration(self):
        pass

    def test_calculate_auv_angular_acceleration(self):
        pass


if __name__ == "__main__":
    unittest.main()
