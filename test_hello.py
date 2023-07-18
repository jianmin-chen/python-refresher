import unittest
import hello
import numpy as np


class TestHello(unittest.TestCase):
    def test_hello(self):
        self.assertEqual(hello.hello(), "Hello, world!")

    def test_add(self):
        self.assertEqual(hello.add(0, 0), 0)
        self.assertEqual(hello.add(-5, 5), 0)
        self.assertEqual(hello.add(5, 5), 10)

    def test_sub(self):
        self.assertEqual(hello.sub(0, 0), 0)
        self.assertEqual(hello.sub(-5, 5), -10)
        self.assertEqual(hello.sub(5, 5), 0)

    def test_mul(self):
        self.assertEqual(hello.mul(0, 0), 0)
        self.assertEqual(hello.mul(-5, 5), -25)
        self.assertEqual(hello.mul(5, 5), 25)

    def test_div(self):
        self.assertRaises(ValueError, hello.div, 0, 0)
        self.assertEqual(hello.div(-5, 5), -1)
        self.assertEqual(hello.div(5, 5), 1)

    def test_sqrt(self):
        self.assertEqual(hello.sqrt(0), 0)
        self.assertTrue(np.isnan(hello.sqrt(-4)))
        self.assertEqual(hello.sqrt(25), 5)

    def test_power(self):
        self.assertEqual(hello.power(0, 0), 1)
        self.assertEqual(hello.power(5, 0), 1)
        self.assertEqual(hello.power(3, 5), 243)
        self.assertRaises(ValueError, hello.power, 3, -5)

    def test_log(self):
        self.assertEqual(hello.log(1), 0)
        self.assertEqual(hello.log(np.e), 1)
        self.assertTrue(np.isnan(hello.log(-100)))

    def test_exp(self):
        self.assertEqual(hello.exp(0), 1)
        self.assertEqual(hello.exp(-1), np.e**-1)

    def test_sin(self):
        self.assertEqual(hello.sin(0), 0)
        self.assertEqual(hello.sin(1), 0.8414709848078965)

    def test_cos(self):
        self.assertEqual(hello.cos(0), 1)
        self.assertEqual(hello.cos(1), 0.5403023058681398)

    def test_tan(self):
        self.assertEqual(hello.tan(0), 0)
        self.assertEqual(hello.tan(1), 1.5574077246549023)

    def test_cot(self):
        self.assertEqual(hello.cot(0), float("inf"))
        self.assertEqual(hello.cot(1), 0.6420926159343306)


if __name__ == "__main__":
    unittest.main()
