import unittest
import numpy as np
np.random.seed(123)
from mesa.time import StagedActivation, RandomActivation

from .model import Model


class TestModel(unittest.TestCase):
    """ (Not at all finished) test module
    A number of simulations, ran reveral timesteps to increase the chance of finding errors
    """

    def run_deterministic_simulation(self):
        model = self.setup_deterministic_simulation()
        for t in range(1000):
            model.schedule.step()

    def setup_deterministic_simulation(self):
        model = Model(n_lanes=Model.MAX_LANES, verbose=2)
        model.schedule.shuffle = False
        for i in range(10):
            self.gen_deterministic_car(model)
        return model

    def gen_deterministic_car(self, model):
        model.generate_car(
            max_speed_sigma=0,
            bias_right_lane=1,
            bias_right_lane_sigma=0,
            minimal_overtake_distance=2.0)

    def run_stochastic_simulation(self):
        max_t = 100
        for seed in np.random.randint(0, 1000, max_t):
            model = Model(n_lanes=Model.MAX_LANES, seed=seed)
            for i in range(1000):
                model.schedule.step()

    def test_generate_methods(self):
        print('init model ')
        model = Model()
        print('generate car (single)')
        model.generate_car(
            max_speed_sigma=0,
            bias_right_lane=0.5,
            bias_right_lane_sigma=0,
            minimal_overtake_distance=2.0)
        print('generate cars (multiple)')
        model.generate_cars()

    def test_simulations(self):
        print('run simulations')
        self.run_deterministic_simulation()
        self.run_stochastic_simulation()

    # ...

    def test_isupper(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())

    def test_split(self):
        s = 'hello world'
        self.assertEqual(s.split(), ['hello', 'world'])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)


if __name__ == '__main__':
    unittest.main()
