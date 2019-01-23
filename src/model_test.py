import unittest
import numpy as np
np.random.seed(123)
from mesa.time import StagedActivation, RandomActivation

from .model import Model


class TestModel(unittest.TestCase):
    """ (Not at all finished) test module
    A number of simulations, ran reveral timesteps to increase the chance of finding errors
    """

    def test_generate_methods(self):
        print('init model ')
        model = Model()
        print('generate car (single)')
        model.generate_car(
            max_speed_sigma=0, bias_right_lane=0.5, bias_right_lane_sigma=0)

    def test_deterministic_simulation(self):
        print('run deterministic simulations')
        model = self.setup_deterministic_simulation()
        for t in range(100):
            model.step()

    def setup_deterministic_simulation(self):
        model = Model(n_lanes=Model.MAX_LANES, min_distance_sigma=0, verbose=1)
        model.schedule.shuffle = False
        for x in np.linspace(0, model.space.length, 100):
            self.gen_deterministic_car(model, x)
        return model

    def gen_deterministic_car(self, model, x=0):
        model.generate_car(max_speed_sigma=0, bias_right_lane=1, x=x)

    def run_stochastic_simulation(self, seed, n_t):
        print('\t next model')
        model = Model(n_lanes=Model.MAX_LANES, seed=seed, verbose=1)
        for i in range(int(n_t)):
            model.step()

    def test_stochastic_simulations(self, n_worlds=3, n_t=1e4):
        print('run stochastic simulations')
        for seed in np.arange(n_worlds):
            self.run_stochastic_simulation(seed, n_t)

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
