from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import UserSettableParameter

from .model import MyModel
from .SimpleContinuousModule import SimpleCanvas


def boid_draw(agent):
    return {"Shape": "circle", "r": 2, "Filled": "true", "Color": "Red"}


slider = UserSettableParameter.SLIDER
"""
UserSettableParameter.__init__(
  self, param_type=None, name='',
 value=None, min_value=None, max_value=None, step=1,
  choices=list(), description=None)
"""

boid_canvas = SimpleCanvas(boid_draw, 500, 500)
model_params = {
    # "population": 100,
    "population": UserSettableParameter(slider, "Population", 100, 1, 200, 1),
    "width": 100,
    "height": 100,
    # "speed": 1,
    "speed": UserSettableParameter(slider, "Speed", 1, 1, 20, 0.1),
    # "vision": 10,
    "vision": UserSettableParameter(slider, "Vision", 10, 1, 20, 1),
    "separation": 2
}

server = ModularServer(MyModel, [boid_canvas], "Boids", model_params)
