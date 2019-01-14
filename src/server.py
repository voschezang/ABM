from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import UserSettableParameter

from .model import MyModel
from .SimpleContinuousModule import SimpleCanvas

# parameters
length = 100
car_length = 5
car_width = 1.8
lane_width = 3.5  # for visualisation

default_n_lanes = 4

model_params = {
    "length":
    length,
    "n_lanes":
    UserSettableParameter(UserSettableParameter.SLIDER, "Number of Lanes",
                          default_n_lanes, 1, 10, 1),
    "n_cars":
    UserSettableParameter(UserSettableParameter.SLIDER, "Number of Cars", 20,
                          1, 100, 1),
    "max_speed":
    33,
    "car_length":
    car_width,
    "min_spacing":
    UserSettableParameter(UserSettableParameter.SLIDER,
                          "Minimum spacing between cars", 1, 0, 5, 0.5),
    "car_acc":
    33 / 100,
    "p_slowdown":
    UserSettableParameter(UserSettableParameter.SLIDER,
                          "Probability of slowing down", 0.2, 0, 1, 0.1)
}


def car_portrayal(agent):
    return {
        "Shape": "rect",
        "w": car_length / length,  # relative to space
        "h": car_width / (agent.model.n_lanes * lane_width),  # relative
        "Filled": "true",
        "Color": "blue"
    }


canvas_height = (
    default_n_lanes + 1) * lane_width * 2  # TODO update this value after init
car_canvas = SimpleCanvas(car_portrayal, 500, canvas_height)

server = ModularServer(MyModel, [car_canvas], "Traffic simulation",
                       model_params)
