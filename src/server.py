from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import UserSettableParameter

from .model import MyModel
from .SimpleContinuousModule import SimpleCanvas

# parameters
length = 1000
car_size = (5, 1.8)
lane_width = 3.5 # only for visualisation

model_params = {
    "length": length,
    "lanes": UserSettableParameter(UserSettableParameter.SLIDER, "Number of Lanes", 1, 1, 10, 1),
    "n_cars": UserSettableParameter(UserSettableParameter.SLIDER, "Number of Cars", 20, 1, 100, 1),
    "max_speed": 33,
    "car_size": car_size[0],
    "min_spacing": UserSettableParameter(UserSettableParameter.SLIDER, "Minimum spacing between cars", 1, 0, 5, 0.5),
    "car_acc": 33/10,
    "p_slowdown": UserSettableParameter(UserSettableParameter.SLIDER, "Probability of slowing down", 0.2, 0, 1, 0.1)
}


def car_portrayal(agent):
    return {
        "Shape": "rect",
        "w": car_size[0] / length, # rectangle width as fraction of space width
        "h": car_size[1] / (agent.model.lanes * lane_width), # rectangle height fraction
        "Filled": "true",
        "Color": "blue"
        }

car_canvas = SimpleCanvas(car_portrayal, 500, 50)

server = ModularServer(MyModel, [car_canvas], "Traffic simulation", model_params)
