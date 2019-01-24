from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import UserSettableParameter
from mesa.visualization.modules import ChartModule

from .model import Model
from .SimpleContinuousModule import SimpleCanvas

# parameters
length = 1000

model_params = {
    "n_lanes":
    UserSettableParameter(UserSettableParameter.SLIDER, "Number of Lanes", 2,
                          1, Model.MAX_LANES, 1),
    "n_cars":
    UserSettableParameter(UserSettableParameter.SLIDER, "Number of cars", 10,
                          1, 100, 5),
    "fraction_autonomous":
    UserSettableParameter(UserSettableParameter.SLIDER,
                          "Fraction of autonomous cars", 0, 0, 1, 0.1),
    "min_spacing":
    UserSettableParameter(UserSettableParameter.SLIDER,
                          "Minimum spacing between cars (s)", 2, 0, 3, 0.5),
    "p_slowdown":
    UserSettableParameter(
        UserSettableParameter.SLIDER,
        "Frequency of a car slowing down randomly (per hour)", 60, 0, 100, 1),
}


def car_portrayal(agent):
    return {
        "Shape": "rect",
        "w": agent.length / agent.model.space.length,
        "h": agent.width /
        (Model.MAX_LANES * agent.model.space.lane_width),  # relative
        "Filled": "true",
        "Color": "rgba(0, 0, 255, 1.0)" if not agent.autonomous else "red",
        "text": agent.unique_id,
        "textColor": "black"
    }


car_canvas = SimpleCanvas(car_portrayal, 800, 200)

chart = ChartModule(
    [{
        "Label": "Density",
        "Color": "blue"
    }, {
        "Label": "Flow",
        "Color": "red"
    }],
    data_collector_name="data")

server = ModularServer(Model, [car_canvas, chart], "Traffic simulation",
                       model_params)
