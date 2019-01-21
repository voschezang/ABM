from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import UserSettableParameter
from mesa.visualization.modules import ChartModule

from .model import MyModel
from .SimpleContinuousModule import SimpleCanvas

# parameters
length = 200
car_length = 5
car_width = 1.8
lane_width = 3.5

model_params = {
    "length":
    length,
    "lane_width":
    lane_width,
    "n_lanes":
    UserSettableParameter(UserSettableParameter.SLIDER, "Number of Lanes", 2,
                          1, MyModel.max_lanes, 1),
    "n_cars":
    UserSettableParameter(UserSettableParameter.SLIDER, "Number of Cars", 2, 1,
                          100, 1),
    "max_speed":
    UserSettableParameter(UserSettableParameter.SLIDER, "Maximum speed (km/h)",
                          120, 1, 150, 10),
    "car_length":
    car_length,
    "min_spacing":
    UserSettableParameter(UserSettableParameter.SLIDER,
                          "Minimum spacing between cars (s)", 1, 0, 5, 0.5),
    "car_acc":
    33 / 10,
    "car_dec":
    33 / 5,
    "p_slowdown":
    UserSettableParameter(UserSettableParameter.SLIDER,
                          "Probability of slowing down", 0.2, 0, 1, 0.1),
    "time_step":
    0.1
}


def car_portrayal(agent):
    return {
        "Shape": "rect",
        "w": car_length / length,  # relative to space
        "h": car_width / (MyModel.max_lanes * lane_width),  # relative
        "Filled": "true",
        "Color": "rgba(0, 0, 255, 1.0)",
        "text": agent.unique_id,
        "textColor": "black"
    }


car_canvas = SimpleCanvas(car_portrayal, 500, 200)

chart = ChartModule(
    [{
        "Label": "Density",
        "Color": "blue"
    },
    {
        "Label": "Flow",
        "Color": "red"
    }],
    data_collector_name="data_collector")

server = ModularServer(MyModel, [car_canvas, chart], "Traffic simulation",
                       model_params)
