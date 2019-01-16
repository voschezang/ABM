from mesa.visualization.ModularVisualization import VisualizationElement

from .model import MyModel

class SimpleCanvas(VisualizationElement):
    local_includes = ["src/js/simple_continuous_canvas.js"]

    def __init__(self, portrayal_method, canvas_width=500, canvas_height=500):
        '''
        Instantiate a new SimpleCanvas
        '''
        self.portrayal_method = portrayal_method
        self.canvas_height = canvas_height
        self.canvas_width = canvas_width
        new_element = ("new Simple_Continuous_Module({}, {})".format(
            self.canvas_width, self.canvas_height))
        self.js_code = "elements.push(" + new_element + ");"

    def render(self, model):
        space_state = []
        for obj in model.schedule.agents:
            portrayal = self.portrayal_method(obj)
            x, y = obj.pos
            x = ((x - model.space.x_min) /
                 (model.space.x_max - model.space.x_min))
            y = ((y - model.space.y_min) /
                 (model.space.y_max - model.space.y_min))

            # scale the y-axis such that a lane does not change it position if the number of lanes is changed
            y = (MyModel.max_lanes - model.space.n_lanes) / MyModel.max_lanes + y * model.space.n_lanes / MyModel.max_lanes
 
            portrayal["x"] = x
            portrayal["y"] = y
            space_state.append(portrayal)
        return space_state
