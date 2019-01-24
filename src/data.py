from mesa.datacollection import DataCollector

### datacollection functions


def density(model):
    """Density: number of cars per unit length of road."""
    return len(model.schedule.agents) / model.space.length


def flow(model):
    """Flow: number of cars passing a reference point per unit of time."""

    # get the flow in the current timestep
    flow_in_timestep = model.data.flow
    # reset flow counter
    model.data.flow = 0
    return flow_in_timestep


class Data(DataCollector):
    def __init__(self, flow_reference_point):
        super().__init__(model_reporters={
                #"Density": density,
                "Flow": flow
            })

        # setup data collectotion variables
        self.flow_reference_point = flow_reference_point
        self.flow = 0

