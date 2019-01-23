### datacollection functions


def density(model):
    """Density: number of cars per unit length of road."""
    return len(model.schedule.agents) / model.space.length


def flow(model):
    """Flow: number of cars passing a reference point per unit of time."""
    reference_pont = model.space.length / 2

    for car in model.schedule.agents:
        if car.pos[0] > reference_pont:
            model.data.flow_cars.add(car.unique_id)

    if model.schedule.time * model.time_step % 10 < model.time_step:
        model.data.flow = len(
            model.data.flow_cars.difference(model.data.flow_cars_previous))
        model.data.flow_cars_previous = model.data.flow_cars
        model.data.flow_cars = set()

    return model.data.flow


class Data():
    def __init__(self):
        # create data collectors
        self.flow = 0
        self.flow_cars = set()
        self.flow_cars_previous = set()
