# Traffic flow model
An Agend-Based Model (ABM) used to study the emergence of traffic jams on highways.

# Setup

### Using `make`

Make sure that `pip`/`pip3` is installed (python version >= 3.6).
On Mac or Linux run the following commands in your terminal. On Windows you may have to install [make](http://gnuwin32.sourceforge.net/packages/make.htm) for PowerShell .

```
git clone https://github.com/voschezang/ABM
cd ABM
make deps
```


### (Alternative) Manual setup

Install all packages listed in `requirements.txt`. Shortcut: `pip3 install -r requirements.txt`


# Run

Start server and run a simulation in your browser
```
make run
```

### (Alternative) Manually run progam

Use an IDE or, depending on your python installation, use one of
```
python3 run.py
python run.py
pythonw run.py
```

## Analysis

Open `analysis.ipynb` with jupyter notebook.





# Basic agent rules
### Nagel-Schreckenberg rules
- Accelerate if agent is not driving at maximum speed
- Slow down if another car is in front of agent (within range)
- Disable backwards movement

### Extended rules
- Overtake any car in front of agent (if possible)
- else, go to right lane (if possible)


# Parameters

length -- length of the road in meters.

lane_width -- width of a lane in meters: 3.5m, https://nl.wikipedia.org/wiki/Rijstrook

n_lanes -- number of lanes.

flow -- number of cars generated per lane per second (stochastic)

max_speed_mu -- maximum speed cars will try to travel at in km/h (will be converted to m/s): 120

max_speed_min --

max_speed_max --

car_length -- length of each car: 4.4m,http://cardriveby.com/what-is-the-average-length-of-a-car/

min_spacing -- the minimum distance in seconds a car keeps from other cars (front to back). Incorporating the cars' velocity but ignoring the other cars' velocity: 2 seconds, https://2seconden.nl/

min_distance_mu -- the mean of the mean min-distance for each car. Min-distance is the min. preferred amount of seconds that a car would want to keep from other cars (incl the other car's velocity). A relative distance of x seconds means that an car will reach another car in x seconds.

min_distance_min --

min_distance_max --

car_acc -- acceleration of the cars (in m\s2).

car_dec -- deceleration of the cars (in m\s2).

p_slowdown -- probability of a car slowing down per hour.

time_step -- in seconds.

bias_right_lane_mu -- per second. I.e. frequency of checking if going to the right lane is possible.

bias_right_lane_sigma -- standard deviation






# Extensions / TODO
- make `lane_change_time` dependent on current velocity (cars that have zero velocity should take longer to change lanes)
- autonomous agents
 - - communication (e.g. accelerate to global average of autonomous (nearby) cars)
