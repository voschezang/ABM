# Agend-Based Model of cars
ABM project course [Drive](https://docs.google.com/document/d/1eFtdlH49CX-1uuGwS9rvptri8tylCG4f30oYA8jWjRM/edit) [Overleaf](https://www.overleaf.com/1484861915nsjhycdfwwyq)

## TODO
Code:
- plot interesting data (in notebook)
 - e.g. aantal kilometer-uur file als functie van density/flow/n_lanes
- global sensitivity analysis
- implement reaction time (but how?)
- autonomous agents
 - better params (e.g. no reaction time, no standard deviation, no random slowdown)
 - communication (e.g. accelerate to global average of autonomous (nearby) cars)
 - make `lane_change_time` dependent on current velocity (cars that have zero velocity should take longer to change lanes)

Papers zoeken:
- [Nagel-Schreckenberg](https://en.wikipedia.org/wiki/Nagel%E2%80%93Schreckenberg_model)
- self-organization, critical point (voor complex system)


## Params


length -- length of the road in meters.

lane_width -- width of a lane in meters.

n_lanes -- number of lanes.

flow -- number of cars generated per lane per second (stochastic)

max_speed_mu -- maximum speed cars will try to travel at in km/h (will be converted to m/s).

max_speed_sigma -- standard deviation

car_length -- length of each car.

min_spacing -- the minimum distance in seconds a car keeps from other cars (front to back). Incorporating the cars' velocity but ignoring the other cars' velocity

min_distance_mu -- the mean of the mean min-distance for each car. Min-distance is the min. preferred amount of seconds that a car would want to keep from other cars (incl the other car's velocity). A relative distance of x seconds means that an car will reach another car in x seconds.

min_distance_sigma -- the standard deviation of the min-distance for each car

car_acc -- acceleration of the cars (in m\s2).

car_dec -- deceleration of the cars (in m\s2).

p_slowdown -- probability of a car slowing down per hour.

time_step -- in seconds.

bias_right_lane_mu -- per second. I.e. frequency of checking if going to the right lane is possible.

bias_right_lane_sigma -- standard deviation




## Basic agent rules
### Nagel-Schreckenberg rules
- Accelerate if agent is not driving at maximum speed
- Slow down if another car is in front of agent (within range)
- Disable backwards movement

### Extended rules (outdated)
- Overtake any car in front of agent (if possible)

overtake reasoning  (outdated)
```python
def step():
 update_vel()
 move()

def update_vel():
 self.vel_next = compute_next_vel()

def move():
 update_current_vel()
 update_pos()

def update_pos():
 self.pos += self.vel * dt
 ... # update model.space

def update_current_vel():
  self.vel = self.nex_vel
  
def update_next_vel():
  # returns the intended velocity of agent
  # 1
  next_vel = accelerate(self.vel) # accelerate if not at max speed

  # 2 prevent collisions
  if car_in_front:
    if not right_of_center_of_lane: # i.e. in the middle or left
      success, vel_next = steer_to_lane(vel_next, [left,right])
    else:
      success, vel_next = steer_to_lane(vel_next, [right, left])
    if not success:
      next_vel = brake(next_vel)

  else: # no car in front
    if chance to go to the right lane > random(): # (probability based on personal preference)
       _, vel_next = steer_to_lane(vel_next, [right])
    else:
      vel_next = center_on_current_lane(vel_next)
      
  self.vel_next = vel_next

def steer_to_lane(direction=[left,right]):
 # returns a tuple (success: bool, vel: (int,int) )
 for direction in directions:
  if can_go_to_lane(direction):
     self.vel_next = steer(direction)
     return (True, vel)
  return (False, center_on_current_lane(vel_next))

def center_on_current_lane(vel_next)
 # returns the required velocity to center the can to on the current lane
 ...

```



## Scenario's
This research focusses on the emergence of traffic jams in an agent-based system
- What is the minimal perturbation to cause a traffic jam? (e.g. a braking car)
- What happens to this threshold if more lanes are added?
- How does the minimum speed affect this behaviour?



# ...


## snelweg - banen
study self-organization	- hierdoor zorgen meer banen niet voor minder files
 - kleine perturbatie zou file moeten veroorzaken (een auto die afremt)
Hoeveel banen moeten toegevoegd worden om files te vermijden

optioneel: rij-strategie, bv aanvallend vs rustig rijden
- welke strategie is het beste?


## zeilboot
beste route op basis van lokale informatie (vijandige boten)
agents kiezen strategie: copy, aanvallend, etc
- welke strategie is het beste?
- alt: hoeveel boten moet een agent in de gaten houden?




# Setup

### Using `make`

Make sure that `pip`/`pip3` is installed (python version >= 3.6).
On Mac or Linux run the following commands in your terminal. On Windows you may have to install [make](http://gnuwin32.sourceforge.net/packages/make.htm) for PowerShell .

```
git clone https://github.com/voschezang/ABM
cd AMB
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
