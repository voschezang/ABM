# Agend-Based Model of cars
ABM project course
https://docs.google.com/document/d/1eFtdlH49CX-1uuGwS9rvptri8tylCG4f30oYA8jWjRM/edit

## TODO
Papers zoeken
- [Nagel-Schreckenberg](https://en.wikipedia.org/wiki/Nagel%E2%80%93Schreckenberg_model)
 - andere cars
- self-organization
- critical point (voor complex system)

Code
- discrete/continu van baan wisselen
- randomly generating cars
- collecting data, statistics (e.g. aantal kilometer-uur file)


## Basic agent rules
### Nagel-Schreckenberg rules
- Accelerate if agent is not driving at maximum speed
- Slow down if another car is in front of agent (within range)
- Disable backwards movement

### Extended rules
- Overtake any car in front of agent (if possible)

### direct approach
- add state : `{driving, switching lane}`
- add choice set : `{switch lane, stay on lane}`

### indirect approach
add rules such as
 - `if on lane and car_in_fron: switch lane`
 - `if in between lanes and car_in_front_and_right: go left`
 - `if in between lanes and car_in_front_and_left: go right`




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
