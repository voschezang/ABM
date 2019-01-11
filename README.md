# ABM
ABM project course

Papers zoeken
- [Nagel-Schreckenberg](https://en.wikipedia.org/wiki/Nagel%E2%80%93Schreckenberg_model)
 - andere cars
- self-organization
- critical point (voor complex system)


# snelweg - banen
study self-organization	- hierdoor zorgen meer banen niet voor minder files
 - kleine perturbatie zou file moeten veroorzaken (een auto die afremt)
Hoeveel banen moeten toegevoegd worden om files te vermijden

optioneel: rij-strategie, bv aanvallend vs rustig rijden
- welke strategie is het beste?


# zeilboot
beste route op basis van lokale informatie (vijandige boten)
agents kiezen strategie: copy, aanvallend, etc
- welke strategie is het beste?
- alt: hoeveel boten moet een agent in de gaten houden?




# Setup

### Using `make`

Make sure that `pip3` is installed (python version >= 3.6).
On Mac or Linux run the following commands in your terminal. On Windows you may have to install [make](http://gnuwin32.sourceforge.net/packages/make.htm) for PowerShell .

```
git clone https://github.com/voschezang/ABM
cd AMB
make deps
```


### (Alternative) Manual setup

Install all packages listed in `requirements.txt`. Shortcut:
```
pip3 install -r requirements.txt
```
or
```
conda install --yes --file requirements.txt
```


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
