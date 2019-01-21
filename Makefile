run:
	python3 run.py


jupyter :
	jupyter notebook

deps:
	pip3 install -r requirements.txt

test:
	python3 -m unittest discover -s . -p "*_test.py"
