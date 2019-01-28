run:
	python3 run.py

jupyter :
	jupyter notebook

deps:
	pip3 install -r requirements.txt

test:
	make cleanup
	make test-types
	python3 -m unittest discover -s . -p "*_test.py"

test-types:
	python3 -m run_mypy.py

cleanup:
	rm -rf */__pycache__
	rm -rf */mypy_cache
