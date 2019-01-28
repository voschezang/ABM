import subprocess
import sys
import os

ignored_files = ['.DS_Store', '__init__.py']
root = 'src'
modules = []
for file_ in os.listdir(root):
    if not file_ in ignored_files:
        modules.append(root + '/' + file_)

exit_codes = []
for module in modules:
    rc = subprocess.call([
        "mypy", "--ignore-missing-imports", "--follow-imports", "skip", module
    ])
    exit_codes.append(rc)
sys.exit(max(exit_codes))
