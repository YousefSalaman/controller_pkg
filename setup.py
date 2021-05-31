
import setuptools
from os import path


# Get relevant paths
PARENT_DIR = path.dirname(__file__)
README_path = path.join(PARENT_DIR, 'README.md')

# Setup
with open(README_path) as README_file:
    setuptools.setup(
        name='controller_pkg',
        version='0.0.0',
        packages=['controller_pkg'],
        long_description=README_file.read()
    )
