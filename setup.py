from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup
import os

# Debugging output
print("Current working directory:", os.getcwd())
print("Content of src/spot directory:", os.listdir('src/spot'))

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    # packages=find_packages(where='src'),
    packages = ['spot'],
    package_dir={'': 'src'},
)
print("Packages found:", setup_args['packages'])
print("Setup arguments:", setup_args)
setup(**setup_args)
