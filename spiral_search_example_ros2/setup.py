from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
        packages=['spiral_search_example_ros2'],
    package_dir={'': 'src'})

setup(**setup_args)
