from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


setup(**generate_distutils_setup(
    packages=['uav_usv_fleet_gateway'],
    package_dir={'': '.'},
))
