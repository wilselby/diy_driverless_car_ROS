from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rbcar_twist2ack'],
    package_dir={'rbcar_twist2ack': 'src/rbcar_twist2ack'},
)

setup(**setup_args)
