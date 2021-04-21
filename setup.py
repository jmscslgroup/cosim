## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

## References: http://docs.ros.org/en/jade/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cosim'],
    package_dir={'': 'src'},
)

setup(**setup_args)
