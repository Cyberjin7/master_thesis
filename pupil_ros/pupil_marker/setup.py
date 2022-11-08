from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup(
    packages=['rqt_bag_pupil_marker'],
    package_dir={'': 'src'}
)

setup(**package_info)
