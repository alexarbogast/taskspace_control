from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = "taskspace_control_examples"

setup_args = generate_distutils_setup(
    packages=[package_name],
    package_dir={'': 'src'},
    requires=[
        "numpy",
        "numpy-quaternion",
    ],
)

setup(**setup_args)
