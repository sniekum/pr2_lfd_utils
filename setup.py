from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pr2_lfd_utils'],
    package_dir={'': 'src'},
    requires=['actionlib', 'actionlib_msgs', 'std_msgs' , 'roslib' , 'dmp' , 'ar_track_alvar' , 'pr2_controller_msgs' , 'kinematics_msgs', 'geometry_msgs', 'moveit_msgs', 'pr2_controllers_msgs', 'rospy', 'tf', 'trajectory_msgs'],
    scripts=['scripts/moveArmsToSideMoveit.py' , 'scripts/processDemos.py' , 'scripts/recordNode.py' , 'scripts/singleReplay.py']
)

setup(**d)
