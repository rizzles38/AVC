# Installing ROS packages from source.

Under `~bsomers/ros_workspace` on the robot, run:
    $ ./add_src_package <name of package>

When done, run:
    $ ./fetch_src_packages

Then run:
    $ ./deps

Add ROS source package deps, system deps, AUR deps, etc. as necessary to satisfy
all dependencies. When finally done, run:
    $ ./make

And everything should be baked into `/opt/ros/kinetic`.
