# The Resistance Code Repo
### To find the Resistance Code Repo README, click <a href="https://github.com/hzobrist/TheResistance/blob/master/README.md">here</a>.

### The robot's code is in the subscriber/scripts folder. The AI is contained in subscriber/scripts/fieldListener.py. The python function reacts to the updated states of the robots and ball.

### The Vision code is contained in publisher/src/fieldTalker.cpp

#### The best approach to reusing this code would be to install ROS and OpenCV, and then go through various tutorials. 
  * Follow the directions on the class wiki <a href="http://rwbclasses.groups.et.byu.net/doku.php?id=robot_soccer:ros">How to Install ROS</a> and <a href="http://rwbclasses.groups.et.byu.net/doku.php?id=robot_soccer:vision">How to Install OpenCV</a>.
  * Follow the various tutorials for using ROS topics, messages, services, and publishers and subscribers on the ROS website <a href="http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers">Publishers and Subscribers</a>.
  * Follow the various tutorials for OpenCV as suggested in the class wiki page for vision.
  * Using the files fieldListener.py and fieldTalker.cpp is best done by the following
    * Set up your own catkin workspaces first.
    * Add files or code to your workspace.
    * Update your CMakeLists.txt and package.xml files to work with those files.
  

