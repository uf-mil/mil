To get started in MIL, visit the lab in MAEC 126; the software team will walk you through the next steps. To get an idea of what kind of environment MIL is, read the [[Manifesto]] page. If you are sharp, funny and hard-working, we have a place for you.


# Steps to Start Working in MIL

Assuming you've already come by the lab and spoken to us:

1. Install Ubuntu 16.04 in a virtual machine or on your bare metal if you're experienced with it
    * Debian 8.7 is also supported if you prefer that
    * Bonus points if you're already running a Linux based OS
2. See the readme in the [installer repository](https://github.com/uf-mil/installer) for information about how to install our software stack
    * Spoiler alert: it's very straightforward
3. Read the  and [[Code Policy]] page
4. Explore the Git repositories in your catkin workspace so you have an idea of what software components make up our projects
5. Ask someone to add you to the Facebook chat and mailing list
6. If you do not already have a task, choose one from the projects page on the repository of the vehicle you are working on
    * Feel free to ask around in the lab to get an idea of what needs to be done
    * If you have a task in mind, you can propose it

### Guide for Proposing a Project

You: "Hey guys, I'd like to do XYZ"
Everyone Else: "Okay"

### Meetings

We will have semi-regular meetings MAE-C 126 according to the will of Dr. Schwartz

* See step 6 above to get the dates and times
* Don't be the sort of person who has a new excuse every week


# Required Background Knowledge

* Python - the main programming language used on the project and the one recommended to new programmers
    * C++ is also an option if you feel more comfortable with it, but the learning curve to use it with ROS is a bit steeper.
* Linear Algebra at a [Khan Academy](https://www.khanacademy.org/math/linear-algebra) level
* Calculus - enough to understand the meaning of a derivative
* How to communicate and spell clearly (in English)


# Suggested Background Knowledge

* Linux (using Ubuntu or Debian)
    * If you are not familiar with Linux and Bash, I recommend you read through this [tutorial](http://ryanstutorials.net/linuxtutorial/).
* ROS - the Robot Operating System
    * This is what we use for communication between programs running on the robots
    * For more information, we recommend reading through a couple of the [tutorials](http://wiki.ros.org/ROS/Tutorials).
* Git version control and Github
    * This allows us to keep track of the history of the software stack
    * For a beginner's introduction to git, see this [tutorial](http://rogerdudler.github.io/git-guide/)
    * We use a [forking workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow) to facilitate contributions
* What a rotation matrix is in 2 and 3 dimensions (And why not to use Euler angles)
    * Bonus points for understanding how a 4x4 matrix can represent both [translation and rotation](https://en.wikipedia.org/wiki/Transformation_matrix#Affine_transformations) in 3D.

### If you are working in perception

* Basic statistics (Bayes' rule, simple probability)
* C++ if you are interested in doing point cloud analysis
* Some knowledge of computational geometry
* Really, really know what a transformation matrix is

##### Things to Start Learning

* Sebastian Thrun's Probabilistic Robotics
* Jayne's statistics
* The Point Cloud Library
* [OpenCV](http://opencv.org/) - This library enables us to perform complex robot vision tasks in a simple and elegant way. If you are intrested in robot vision, this will be your go to.
    * Try "Learning OpenCV - Computer Vision with the OpenCV Library" by Gary Bradski and Adrian Kaehler
* Machine learning for perception
* Hartley & Zisserman's Geometric Computer Vision
* [PCL](pointclouds.org)

### If you are working in control or motion planning

* C++ if you want to work with OMPL
* Really, really, really know what a transformation matrix is
* Understand why there are singularities in Euler angles
* Know what "PID" means, and at the very least be interested in MPC and adaptive control methods

##### Things to Start Learning

* Lavalle's [Planning Algorithms](http://planning.cs.uiuc.edu/)
* [OMPL](http://ompl.kavrakilab.org/)


# Useful Background Knowledge

* Numerical Methods
* Numerical Optimization (A subset of numerical methods, but optimization methods often have their own course)
    * Convex optimization
    * Gauss-Newton minimization
    * Levenberg-Marquardt Algorithm (Damped least-squares)
* Statistics and probability
    * State estimation
    * Model parameter estimation
    * Probabilistic motion planning
    * Statistical signal processing
    * Statistical image analysis
    * SLAM with uncertainty (i.e. **all** real-world SLAM)
* General planning (i.e. task planning, not motion planning)
* Computational geometry, for both perception and motion planning
* The MIT [Intro to Algorithms](http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-006-introduction-to-algorithms-fall-2011/) course
* The MIT [Underactuated Robotics](http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/) course
* The Stanford [Convex Optimization](http://stanford.edu/class/ee364a/) course