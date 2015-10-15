Getting Started Guide
=====================

1. Clone this repository. As of now, Sub8 has no dependencies on the rest of uf-mil, so it can exist in its own catkin workspace.

2. Read the code policy page on this wiki. 

3. Get to know the people on the team


Check out a few libraries:
[OpenCV](http://opencv.org/), [PCL](pointclouds.org), [OMPL](http://ompl.kavrakilab.org/)

And a few courses:
[Linear Algebra](https://www.khanacademy.org/math/linear-algebra), [Intro to Algorithms](http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-006-introduction-to-algorithms-fall-2011/), [Underactuated Robotics](http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/), [Convex Optimization](http://stanford.edu/class/ee364a/)

There is another Underactuated robotics course taught by the same guy on Edx. The Edx course is much better.


# Required Background Knowledge

If you are working on robotics in general, you should know...

* Python, enough to know how to Google syntax questions

* Linear Algebra, enough to explain least-squares and the significance of an eigenvector

* Calculus, enough to understand the meaning of a derivative

* What a rotation matrix is in 2 and 3 dimensions (And why not to use Euler angles)

    * Bonus points for understanding how a 4x4 matrix can represent both [translation and rotation](https://en.wikipedia.org/wiki/Transformation_matrix#Affine_transformations) in 3D.

* How to communicate and spell clearly (in English)

## If you are working in perception

#### Know

* Basic statistics (Bayes' rule, simple probability)

* C++ if you are interested in doing point cloud analysis

* Really, really know what a transformation matrix is

#### Start with

* Sebastian Thrun's Probabilistic Robotics

* Jayne's statistics

* The Point Cloud Library

* OpenCV

* Machine learning for perception

## If you are working in control or motion planning

* C++ if you want to work with OMPL

* Really, really, really know what a transformation matrix is

* And super especially understand why there are singularities in Euler angles

* Know what "PID" means, and at the very least be interested in MPC and adaptive control methods

* And know why we don't do these using Euler angles