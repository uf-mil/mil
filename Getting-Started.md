Getting Started Guide for the Subjugator Software Team
======================================================
## [Can I work on the Sub?](https://github.com/uf-mil/Sub8/wiki/Manifesto#can-i-work-on-the-sub)

# How to join

1. Come on by MAE-C 126 and find out what's going on in MIL.

2. You will walk through the next steps with one of the software guys in there.

3. If you don't feel that you meet our requirements, but would still like to get started working on robots, there are other projects in the lab with less stringent requirements
    * If you are sharp, funny and hard-working, we have a place for you, no matter what

# Getting started with code

Assuming you've already come by the lab and spoken to us

1. Install Ubuntu 14.04

2. Read the code policy page on this wiki.

3. Install Sub8 and dependencies (use install.sh, follow the instructions the Sub8 repository readme).

4. Skim over the repository's ROS packages so you have an idea of what the software components are.

5. If you do not already have a task, choose one from [[Implementation Plans]], or propose a project

6. Ask someone to add you to the sub Facebook chat, mailing list, or to another contact medium.

Make sure you have some task or learning in progress before you start coming to Wednesday meetings.

## Guide for Proposing a Project
You: "Hey guys, I'd like to do XYZ"
Everyone Else: "Okay."

# Meetings

* We will have semi-regular meetings MAE-C 126 according to the will of Dr. Schwartz.

* Don't be the sort of person who has a new excuse every week

# Learning background material

Check out a few libraries:
[OpenCV](http://opencv.org/), [PCL](pointclouds.org), [OMPL](http://ompl.kavrakilab.org/)

And a few courses:
[Linear Algebra](https://www.khanacademy.org/math/linear-algebra), [Intro to Algorithms](http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-006-introduction-to-algorithms-fall-2011/), [Underactuated Robotics](http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/), [Convex Optimization](http://stanford.edu/class/ee364a/)

And some guides: [Git the Simple Guide](http://rogerdudler.github.io/git-guide/)

There is another Underactuated robotics course taught by the same guy on Edx. The Edx course is much better.


# Required Background Knowledge

If you are working on robotics in general, you should know...

* Python, enough to know how to Google syntax questions

* Linear Algebra at a Khan Academy level

* Calculus, enough to understand the meaning of a derivative

* How to communicate and spell clearly (in English)

# Suggested Background Knowledge

You should know...

* What a rotation matrix is in 2 and 3 dimensions (And why not to use Euler angles)

    * Bonus points for understanding how a 4x4 matrix can represent both [translation and rotation](https://en.wikipedia.org/wiki/Transformation_matrix#Affine_transformations) in 3D.


## If you are working in perception

#### Know

* Basic statistics (Bayes' rule, simple probability)

* C++ if you are interested in doing point cloud analysis

* Some knowledge of computational geometry

* Really, really know what a transformation matrix is

#### To learn, start with

* Sebastian Thrun's Probabilistic Robotics

* Jayne's statistics

* The Point Cloud Library

* OpenCV

    * Try "Learning OpenCV - Computer Vision with the OpenCV Library" by Gary Bradski and Adrian Kaehler

* Machine learning for perception

* Hartley & Zisserman's Geometric Computer Vision


## If you are working in control or motion planning

* C++ if you want to work with OMPL

* Really, really, really know what a transformation matrix is

* And super especially understand why there are singularities in Euler angles

* Know what "PID" means, and at the very least be interested in MPC and adaptive control methods

### To learn, start with

* Lavalle's [Planning Algorithms](http://planning.cs.uiuc.edu/)


## Other stuff

Some other topics that are supremely useful:

* Numerical Methods

* Numerical Optimization (A subset of numerical methods, but optimization methods often have their own course)
    * Convex optimization
    * Gauss-Newton minimization
    * Levenberg-Marquardt Algorithm (Damped least-squares)

* Statistics and probability
    * State estimation
    * Model parameter estimation
    * Probabilistic motion planning
    * Statistical signal processing / statistical image analysis
    * SLAM w/ uncertainty (All real-world SLAM)

* Medical Imaging (A huge field with lots of research on crazy perception)
    * Active contours

* General planning (Task-planning beyond motion planning)

* Computational geometry, for both perception and motion planning