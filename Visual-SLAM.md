Visual SLAM
===========

Right now, there are three parallel paths to visual navigation:

Ralph & David: Forward camera SLAM and stereo point cloud generation

David: Visual Odometry from conventional optical flow

Tess: PTAM/DTAM + MonoSLAM for down-camera

Jake: SVO + MonoSLAM for down-camera

# Considerations
* Work in an underwater "outdoor" environment
* Handle occlusion due to opaqueness of water
* Handle occlusion in general
* Provide a motion estimate based on world knowledge, and not just integrating velocity (i.e. DVL)
* Generate a traversibility map
* Work in real-time

# Questions
* Do we need to enhance the simulator to support visual SLAM activities?
* Do we need better cameras?

# Notes
* Down-camera and forward camera SLAM are almost certainly different problems. Each works with remarkably different scene disparity, and the forward cameras work with much more occlusion than the down-camera


# Down-Camera

## Implementation Milestones

Topics that appear to be common to essentially every monocular SLAM implementation, or will likely come in handy
* Depth-Filter

* 2D-2D correspondence -> 3D transform estimation

* Sparse bundle adjustment for depth initialization (Tools exist for this)

## Notes
* PTAM is widely regarded as one of the best MAV SLAM approaches

* **Why don't we just gather features and do the 5-pt algorithm?**
    * The results are extremely noisy
    * Can't make a useful map/recover scene depth
    * No way to reason about the quality of the track produced
    * *Each relative pose is known only to a translational scaling!!!!*


## General Knowledge
* [Review of Epipolar geometry](https://www.robots.ox.ac.uk/~vgg/hzbook/hzbook1/HZepipolar.pdf)
    * This is super important!


* [Representation in Homogeneous Coordinates](http://robotics.stanford.edu/~birch/projective/node4.html)


* [Multiple View Geometry in Computer Vision - Hartley, Zisserman](ftp://vista.eng.tau.ac.il/dropbox/aviad/Hartley,%20Zisserman%20-%20Multiple%20View%20Geometry%20in%20Computer%20Vision.pdf)

    * This book is amazing, but the link will die someday. Just google $title + 'pdf'

    * Jake has a pdf - ask him if you want it


* [Markov Random Field Optimization](http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/AV0809/ORCHARD/)


## References

* [1] [SVO (Semi-Direct monocular Visual Odometry)](http://rpg.ifi.uzh.ch/docs/ICRA14_Forster.pdf)

        * [Other Link](https://github.com/uzh-rpg/rpg_svo)

* [2] [ORB_SLAM](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)

        * [Other Link](https://github.com/raulmur/ORB_SLAM)

* [3] [Inverse Depth Parameterization for Monocular SLAM](https://www.doc.ic.ac.uk/~ajd/Publications/civera_etal_tro2008.pdf)

* [4] [Five-Point Estimation Made Easy](http://users.cecs.anu.edu.au/~hongdong/new5pt_cameraREady_ver_1.pdf) - Estimate relative pose from 2d-2d correspondences


## Tools

* [OpenGV](http://laurentkneip.github.io/opengv/index.html) - Open Geometric Computer Vision (C++)

    * Documentation is sparse (Maybe I'm not looking hard enough?)

* [OpenMVG](http://imagine.enpc.fr/~moulonp/openMVG/) - Open Multi-View Geometry Tools (C++)

    * (Is this a good idea?)

* [SBA](http://users.ics.forth.gr/~lourakis/sba/) - Sparse Bundle Adjustment library (C++)

* [Theia](http://www.theia-sfm.org/applications.html) - Open Structure from Motion library

* [Ceres Solver](http://ceres-solver.org/nnls_tutorial.html#introduction) - Google's very own "not-your-mother's-solver" NLLS/general minimization solver


## Equipment
* Structured Light Projector: 10 Lumens Pico Microvision laser projector
    * [$130](http://www.amazon.com/MicroVision-SHOWWX-Classic-Laser-Projector/dp/B003G5ML9Y)
    * [$285](http://www.amazon.com/MicroVision-SHOWWX-Classic-Laser-Projector/dp/B003G5ML9Y)