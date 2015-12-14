Imaging Sonar
=============


We need an imaging sonar for underwater SLAM. A 3D forward looking sonar (FLS) would be *ideal*, but maybe be out of our reach.

# Requirements
- High update rate, so we don't have to deal with motion nonsense
    - Most 2D multibeam sonars are unaffected by motion

# Desirements
- Avoid a plane-scanning sonar, so we don't have to mechanically scan

# What we don't want
- Low quality side-scan sonar (This is for fishermen and mapping from a surface vessel)

# Products

| Name/Link  | FOV |  Range | Update Rate (Hz)  |  Communication  |  Size  | Weight  | Price  |
|-----------:|:---:|:------:|:-----------------:|:----------------|:------:|:-------:|:------:|
| [Teledyne M900](http://www.blueview.com/products/2d-imaging-sonar/m900-series/) | 90 deg or 130 deg | 100m  | 25Hz | Ethernet/VDSL | 19.20 x 10.16 cm x 10 cm (LxWxH) | 0.86 lbs | UNKNOWN! |
| [Teledyne m450](http://www.blueview.com/products/2d-imaging-sonar/m450-series/) | 90 deg or 130 deg | 300m | 25Hz | Ethernet/VDSL | 22.86 x 19.56 x 10.16 cm (LxWxH) | 0.86 lbs | UNKNOWN! |
| [M900X FLS](http://www.blueview.com/products/2d-imaging-sonar/m900xm450x-forward-looking-multibeam-imaging-sonar-solutions/) (This is the dream) | 130x20 deg (Horizontal) 45x20 deg (Vertical) | 100m | 15 Hz | Ethernet | Unlisted  | Unlisted | UNKNOWN!|
| [M450X FLS](http://www.blueview.com/products/2d-imaging-sonar/m900xm450x-forward-looking-multibeam-imaging-sonar-solutions/) | 130x15 deg (Horizontal) 45x15 deg (Vertical) | 280m | 15 Hz | Ethernet | Unlisted  | Unlisted | UNKNOWN!|
| [Sonartronic WBMS 128 FLS](http://www.sonartronic.com/pdf/WBMS-128-FLS.pdf) | 90 deg horizontal 20 deg vertical | 100m | 20 Hz | Ethernet | 6.7 x 23.1 x 15.4 cm  (HxWxD) | 1.2 kg | UNKNOWN! |


# Yet to be explored
* FarSounder FS-3DT and FS-3ER 3D Models
* Interphase Twinscope
* Tritech Eclipse
* BlueView P900
* Marine Electronics 6201 and SeaEcho
* Reson SeaBat 7128
* Echopilot (Gold Platinum and future 3D)
* L-3 Communication Subeye

# Links
* [Video of P900-130 view](https://www.youtube.com/watch?time_continue=8&v=pSNoFgEtook)
* [Imaging Sonars for ship operators](http://www.farsounder.com/files/NavigationSonarForTheShipOperator_ForwardLookingSonarsAndMultibeamEchosoundersExplained.pdf)


# How do we get one

* Kickstarter
* Beg university for funding (Unlikely)
* Maybe we can get an old P-Series?
    * The P900 fits our reqirements very well, but they are cancelled
    * There may be a lab at UF or elsewhere in Florida that is no longer using theirs
* Researchers at UF? Mohseni?
    * Prove that we can do SLAM with the Velodyne
    * Or do Monocular SLAM
* Some of the older models suit us well, we should contact labs, government agencies and companies that may have old ones

# Notes
* Underwater SLAM for robots is not very common...people do obstacle avoidance, but not much in the way of true SLAM
    * There is a book called "Underwater SLAM in structured environments using an imaging sonar"

# Need to call
* Teledyne
* VideoRay
* Halliburton
* BlueFin
* Transocean


# So what's the plan
Find a good sonar. Call the company, and prove to them that we can make *them* the face of underwater SLAM.