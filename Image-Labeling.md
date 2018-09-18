# Overview
We have hundreds of gigabytes of recorded [ROS Bags](http://wiki.ros.org/Bags) containing image data of the competition challenges. These bags span from our earliest testing in 2016, through the competition, to our recent test days. 

It is useful to have humans go through some subset of this image data and manually note which challenges are in the image and where. This serves two purposes:

* Validation: we can compare computer vision algorithms against the human labels to test performance / accuracy
* Training: human labeled image data can be used to train machine learning algorithms to recognize the challenges autonomously

#  Guide
The remainder of this page guides the user through contributing to the labeling project for fun and profit.

## Joining the project

1. Create an account on [labelbox.io](http://labelbox.io/). You can use your github account
1. Joint the uf-mil team by sending your labelbox.io email/username to Kevin (kev-the-dev) on slack
1. Go to the [navigator_computer_vision project](https://app.labelbox.com/projects/cjm82v7349sed0780ftl7pawi/overview)

## Labeling Images

> NOTE: before submitting any labeled images, please see the "Examples" section so you don't make common labeling mistakes

1. From the project page, click the blue "start labeling" button.  You will be presented with an image from previously recorded data
1. See if any of the challenges listed on the left side bar of in the image. If they are, click on this challenge and draw a rectangle as closely around the object as possible. **Be sure to see the additional Labeling Rules below** for certain objects to ignore.
1. If no objects in the image comply with the rules, click the "None" object on the sidebar and click anywhere in the image than proceed
1. Click submit or press e to submit the label and go to the next image
1. Repeat until you all images are labeled or you suffer a schizophrenic break from extreme boredom of this repetitive task. 

### Labeling Rules
Be sure to comply with the following rules for labeling so we do not end up with useless data (then we get to do it all over again!):

* If an object is far away (see examples), do NOT label it
* If an object is at extreme angles to the camera, do NOT label it.
* If the lighting makes the color of an object ambiguous for the objects that have labled colors (totems, circle, triangle, cruciform, stc panel), do NOT label it.
* Do not label the 2016 docking challenge.
* If the same object from the same scene appears many times without changes to lighting, position, etc, you may skip some of these.
* Only label the large spherical buoys, the smaller ones are not in the real competition (see examples)


### Examples
* Triangle is ignored because its color is ambiguous (is it green or blue?)
![](https://i.imgur.com/Qp1sMeU.png)
* Image contains only the small buoys (which should not be labeled), so leave as None


![](https://i.imgur.com/Qp1sMeU.png)
![](https://i.imgur.com/Qp1sMeU.png)

