Use this page to record _progress_, _issues_, and other information from test days.

# May 2, 2017

We had the serial communication with the thrusters slow down (reason unknown) and it made it so
that we weren't detecting thrusters correctly because of badly written thruster driver code we
had from last year. It took us until about three hours after we were supposed to leave fix the
issue, but at least we didn't have to cancel the pool day.

At the pool we had thruster slip problems again, but after disabling two thrusters with slipping
props, we were moving around in the pool and trying to test some perception.

Our goal was to test the new path alignment code we had working on bags and simulation and chain
two of those together. It turned out that the new down camera had a too narrow field of view to see
the whole marker even at the maximum depth.

The new down camera enclosure was first used this testing day, no seal/leak problems at all. The new ethernet bulkhead on the sub also did not leak. The 1st generation POE injector was used in the sub to power the down camera, and we did not see any ethernet decode drops on the camera ethernet interface.

Some of the takeaways from testing are:
+ We should pay to get the props machined as early as possible and as soon as we are confident in
  the design
+ There are a few essential thruster driver and diagnostics improvements that need to be made for
  our sanity at competition.
+ We need to use a much wider fov lens on the down camera
+ Now that we have 3 cameras running simultaneously, the frame rate slowed down to about 15 fps for
  front cameras and 7 fps for down. These did not respond well when we tried to increase them so
  we should figure out how to get higher frame rates before the next pool day.
+ We need to make progress on [mission props](https://github.com/uf-mil/SubjuGator/wiki/Mission-Props)
---

# May 8
### GNC
* These [changes](https://github.com/uf-mil/SubjuGator/pull/233) to the thruster driver solved our previous pool day's issues with detecting thrusters, also makes it easy to discover thrusters that are mis-configured.
* We ran without thruster 3 (FRV) after hearing it scrape against the shrouds
* We were not able to track waypoints in the water with zero steady state error. There were strong oscillations around our yaw reference. The sub was stable however and did track position, pitch, and roll.
- [ ] The current controller gains have not been assured to be stable for our current mass + buoyancy parameters which have changed substantially.

### Stereo
David spent a couple of hours [calibrating the stereo cameras](https://github.com/uf-mil/SubjuGator/pull/235) with as high fidelity as possible and we were able to generate some stereo point clouds that were encouraging. He argued that we can use stereo clouds generated using this calibration to segment out things like the start and navigation gate among others. He will be using this calibration to try and generate some stereo benchmarks from Robosub 2016 data.

<img src="https://cloud.githubusercontent.com/assets/8714358/25931032/c20b3052-35d7-11e7-983b-97380052d5f7.png" width="300"> <img src="https://cloud.githubusercontent.com/assets/8714358/25931094/36a22f4c-35d8-11e7-8649-2e9b810f4b1c.png" width="300"> <img src="https://cloud.githubusercontent.com/assets/8714358/25931126/64c0a2dc-35d8-11e7-8088-8557d5e188d7.png" width="300">

### Path Marker Alignment
* More data collected for testing with new camera
* Attempted to calibrate camera underwater, could not with large part of frame blocked by camera housing
* Lack of control and calibration prevented running mission, perception recognized path marker when in frame and pose estimates seemed reasonable.
* We overcompensated for having too narrow of a field of view last pool day and used one with two wide of a field of view (too much radial distortion)
* After testing we found and tested a camera lens that works well, it just needs to have a new mount printed for it to bring it closer to the glass
  - [ ] Down camera mounted with new mount (_date_)

### OnlineBagger
STATUS:
* Tested online bagger on sub while integrated with current bagging utility successfully

ISSUES:
* Partial bagging on topics with one or zero messages (but topics that are available)
* Bagging directory was not consistent with the rest of bagging utilities
* Lots of unnecessary output to terminal when running

These problems have been addressed in [PR51](https://github.com/uf-mil/mil_common/pull/51) in mil_common **Merged**

Online Bagger has also been added to launch and kill systems now ([PR228](https://github.com/uf-mil/SubjuGator/pull/228)) **Merged**

TO DO:
  - [ ] Add feature to allow for multiple calls to bagging service simultaneously (_date_)



### Imaging Sonar
- [x] Blueview launch file had an error referencing wrong directory for color map file. [**Fixed**](https://github.com/uf-mil/SubjuGator/pull/236)


* **_PROGRESS:_** Plainer Occupied regions, safe zones, and uncertain zones using blueview and some way-point validity in C3. If missions do a tilting up-down motion, it may be possible to obtain a 3D position of an object and re-project to front cameras.

* **_ISSUES:_** Blueview. From the bags, Blueview does get hits for objects (gate, torpedo board, and sometimes buoy.)  This data can be used for estimating 3D position of an object, but not it's orientation. However, it is uncertain what the limitations are and how likely blueview will get hits of an object. Classification or volume recognition from blueview data appears to be impractical.

* **_TODO:_** Test out waypoint validity. Implement a tilt mission to populate a pointcloud with blueview. More thorough testing of blueview data on pool-days. (Daniel++)

### Gate Detection, Posing
* **_PROGRESS:_** Detection of gate works. Mission - vision stereoing to the gate.
* **_TODO:_** More robost gate detection, perhaps use blueview or stereo to obtain a pose. Add gate into simulator.

### Buoy Segmentation
* **_TODO:_** note current status, progress (Aaron)
---

# May 10

David, Ken, and Alan took the sub to the Towne Parc pool to try and get the controller (RISE) gains tuned now that our mass parameters have changed considerably and we were unable to track our steady state yaw reference the previous pool day.

### Gains
The RISE gains were tuned:
* Hardware Setup: (_Don't expect these gains to work too well if this changes substantially_)
  + Imaging Sonar mounted on wooden top plate
  + Thruster 3 (FRV) disconnected _(might have to re-tune if getting thruster 3 back negatively affects stability)_
  + Current iteration of down camera box mounted on the left side
  + Down plate from 2016 mounted
  + Pool noodles added in an attempt to get a zero net force and torque due to gravity + buyancy, although we were only able to get somewhat close
  + Nothing else not listed above was mounted, besides the computer and nav vessels
* Gains Diff:

```diff
-     ks = [620, 620, 650, 130, 130, 80]
+     ks = [372, 372, 400, 78, 78, 48]
```
  
### Thrusters
_Check off the following boxes as the issue is addressed and leave a note with name and date_
- [ ] The prop on thruster 3 (FRV) came off during operation. During a thruster spin test, we noticed that it was rubbing against the shroud and its shaft seemed to be oscillating.
- [ ] Thruster 5 propeller was also oscillating
- [ ] All thrusters should have the current prop design (joined collar and prop) mounted before the sub's next outing
- [ ] Before the next sub outing, all of the thrusters should be put on Kip's thrust gauge rig and we need to determine whether they are able to provide the Sub's maximum commanded thrust for sustained periods.

### Takeaways
* We can station keep with negligible steady state error now
* Waypoints with positional displacements in the xy-plane will be carried out with very little orientation error in the transient
* We cannot control our orientation very well during depth displacement moves, but still converge to our waypoint. (partly due to losing a vertical thruster)
* Our bus_voltage kill measurements fluctuate too much to be very useful. _See [issue #238](https://github.com/uf-mil/SubjuGator/issues/238)_
* It would be very convenient to have a script to pull all of the latest master branches from our repos into `mil_ws` on the sub. _See [issue #239](https://github.com/uf-mil/SubjuGator/issues/239)_
---