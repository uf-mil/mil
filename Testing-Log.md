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
- blueview launch file has an error referencing wrong directory for color map file, fixed and should be PRed
- oscillating (yaw) when station keeping
- down camera has significant amount of frame blocked by housing, needs to be moved closer to glass
- bus voltage kill raised with charged battery
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