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