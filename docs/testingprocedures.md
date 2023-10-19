# SubjuGator Testing Procedures

## Before leaving

For best results, packing should be done at least one day before a testing session. This

### Packing list
* Power Strip
* Table
* Extension Cord
* Battery Box
* Battery charger
* Tether
* Network Box
* Ethernet Cables (2 + #of devs)
* Laptops
* 7/16 wrench
* 5/32 Allen Key
* Duct tape and scissor
* Buoyancy foams for the sub
* Pliers
* Flat-head screwdriver
* Kill wand
* Multimeter
* Goggles / Snorkeling Tube
* Pool noddles
* Dive fins
* O'ring grease (Molykote 55)
* Cable grease (Molykote 44)
* Hot glue gun
* Hot glue sticks
* Large and small zip ties
* clippers
* Towel(s)
* [Sunscreen](https://www.youtube.com/watch?v=sTJ7AzBIJoI)
* Tent (If going to Graham Pool or Florida Pool)
* Chairs (If going to Graham Pool)

### Software/Electrical Checklist
* Update (`git pull`) and build (`cm`) code.
* Verify all sensors output data.
* Verify that the correct thrusters spin given the appropriate command.
* Verify kill (soft and hard).
* Grease all electrical connectors appropriately.

## At testing site
* Get wall power to powerstrip.
* Setup and connect to Network Box.
* Roll tether and connect it to network box. **(DO NOT USE POE)**
* Connect Sub to to tether. **(NOT POE)**
* Connect battery alarm, power on sub.
* SSH into sub.
* Start tmux, write code.
* Grease O-rings with Molykote 55 every time a pressure vessel is closed.
* * **ENSURE THAT THE RED PRESSURE RELIEF CAP ON THE BATTERY TUBE HAS BEEN SCREWED IN PLACE AFTER CHANGING BATTERIES**
* Person getting into the pool must do backflip.
* Deploy sub. (check for bubbles, make sure buoyancy is correct).
* Verify odometry.

### Troubleshooting

- :ref:`Troubleshooting The Nav Tube Connection<troubleshooting-nav>`
