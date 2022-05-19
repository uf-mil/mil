# Testing Procedures:

## Before leaving

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
* Allen Key for paintball tank
* Pliers
* Flat-head screwdriver
* Kill wand
* Goggles / Snorkeling Tube
* Pool noddles
* Dive fins
* O'ring grease
* Zip ties
* clippers
* Towels


### Software/Electrical Checklist
* Update and build code
* Verify all sensors output data
* Verify thrusters spin given command
* Verify kill (soft an hard)
* Grease all electrical connectors appropiately


## At testing site

* Get wall power to powerstrip.
* Setup and connect to Network Box.
* Roll tether and connect it ot network box. **(DO NOT USE POE)**
* Connect Sub to to tether. **(NOT POE)**
* Connect battery alarm, power on sub.
* SSH into sub.
* Start tmux, write code.
* Grease O-rings with Molykote 55 everytime a pressure vessel is closed.
* Make sure ALL pneumatic tubes are inserted correctly. **(DO NOT FLOOD THE VALVE BOX)**
* Make sure all holes to paintball tank are sealed correctly. **(THIS WILL ALSO RESULT IN FLOODING IF NOT DONE)**
* Person getting into the pool must do backflip.
* Deploy sub. (check for bubbles, make sure buoyancy is correct)
* Verify odometry.

What happens when the valve box isn't closed:
![What happens when the valve box isn't closed.](/docs/flooded_valve_box.jpg)
