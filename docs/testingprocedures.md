# SubjuGator Testing Procedures

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
* Duct tape and scissor
* Buoyancy foams for the sub
* Allen Key for paintball tank
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
* Zip ties
* clippers
* Towels
* [Sunscreen](https://www.youtube.com/watch?v=sTJ7AzBIJoI)


### Software/Electrical Checklist
* Update and build code
* Verify all sensors output data
* Verify thrusters spin given command
* Verify kill (soft an hard)
* Grease all electrical connectors appropriately


## At testing site

* Get wall power to powerstrip.
* Setup and connect to Network Box.
* Roll tether and connect it to network box. **(DO NOT USE POE)**
* Connect Sub to to tether. **(NOT POE)**
* Connect battery alarm, power on sub.
* SSH into sub.
* Start tmux, write code.
* Grease O-rings with Molykote 55 every time a pressure vessel is closed.
* Make sure ALL pneumatic tubes are inserted correctly. **(DO NOT FLOOD THE VALVE BOX)**
* Make sure all holes to paintball tank are sealed correctly. **(THIS WILL ALSO RESULT IN FLOODING IF NOT DONE)**
* Make sure the pressure release valve in the battery vessel (the red cap) is closed
* Person getting into the pool must do backflip.
* Deploy sub in the warer (check for bubbles, make sure buoyancy is correct)
* Verify odometry.
* Verify cameras are working

What happens when the valve box isn't closed:
![What happens when the valve box isn't closed.](/flooded_valve_box.jpg)
