# Lessons from RoboSub 2022

## Overall Process

***AUTOMATED testing for any system.***
    
We need to add a automated testing output for any system that can be tested. Battery system should report it's voltage. The sub computer should always have a quick self-test to confirm that all of the cameras are working. The thrusters should be explicitly visually confirmed to be spinning.


***Going to the Pool at Competition and at home*** 

The process for going to, working at, and packing up from the pool at the competition needs to be practiced.
We need to have a comprehensive list of equipment needed for any testing. That list *MUST* be the only things that are brought with us to all testing, and it *Must* be all of the things that we need for any reasonably possible challenge.


## Electrical

- BATTERY LOW VOLTAGE WARNING AND CUTOFF
- LOCKING CONNECTORS


## Mechanical

- The weight of the sub is a major detriment to easy testing and transport.
  - We implemented an "Egyptian litter" style carrier that allowed us to carry the sub with relative ease.
      - This comprised of two ~8' round wooden 1.5" diameter rods. These rods were then tied ~6' apart in two places in their middles to prevent them from opening too wide. After that, a rope with a loop in each end can be passed through the sub's frame, above it's center of mass, and all loops are passed over opposing ends of the rods. Two people can then crouch under the pair of rods, placing one on each shoulder, and then they can stand and lift the sub.
      - This system would be much improved by adding some kind of formed shoulder pads to spread the weight and add stability.


## Software
- We need intellisense for all of our code so we can quickly put together tests and missions as needed. We had lots of time loss from forgetting the way functions should be formed, mistypes, and switching contexts to find information.

- We need to practice quickly putting together missions in order to be ready to adapt as situations and capabilities change on the day.
  - This could be helped by improving our knowledge of prebuilt, generalized functions that handle more complex behaviors. (Align in front of vision target, move until target is seen, orbit position, etc...)
- We should be able to write and test code without the sub being on.
  - Multiple times during the competition we had 5-10 minutes waiting to get in to the water, and that time ended up being dead time because we weren't confident that we could move our code on to the sub once it was tethered.
  - Not implementing a gazebo sim exacerbated this issue, as we couldn't do even basic sanity checks until the sub was hooked up.
