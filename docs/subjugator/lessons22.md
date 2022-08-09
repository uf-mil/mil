# Lessons from RoboSub 2022

## Overall Process

***AUTOMATED testing for any system.***

We need to add a automated testing output for any system that can be tested. Battery system should report it's voltage. The sub computer should always have a quick self-test to confirm that all of the cameras are working. The thrusters should be explicitly visually confirmed to be spinning.


***Going to the Pool at Competition and at home***

The process for going to, working at, and packing up from the pool at the competition needs to be practiced.
We need to have a comprehensive list of equipment needed for any testing. That list *MUST* be the only things that are brought with us to all testing, and it *Must* be all of the things that we need for any reasonably possible challenge.

### Shipping
Shipping was extremely painful to arrange this year. There were several primary reasons:
 - Our shipping was arranged the week that we needed to ship, and the process of loading wasn't completed until the day we were shipping the items.
 - The items that were loaded were mostly catalouged, but not all of them were enumerated.
 - The cataloging wasn't for each box, it ended up being for the set of items each team was packing.
 - We didn't know how many pelicans we would need, and ended up having to go get more from solar park

## Electrical

Many issues faced by the electrical team during the competition were preventable. Lack of documentation and effective ways to debug costed us a lot of time. The following will need to be worked and improved on:

- **Electrical documentation**
  - Lack of documentation made debugging harder, as we had to learn how certain systems worked on the spot
  - Documentation for each individual board and the higher level system will be beneficial for understanding the submarine and help make onboarding easier
  - Having a consistant format and include information such as purpose, bill of material (BOM), and a description would make reading documentation faster
  - Use GitHub issues to share work that needs to be done 
- **Circuitry**
  - BATTERY LOW VOLTAGE WARNING AND CUTOFF
    - Many batteries were lost due to running the sub too long and forgetting to remove certain cables. There is a need for a board to audibly warn and cut the power once the battery goes under a certain percentage. 
  - SYSTEM STATUS
    - Certain electrical systems went down due to loose connections (see Locking Connectors section). A board that checks whether each electrical system is powered and trasmitting/receiving data properly will be immensly helpful to lower the amount of time to debug and fix.
  - DEBUG PINS
    - Most electrical systems lack debug pins that could be used to check signals with an oscilliscope. The USB to CAN board is one of the few boards that contain this and is useful to have to check whether the board functions properly. This will require updating existing circuit and PCB to include debug headers
  - LOCKING CONNECTORS
    - During the competition, many wires and connections fell off, which disconnected certain systems. Due to the poor wire management, this was difficult to trace and debug. Locking connectors and other methods that will keep secure connections through shipping and transportation is needed to save testing time.
  - BETTER SOLDERING TOOLS
    - Our heat gun and one soldering gun stopped working during the competition. These items need to be replaced.
- **Random**
  - Bring tupperware for food

### Hydrophones

We talked with Forrest during the 2022 competition and he warned us that the Sylphase hydrophone board was very sensitive to moisture. While it initially had a desiccant pack placed inside during shipping, we failed to pack more to replace it after it was saturated.
The moisture causes the board to misbehave. We experienced an issue where a self check (reading and writing a value from a place called `"ADDR15"`) failed repeatedly. We had opened the board in the team village (which was exceptionally muggy that day) minutes before getting that issue.


The connector in the hydrophone vessel was also problematic.
We started getting an inconsistent "device not found" error after arriving at the competition.
For some unknown reason, the connector used was a reversible USB micro B cable that none of us had ever seen used for anything else. Needless to say, it was rather janky. We ended up swapping it out for a standard USB micro B cable.


## Mechanical

- The weight of the sub is a major detriment to testing and transport.
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
