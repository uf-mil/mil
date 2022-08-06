# Lessons from RoboSub 2022

## Overall Process

***AUTOMATED testing for any system.***

We need to add a automated testing output for any system that can be tested. Battery system should report it's voltage. The sub computer should always have a quick self-test to confirm that all of the cameras are working. The thrusters should be explicitly visually confirmed to be spinning.


***Going to the pool at competition and at home***

The process for going to, working at, and packing up from the pool at the competition needs to be practiced.
We need to have a comprehensive list of equipment needed for any testing. That list *MUST* be the only things that are brought with us to all testing, and it *MUST* be all of the things that we need for any reasonably possible challenge.


One person needs to be fully responsible for assuring that these procedures are followed every time we do anything in or near the water. This assures that bystander effect doesn't lead to avoidable mistakes causing large setbacks. This doesn't mean that everyone else doesn't help, but they should look to that designated person for confirmation. This person should likely either only be tasked with this job, or something small (like briefing the swimmer on the sub's systems) that doesn't occupy much of their time.


***Timing of rest and late night work sessions***

Several nights during the competition we would be up until 1:00-2:00, and then be up before 6:00 for the team meeting and testing time drawing in the team village. This led to critical mistakes being made when we were too tired to act on concerns and resolve issues we otherwise would have been proactive on. Sleep is critical to performing, especially when the whole team is needed to be able to run the testing cycles, set up hardware, and transport between the team village and the pool. If we had a large group, maybe that load could be shared around, but we didn't have the critical mass to let others take over our responsibilities.


***Interaction with other teams***

One of the biggest things we missed out on was getting to interact with and learn from the other teams at the event, as we were always chasing our own issues. Had we been more stable at the competition, we likely would have gotten a lot more out of that interaction


***Social Media presence***

  * [ ] There is a lot of opportunities for recruiting, outreach, connection with mentor-ship and sponsorship opportunities, and interaction with other teams over the internet. Be it Instagram, band, the mysterious data sharing platform, or old fashioned email, we are missing lots of support, motivation, opportunity, and experience

### Shipping
Total items shipped for the team was:
| # | Type                   | Shipped Via   |
|---|------------------------|---------------|
| 3 | Large Pelicans         | FedEx Ground  |
| 4 | Medium Pelicans        | FedEx Ground  |
| 1 | Small Pelicans         | FedEx Ground  |
| 1 | Sub Shipping Container | FedEx Freight |
| 1 | Battery Box            | UPS           |

Shipping was extremely painful to arrange this year. There were several primary reasons:
 - Our shipping was arranged the week that we needed to ship, and the process of loading wasn't completed until the day we were shipping the items.
 - The items that were loaded were mostly cataloged, but not all of them were enumerated.
 - The cataloging wasn't for each box, it ended up being for the set of items each team was packing.
 - We didn't know how many pelicans we would need, and ended up having to go get more from [solar park](../../infrastructure/solar_park)
 - Arranging the shipping pickup at the event ended up taking another day after we got a FedEx Express Shipment, not FedEx Ground.
   - FedEx didn't ask or specify when Dr. Schwartz scheduled the pickup, and they sent us a FedEx express truck who couldn't take our packages because they were labeled for FedEx Ground
   - Takeaway: Make sure to check what label you got, and arrange shipping to include the return label when getting it in the first place
 - We should have taken more pelicans, as you never seem to be able to pack as much in on the way home.
   - More shipping supplies, foam, and labels would have been very helpful
 - The shipping paperwork, labels, return shipping, pick-up and drop-off times and locations, and shipping manifests need to be completed several weeks before the competition. Arranging them last minute is a recipe for failure.

## Electrical

Many issues faced by the electrical team during the competition were preventable. Lack of documentation and effective ways to debug costed us a lot of time. The following will need to be worked and improved on:

- **Electrical documentation**
  - Lack of documentation made debugging harder, as we had to learn how certain systems worked on the spot
  - Documentation for each individual board and the higher level system will be beneficial for understanding the submarine and help make onboarding easier
  - Have consistent format and include information such as purpose, bill of material (BOM), and a description
- **Useful Circuits to add**
  - BATTERY LOW VOLTAGE WARNING AND CUTOFF
    - Many batteries were lost due to running the sub too long and forgetting to remove certain cables. There is a need for a board to warn and cut the power once the battery goes under a certain percentage.
  - LOCKING CONNECTORS

### Hydrophones

We talked with Forrest during the 2022 competition and he warned us that the Sylphase hydrophone board was very sensitive to moisture. While it initially had a desiccant pack placed inside during shipping, we failed to pack more to replace it after it was saturated.
The moisture causes the board to misbehave. We experienced an issue where a self check (reading and writing a value from a place called `"ADDR15"`) failed repeatedly.


The connector in the hydrophone vessel was also problematic.
We started getting an inconsistent "device not found" error after arriving at the competition.
For some unknown reason, the connector used was a reversible USB micro B cable that none of us had ever seen used for anything else. Needless to say, it was rather janky. We ended up swapping it out for a standard USB micro B cable.


## Mechanical

- The weight of the sub is a major detriment to testing and transport.
  - We implemented an "Egyptian [litter](https://en.wikipedia.org/wiki/Litter_(vehicle))" style carrier that allowed us to carry the sub with relative ease.
      - This comprised of two ~8' round wooden 1.5" diameter rods. These rods were then tied ~6" apart in two places in their middles to prevent them from opening too wide. After that, a rope with a loop in each end can be passed through the sub's frame, above it's center of mass, and all loops are passed over opposing ends of the rods. Two people can then crouch under the pair of rods, placing one on each shoulder, and then they can stand and lift the sub.
      - This system would be much improved by adding some kind of formed shoulder pads to spread the weight and add stability.
- Several mechanical and electrical systems were damaged by transport (either from shipping or moving the sub around from the team village to the pool) and had to be fixed multiple times. This involved opening up the main computer vessel, the nav tube, the valve box, and hydrophone enclosure multiple times each. Most of this damage came down to loose, non-locking connectors. (Mechanically this was mostly represented as parts moving around inside of their respective vessels and bumping wires and connectors loose.)


## Software

- We need intellisense for all of our code so we can quickly put together tests and missions as needed. We had lots of time loss from forgetting the way functions should be formed, mistypes, and switching contexts to find information.
- We need to practice quickly putting together missions in order to be ready to adapt as situations and capabilities change on the day.
  - This could be helped by improving our knowledge of prebuilt, generalized functions that handle more complex behaviors. (Align in front of vision target, move until target is seen, orbit position, etc...)
- We should be able to write and test code without the sub being on.
  - Multiple times during the competition we had 5-10 minutes waiting to get in to the water, and that time ended up being dead time because we weren't confident that we could move our code on to the sub once it was tethered.
  - Not implementing a gazebo sim exacerbated these issues, as we couldn't do even basic sanity checks until the sub was hooked up.
- Everyone working on the sub should be running the same version of the sub's current OS if at all possible. This lowers the amount of churn and multiple configurations that we need to keep a working knowledge of, and makes using tools like `ROS_MASTER_URI` forwarding quicker and easier.
