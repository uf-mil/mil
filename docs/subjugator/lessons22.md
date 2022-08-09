# Lessons from RoboSub 2022

## Overall Process

* **We need to implement an automated check system for all systems.** - For example,
the battery system should report it's voltage and the thrusters should be verified
to ensure that the proper thrusters spin when asked, a check that cameras and sensors
such as DVL, gyroscope, depth, ouput reasonable values. This should be automated
and required (and hopefully quick) so that we can avoid having busted batteries
or accidentally loose connections. At the competition, we needed to take out the
robot multiple times because of things which could have been verified _before_
the sub was put in the water.

* **We need to practice the logistic of testing in water more.** - The process for 
going to, working at, and packing up from the pool at the competition needs to be 
practiced. We need to have comprehensive list of equipment needed for any testing. 
The lists _must_ include only things that are brought with us to all testing, and 
it _must_ include all of the things that we need for any reasonably possible challenge.
A useful rule of thumb is to always be ready 10 min before. 

* **We need a dedicated testing leader.** - One person needs to be fully responsible
for assuring that these procedures are followed every time we do anything in or
near the water. This assures that bystander effect doesn't lead to avoidable
mistakes causing large setbacks. This does not mean that everyone else is unresponsible
for ensuring proper testing techniques, rather, having a dedicated member helps
to ensure that the practices will always be followed.

* **We need to rest responsibly.** - Several nights during the competition we
would not fall asleep until 1-2AM, and then be up before 6AM for the mandatory
testing time drawing. This lack of sleep caused us to make mistakes we may
not have made had we been more well-rested.

* **We should have interacted more with other teams.** - One of the most notable
things we missed out on was getting to interact with and learn from the other
teams at the event, as we were always chasing our own issues. Had we been more
stable at the competition, we likely would have had the time to interact with more
teams.

* **We should have had more of a social media presence.** - There is a lot of
opportunities for recruiting, outreach, connection with mentor-ship and sponsorship
opportunities, and interaction with other teams over the internet. Be it Instagram,
Band, the mysterious data sharing platform, or old fashioned email, we are missing
lots of support, motivation, opportunity, and experience that can be found through
these platforms.

* **We should have a designated person that signs up for testing slots.** - Two days
of competition we missed chances to test in the pool because none of our team members
woke up early to sign up for the slots. A way to solve this is to have a designated
person sign up for the slots or at least a rotation where a person is responsible to 
sign up per day. 

* **We need to pack a replacement for everything in the sub.** - We forgot the radiator, 
spare thrusters and spare motherboards. The proces of packing should not be done in a rush
and on it we should think about what could break in the sub, and bring a replacement for 
basically every part of the sub.

## Shipping
Total items shipped for the team was:
| # | Type                   | Shipped Via   |
|---|------------------------|---------------|
| 3 | Large Pelicans         | FedEx Ground  |
| 4 | Medium Pelicans        | FedEx Ground  |
| 1 | Small Pelicans         | FedEx Ground  |
| 1 | Sub Shipping Container | FedEx Freight |
| 1 | Battery Box            | UPS           |

Shipping was extremely painful to arrange this year. There were several primary reasons:
 - Our shipping was arranged the week that we needed to ship, and the process of
   loading wasn't completed until the day we were shipping the items.
 - The items that were loaded were mostly cataloged, but not all of them were enumerated.
 - The cataloguing wasn't for each box, it ended up being for the set of items each team was packing.
 - We didn't know how many pelicans we would need, and ended up having to go get
   more from [Solar Park](../../infrastructure/solar_park).
 - Arranging the shipping pickup at the event ended up taking another day after
   a FedEx Express Shipment truck attempted to pick up our packages instead of
   a FedEx Ground Shipment truck.
   - FedEx didn't ask or specify when Dr. Schwartz scheduled the pickup, and they
     sent us a FedEx express truck who couldn't take our packages because they
     were labeled for FedEx Ground.
   - **Takeaway:** Make sure to check what label you got, and arrange shipping
     to include the return label when getting it in the first place
 - We should have taken more Pelican cases, as packing materials for the return trip
   is more difficult because of time crunches, missing foam, and tiredness.
 - The shipping paperwork, labels, return shipping, pick-up and drop-off times
   and locations, and shipping manifests need to be completed several weeks
   before the competition. Arranging them last minute is a recipe for failure.

## Electrical

Many issues faced by the electrical team during the competition were preventable.
Lack of documentation and effective ways to debug costed us a lot of time. The
following will need to be worked and improved on:

- **We need more extensive electrical documentation.**
  - Lack of documentation made debugging harder, as we had to quickly learn how
    certain systems operated at the competition site.
  - Documentation for each individual board and the higher level system will be
    beneficial for understanding the submarine and help make onboarding easier.
  - Have consistent format and include information such as purpose, bill of
    material (BOM), and a description.

- **We need to add more electrical redundancy.**
  - We need to add a circuit which reports battery voltage and cuts off the batteries
    if their voltage drops too low.
    - Many batteries were lost due to running the sub too long and forgetting to
      remove certain cables. There is a need for a board to warn and cut the power
      once the battery goes under a certain percentage.
  - We need to add locking connectors. These would have prevented cables from
    accidentally coming loose while the sub was on the move.

### Hydrophones

We talked with Forrest during the 2022 competition and he warned us that the
Sylphase hydrophone board was very sensitive to moisture. While it initially had
a desiccant pack placed inside during shipping, we failed to pack more to replace
it after it was saturated.

The moisture causes the board to misbehave. We experienced an issue where a self
check (reading and writing a value from a place called `"ADDR15"`) failed repeatedly.

The connector in the hydrophone vessel was also problematic. We started getting an
inconsistent "device not found" error after arriving at the competition.
For some unknown reason, the connector used was a reversible USB micro B cable
that none of us had ever seen used for anything else. Needless to say, it was
rather janky. We ended up swapping it out for a standard USB micro B cable.

## Mechanical

- The weight of the sub is a detriment to testing and transport.
  - We implemented a [litter](https://en.wikipedia.org/wiki/Litter_(vehicle))
    style carrier that allowed us to carry the sub with relative ease.
     - This comprised of two ~8' round wooden 1.5" diameter rods. These rods were
       then tied ~6" apart in two places in their middles to prevent them from
       opening too wide. After that, a rope with a loop in each end can be passed
       through the sub's frame, above it's center of mass, and all loops are passed
       over opposing ends of the rods. Two people can then crouch under the pair
       of rods, placing one on each shoulder, and then they can stand and lift
       the sub.
     - This system would be much improved by adding some kind of formed shoulder
       pads to spread the weight and add stability.
- If we are going to continue using SubjuGator 8, the mechanical team will need
  to find a way to transport the bot around efficiently.
- Several mechanical and electrical systems were damaged by transport (either from
  shipping or moving the sub around from the team village to the pool) and had to
  be fixed multiple times. This involved opening up the main computer vessel,
  the nav tube, the valve box, and hydrophone enclosure multiple times each. Most
  of this damage came down to loose, non-locking connectors. (Mechanically this
  was mostly represented as parts moving around inside of their respective vessels
  and bumping wires and connectors loose.)

## Software

- We need to add Vim plugins on the sub's computer for all of our code so we can
  quickly put together tests and missions as needed. We had lots of time loss
  from not understanding what specific modules, classes, and functions did, and
  from syntax errors that could have been avoided by a checker.
- We need to practice quickly putting together missions in order to be ready to
  adapt as situations and capabilities change on the day.
  - This could be helped by improving our knowledge of prebuilt, generalized
    functions that handle more complex behaviors. (Align in front of vision
    target, move until target is seen, orbit position, etc...)
- We should be able to write and test code without the sub being on.
  - Multiple times during the competition we had 5-10 minutes waiting to get
    in to the water, and that time ended up being dead time because we weren't
    confident that we could move our code on to the sub once it was tethered.
  - Not implementing a gazebo sim exacerbated these issues, as we couldn't do
    even basic sanity checks until the sub was hooked up.
  - We need to come up with an easy way to move code from the developer's computer
    to the sub's computer that hopefully prevents mistakes when migrating. 
- Everyone working on the sub should be running the same version of the sub's
  current OS if at all possible. This lowers the amount of churn and multiple
  configurations that we need to keep a working knowledge of, and makes using
  tools like `ROS_MASTER_URI` forwarding quicker and easier.
