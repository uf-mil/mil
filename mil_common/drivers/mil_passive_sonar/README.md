# mil_passive_sonar
This package includes the main algorithm to find the direction of a pinger. It also includes some debug tools

# Running:
Ensure that hydrophones -> ros is running and publishing a `mil_passive_sonar/Ping` msg.

Then run `roslaunch mil_passive_sonar mil_passive_sonar.lauch`

`mil_passive_sonar/FindPinger`

# Conventions used:

Incoming data is assumed to be from four hydrophones, indexed from 0 to 3,
laid out as follows:

       1
       |      x ^
    3--0        |
       |   y <--0 z (out of screen)
       2

The resulting published position point is in the coordinate frame shown above.

# Other trivia:

`mil_passive_sonar.yaml` contains `dish_h`, `dist_h4`, and `v_sound`.
`dist_h` is the distance (in meters, of course) from hydrophone 0 to 1 (and
equivalently, 0 to 2). `dist_h4` is the distance from 0 to 3.

The deltas (seen internally and published on the debug topic) are _delays_, so
one being more positive means that the ping was heard by that hydrophone
later.

`v_sound` is speed of sound in meters/sec. Typically measured value is 1482m/s at 20c.
