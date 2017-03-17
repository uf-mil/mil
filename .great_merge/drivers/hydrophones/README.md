Conventions used:

Incoming data is assumed to be from four hydrophones, indexed from 0 to 3,
laid out as follows:

       1
       |      x ^
    3--0        |
       |   y <--0 z (out of screen)
       2

The resulting published position point is in the coordinate frame shown above.

Other trivia:

`dist_h` is the distance (in meters, of course) from hydrophone 0 to 1 (and
equivalently, 0 to 2). `dist_h4` is the distance from 0 to 3.

The deltas (seen internally and published on the debug topic) are _delays_, so
one being more positive means that the ping was heard by that hydrophone
later.
