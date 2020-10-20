Petri Nets
==========

Why?
----

Recently in MIL, our biggest weaknesses at competition have been in our mission system. Not that the system does not work. It is perfectly functional asynchronous python interface for ROS.

Strengths:
**********

* The missions are extremely flexible
* We have alot of missions and supporting code that already works

Weaknesses:
***********
* Not easy to make type safe
* Not easy to introspect about the possible states of a given mission
* Not easy to test a mission part way through
* Not easy to handle exceptions
* Requires additional API layer(s) to be written to use with ROS systems

Why is This a Better Solution?
------------------------------

Strengths:
**********

* Easier to test any state of a mission
* Easier for a human to interperet a mission after it has been written
* Easier for a human to interperet a mission while it is being run (TODO)
* Easier to write new missions that re-use elements from old missions
* Easier to automatically check for simple errors in a mission(TODO)
* Easier to deal with exceptions
* More type safe
* Fewer API layers to actually interact with a ROS system

Weaknesses:
***********

* It is not battle tested. There could still be bugs.
* Very little code using this Framework at the moment.
* No shared heap memory protection
* No garuntee that a perfectly functional petri net program will not become an sprawling, uninterpretable monster
* No live renderer (YET)
* No automatic checker (YET)
* No Network support (the whole petri is executed in one process)

What Are Petri Nets?
--------------------

Petri Nets are a mathematical modeling language first used in 1939 to describe chemical reactions.
However, Petri Nets are exetremely usful for describing any distributed system.

These distributed systems include concurrent programing.

*NOTE: (If you do not know what concurrent programming is, you can read about it* `here <https://en.wikipedia.org/wiki/Concurrent_computing>`_ *)*

How They Used in MIL?
---------------------

The ``mil_petri_nets`` package is a concurrnet programming framework where the programmer builds a concurrent program by describing a petri net in terms of a tranditional program.

This is the exact inverse of what is tranditionally done with modeling languages where the program would be described in terms of a petri net after it is written.

*Note: This framework is not unlike ROS. ROS is itself a concurrent programming framework with its own set of strengths and weaknesses. However, one of ROS's weaknesses happens to be synchronization, which is what Petri Nets are very good at describing.*

You should read **Basic Petri Net Theory** before reading **Application of Petri Nets in MIL** if you are unfamiliure with the theory of Petri Nets.

.. toctree::
  :maxdepth: 1

  Basic Petri Net Theory <theory>
  Application of Petri Nets in MIL <application>
