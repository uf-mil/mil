Application of Petri Nets in MIL
================================

How are petri nets applied to concurrent programming with this library?

Places
------

Role
****

Places are supposed to be where the actual computation / actions in a system are done.

Structure
*********

Callback Function
^^^^^^^^^^^^^^^^^

A place has a single callback function which intakes a single constant token and returns a single token.

Output Channels
^^^^^^^^^^^^^^^

In order to support the returning of mutliple types from a single place, a place has mutliple output channels, exaclty one for each registered return type. If a place returns a token of a type for which there is no registered output type, the whole petri net is killed, and an error message is generated.

These output channels do not break the rules of petri nets as they are modeled by each output channel being a transition followed by a place. In petri net theory (that is for non-colored petri nets), it is generally agreed upon that if two transitoins can fire, it cannot be predicted which will fire.

Since this framework makes no assumptions about what code will be run inside of each place, then it is considered "not predictable" from a petri net point of view, even though the programmer may have a very good understanding of how the inputs types map to the output types of a place.

An Example
^^^^^^^^^^

Lets look at an example rendering from a petri net with a place that has multiple input and output types:

A petri net as a graph (in its initial state, Mult is current computing 4 tokens concurrently):

.. graphviz:: ./multi_type.dot

A petri net as a function which "places" the elements in an already exiting petri net

.. code-block:: cpp

  namespace petri_net::tests
  {

  void mult(PetriNet& pn)
  {

    pn.AddPlace("Mult", { in_type<std::pair<int, int>>(),
                          in_type<std::pair<int, double>>(),
                          in_type<std::pair<double, int>>(),
                          in_type<std::pair<double, double>>(),
                        });
    pn.SetPlaceCallback("Mult", [&](const Token _in) -> Token {
      auto type = _in->type().hash_code();
      if (typeid(std::pair<int, int>).hash_code() == type)
      {
          auto& ret = token_cast<std::pair<int, int>>(_in);
          return make_token(ret.first * ret.second);
      }
      else if (typeid(std::pair<int, double>).hash_code() == type)
      {
          auto& ret = token_cast<std::pair<int, double>>(_in);
          return make_token(ret.first * ret.second);
      }
      else if (typeid(std::pair<double, int>).hash_code() == type)
      {
          auto& ret = token_cast<std::pair<double, int>>(_in);
          return make_token(ret.first * ret.second);
      }
      else if (typeid(std::pair<double, double>).hash_code() == type)
      {
          auto& ret = token_cast<std::pair<double, double>>(_in);
          return make_token(ret.first * ret.second);
      }
      pn.Kill(std::string("type ") + demangle(_in->type().name()) + std::string(" was not handled\n"));
    });

    pn.ConnectPlaceTransition("Mult", "Return/Transition", typeid(int), 1);
    pn.ConnectPlaceTransition("Mult", "Return/Transition", typeid(double), 3);
  }

Notice that we do not need to include the return transition and return place, they are taken car of for us by the library when ever adding elements to a petri net / sub net.

**NOTE: All this code came from the** ``test/mult.hpp`` **under the tests directry of the mil_petri_nets package**

Action
******
Upon being given a token, a place's callback function is run in its own `thread <https://www.geeksforgeeks.org/thread-in-operating-system/>`_.

All the places of a petri net are run in one process.

**NOTE: If a place is given tokens faster than it can finish it computations, multiple threads belonging to that place will be started.**

Transitions
-----------

Role
****

Transitions serve the role of data mutliplexers.

Structure
*********

A transition has input places, which it will take a specified number of tokens(default to 1) from some number of places.
A transition has output places, which it will give each place exactly one token.

A transition has a collection of functions specifying how to map the collection of input places' tokens to each output place.


Action
******

A transition will only fire when all of its input places have greater than or equal to a specified theshhold.

TODO: Render this threshold.

Upon firing, a transition takes tokens from each input place, and puts one token into each of its out put places.


Visualization
-------------

There is a debug logger built into the library, which logges every state of the petri net (every time the distribution of tokens changes).

To run the render on a log file from the tests:

run the tests with:

``mil && cd ../.. && catkin_make run_tests_mil_petri_nets_gtest``

Then, the logs will have been generated.

To render the log file, from the multi_type test, run.

``rosrun mil_petri_nets render_log.py ~/.mil/petri_net_tests/multi_type.log``

This should bring up a rendering of the mult petri net (that was demonstrated earlier). You can advance the state in the renderer by pressing the ``Enter`` key on your keybard in the terminal where you are running the renderer script.

TODO: Make a real time renderer.
