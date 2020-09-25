Basic Petri Net Theory
======================

What is a Petri Net Exactly?
----------------------------
Petri Nets are directed bipartite graphs (`graphs <https://en.wikipedia.org/wiki/Graph_(discrete_mathematics)>`_ which are `directed <https://en.wikipedia.org/wiki/Directed_graph>`_ and `bipartite <https://en.wikipedia.org/wiki/Bipartite_graph>`_).

The two types of nodes are **Transitions** and **Places**.

Places
******

Places are usually represented with circles or ovals.

Transitions
***********

Transitions are usually represented with boxes or straight lines.


A simple Petri Net Example
**************************

.. graphviz:: ./simple_petri_net.dot

What Makes Petri Nets Useful?
-----------------------------

What makes Petri Nets a useful modeling language are **Tokens** and the rules which govern them.

What is a Token?
****************

Tokens exist inside of places and are passed from one place to another via transition(s).

Tokens are usually represented with a dot inside of a place.

What Are The Rules For Receiving / Distributing Tokens?
*******************************************************

Transitions are responsible for distributing tokens.

A Transition distributes tokens when it **Fires**.

What Happens When A Transition Fires?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* It **Removes** one token from each in Place (places with a connecting edge that points to the transition that is firing).

* It **Inserts** one token into each out Place (places with a connecting edge that points to the transition that is firing).

**NOTE: There is no conservation of Tokens**

When Does a Transition Fire?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Transitions will fire **if and only if** all the input places have at lease one token.


What Does it Look Like When a Transition Fires?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Lets use our example from above.

Here is a Petri Net with tokens in ``My Place`` and ``My Second Place``.

.. graphviz:: ./simple_petri_net_token_1.dot

In this case, the transition ``MyTransition`` will fire.

This will:

* Remove a single token from ``My Place`` and ``My Second Place`` each.

* Insert a single Token into ``My Third Place``.

Here is what the Petri Net looks like after ``My Transition`` Fires.

.. graphviz:: ./simple_petri_net_token_2.dot


More Advanced Petri Net Theory
------------------------------

It is recommended that you watch these `Free Tutorials Covering Petri Net Theory <https://www.youtube.com/watch?v=uL1iwZiqLo4&list=PLeqHS5WpW-JUJFnolVVDdQrnUxKuYEJ5i>`_ in order to have acomplete understanding of petri nets before you proceed to write your own.
