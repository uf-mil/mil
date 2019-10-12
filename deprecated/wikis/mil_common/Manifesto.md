This is the mantra of the MIL software team.


* Don't be stubborn, but *do* be very skeptical

* Do not fear complexity

    * But at the same time, don't introduce complexity for no reason

* Sense of humor is required

* Impatience is unacceptable

* Uncommented code is garbage code

* If you cannot understand someone else's code without consulting them, their code should never have been pulled

* [Meetings should happen as infrequently as possible](http://fortune.com/2015/09/30/workplace-bureaucracy-simple-sabotage/)

* No bullshit tasks for new people

* Work done for free is always open source. No exceptions.

* You shouldn't be restricted to working on things that you understand, but you should *absolutely* understand them once you're done working on them

* Testing code you didn't write should be automatic, and never a nuisance [1]

* If you need to write a small novel justifying your design, something went wrong

* [The Joel Test](http://www.joelonsoftware.com/articles/fog0000000043.html)

# Can I work in MIL?

Yes, if you satisfy any or all of the following:
* I am a very dedicated learner
* I have a background in robotics
* I already know Python and am learning ROS

If you don't, you may be better suited to working on the IEEE Hardware Team until their competition, to develop a strong background before coming into the MIL.


# Footnotes
* *[1]* To test other people's code, you should be able to easily run `catkin_make run_tests`, and immediately see that the integration tests passed. You should not have to follow a page-long instruction list about which nodes to start in which order. In fact, you should need only a *cursory* understanding of the rest of the software stack to test your node. Moreover, it should be possible to see the results of full-simulation integration tests *directly from* continuous integration, when viewing a pull-request.