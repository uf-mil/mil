# Getting Help
Programming is hard, and programming at the level needed for autonomous systems
is even harder. Every developer write bugs, encounters confusing errors, and gets stuck
in countless other ways. With time you'll learn not just how to write better code,
but also how to effectively get help when you're stuck.

You should always try to solve problems yourself using the internet, documentation,
etc before asking others. This not only shows respect for your peers' time (who likely
also have their own problems) but will lead you to learn more. In other words,
teach yourself to fish rather than asking for fish.

## Check your sanity
A great first move is to take a step back and think about what you're trying
to do and if it makes sense or if the solution is obvious.

Ask yourself the following questions:

* Are you running the latest code? Have you pulled from git lately (including
submodules) and updated your system packages?
  * Be sure to back up / commit any changes to a branch
  * To reset to the latest code from github run `git fetch origin` and then
  `git checkout origin/master -B master`
  * Be sure to initialize and update all submodules as well with `git submodule update --init --recursive`.
* If the error says exactly how to fix it, have you tried that?
* Is your system setup correctly as described in the [Getting Started Guide](/docs/software/getting_started)?
* Have you blindly copy-pasted/ran things from the internet? If so, do you know
why? Do you know what it's doing?
* Could you be trying to use something that is outdated / deprecated?
* Have you tried simply rebooting your system or running it again?

## Search these docs
You may have noticed that these docs have a search bar. If you want
to learn about a topic, try putting in some keywords related to your topic
or finding a docs page in the table of contents. The search bar will also help
you to find related code to the problem you may be encountering, as it can search
through the entire software reference.

You may find the [Glossary](/docs/glossary) especially useful.

## View the docs on related classes/methods
Although you may be developing a new system/class/method (or not!), finding the
documentation on classes/methods related to what you're building can be very helpful.

For example, let's say you're working with the Passive Sonar system. Try finding some
of the MIL-specific classes/methods you're working with (such as {class}`mil_passive_sonar.TxHydrophonesClient`)
and then viewing the docs on those classes/methods in our reference docs.

## Search the code with `grep`
Perhaps the most useful command for a programmer is `grep`. Grep can search
quickly through files for a particular pattern or string. `git grep` is an
even faster way to search everything in a git repository (such as the MIL repo).

For example, if you see an error `DVL: error on write`, you can run:

    ~/catkin_ws/src/mil$ git grep "DVL: error on write"
    SubjuGator/drivers/sub8_rdi_dvl/include/rdi_explorer_dvl/driver.hpp: \
        ROS_ERROR_THROTTLE(0.5, "DVL: error on write: %s; dropping heartbeat", exc.what());

And see the exact source file where this error comes from.

Likewise, if you want to learn how to use a function, you can search the code
for examples of it being used.

## Search the internet
If your problem is not MIL-specific (issue with Linux, ROS, C++, etc),
somone has most likely had the same problem and written about it on the internet.
You'll be surprised how often you can fix your issue by simply Googling the error.

In fact, often when people come to me with issues I simply Google them myself.

## Search Slack
Someone may have already asked a similar question on Slack. Try searching there.

## Ask for Help
If you made decent efforts to follow the above steps to figure things out on
your own, now it is time to ask for help!

In order for people to be able to understand your issue, please include:

* Precisely what circumstances brought about the error (command, version of software, etc)
* What you are trying to do / achieve
* What steps you have already taken to debug
* The exact code in question (push it to a github branch if it is your changes)

### File an issue
If you believe you have discovered an issue with something that should be working,
it may be appropriate to file an issue [on Github](https://github.com/uf-mil/mil/issues).

### Ask on slack / in-person
If your issue regards code that you are developing or might be specific
to your use case, come by office hours or ask around in the #software channel
on slack.
