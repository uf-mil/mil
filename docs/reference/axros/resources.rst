Managing Resources
^^^^^^^^^^^^^^^^^^
When using resources in axros, you must be mindful of how you start the resource,
and how you clean the resource up. Many times, these actions are not done for you
(to give you more control), and you must be mindful of when you plan to do them
yourself. Doing these actions in the improper order or not doing them at all can result
in resources becoming broken.

For example, to start a publisher, you must first call the :meth:`Publisher.setup`
after initializing the class. Then, you must either shut down the publisher using
:meth:`Publisher.shutdown` or shut down the entire node handle using :meth:`NodeHandle.shutdown` - this
will shutdown all publishers spawned by the node.

Furthermore, attempting to shut down a resource more than once will result in an
instance of :class:`RuntimeError` being raised.
