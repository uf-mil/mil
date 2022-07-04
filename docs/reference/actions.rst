Actions
-------

Path Planner
^^^^^^^^^^^^

MoveAction
~~~~~~~~~~

.. attributetable:: navigator_path_planner.msg.MoveAction

.. class:: navigator_path_planner.msg.MoveAction

    A custom message representing the general movement of an entire system.

    .. attribute:: action_goal
    
        The goal for an action's movement.

        :type: MoveActionGoal

    .. attribute:: action_result
    
        The result of a move action's result.

        :type: MoveActionResult

    .. attribute:: action_feedback
    
        The feedback for an action movement.

        :type: MoveActionFeedback

MoveActionResult
~~~~~~~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg.MoveActionResult

.. class:: navigator_path_planner.msg.MoveActionResult

    A custom message representing the result of a system's movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: status
    
        The status of the system in its movement.

        :type: GoalStatus

    .. attribute:: result
    
        The result of the movement

        :type: MoveResult

MoveActionFeedback
~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg.MoveActionFeedback

.. class:: navigator_path_planner.msg.MoveActionFeedback

    A custom message representing the feedback of a system's movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: status
    
        The status of the system in its movement.

        :type: GoalStatus

    .. attribute:: feedback
    
        The feedback of the movement.

        :type: MoveFeedback

MoveActionGoal
~~~~~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg.MoveActionGoal

.. class:: navigator_path_planner.msg.MoveActionGoal

    A custom message representing the goal of an object's action movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: goal_id
    
        The ID of the goal.

        :type: GoalID

    .. attribute:: goal
    
        The goal to move to.

        :type: MoveGoal

MoveFeedback
~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg.MoveFeedback

.. class:: navigator_path_planner.msg.MoveFeedback

    A custom message representing the feedback of a system's movement.

    .. attribute:: behavior

        A description of the behavior.

        :type: str

    .. attribute:: tree_size
    
        The size of the lqRRT tree.

        :type: int

    .. attribute:: tracking
    
        ???

        :type: bool

    .. attribute:: distance
    
        ???

        :type: List[float]

    .. attribute:: time_till_next_branch
    
        ???

        :type: float

MoveGoal
~~~~~~~~
.. attributetable:: navigator_path_planner.msg.MoveGoal

.. class:: navigator_path_planner.msg.MoveGoal

    A custom message representing the goal of an object's movement.

    .. attribute:: HOLD

        A constant string representing to hold the object's movement. Actually
        equally to ``hold``.

        :type: str

    .. attribute:: DRIVE

        A constant string representing to using driving movement. Actually
        equally to ``drive``.

        :type: str

    .. attribute:: DRIVE_SMOOTH

        A constant string representing to using a smooth driving movement. Actually
        equally to ``drive!``.

        :type: str

    .. attribute:: SKID

        A constant string representing a skidding movement. Actually
        equally to ``skid``.

        :type: str

    .. attribute:: SPIRAL

        A constant string representing a spiral movement. Actually
        equally to ``spiral``.

        :type: str

    .. attribute:: BYPASS

        A constant string representing a spiral movement. Actually
        equally to ``bypass``.

        :type: str

    .. attribute:: move_type

        The type of movement desired, often one of the values above.

        :type: str

    .. attribute:: goal

        The goal to move to.

        :type: Pose

    .. attribute:: focus

        The focal point.

        :type: Point

    .. attribute:: initial_plan_time

        The initial time at which the movement was planned.

        :type: float

    .. attribute:: blind

        ???

        :type: bool

    .. attribute:: speed_factor

        ???

        :type: List[float]

MoveResult
~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg.MoveResult

.. class:: navigator_path_planner.msg.MoveResult

    A custom message representing the goal of an object's movement.

    .. attribute:: failure_reason

        The reason for the movement failing, if any.

        :type: str

Shooter
^^^^^^^

ShooterDoAction
~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg.ShooterDoAction

.. class:: navigator_msgs.msg.ShooterDoAction

    A custom message representing the general movement of an entire system.

    .. attribute:: action_goal
    
        The goal for an action's movement.

        :type: ShooterDoActionGoal

    .. attribute:: action_result
    
        The result of a move action's result.

        :type: ShooterDoActionResult

    .. attribute:: action_feedback
    
        The feedback for an action movement.

        :type: ShooterDoActionFeedback

ShooterDoActionResult
~~~~~~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg.ShooterDoActionResult

.. class:: navigator_msgs.msg.ShooterDoActionResult

    A custom message representing the result of a system's movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: status
    
        The status of the system in its movement.

        :type: GoalStatus

    .. attribute:: result
    
        The result of the movement

        :type: ShooterDoResult

ShooterDoActionFeedback
~~~~~~~~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg.ShooterDoActionFeedback

.. class:: navigator_msgs.msg.ShooterDoActionFeedback

    A custom message representing the feedback of a system's movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: status
    
        The status of the system in its movement.

        :type: GoalStatus

    .. attribute:: feedback
    
        The feedback of the movement.

        :type: ShooterDoFeedback

ShooterDoActionGoal
~~~~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg.ShooterDoActionGoal

.. class:: navigator_msgs.msg.ShooterDoActionGoal

    A custom message representing the goal of an object's action movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: goal_id
    
        The ID of the goal.

        :type: GoalID

    .. attribute:: goal
    
        The goal to move to.

        :type: ShooterDoGoal

ShooterDoFeedback
~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.msg.ShooterDoFeedback

.. class:: navigator_msgs.msg.ShooterDoFeedback

    A custom message representing the feedback of a system's movement.

    .. attribute:: time_remaining

        The amount of time remaining.

        :type: rospy.Duration

ShooterDoGoal
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.msg.ShooterDoGoal

.. class:: navigator_msgs.msg.ShooterDoGoal

    A custom message representing the goal of an object's ShooterDoment. The class
    has no public attributes.

ShooterDoResult
~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.msg.ShooterDoResult

.. class:: navigator_msgs.msg.ShooterDoResult

    A custom message representing the goal of an object's ShooterDoment.

    .. attribute:: ALREADY_RUNNING

        A constant string value to enumerate an error that the shooter is already running.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: NOT_LOADED

        A constant string value to enumerate an error that the shooter is not loaded.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: ALREADY_LOADAED

        A constant string value to enumerate an error that the shooter is already loaded.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: MANUAL_CONTROL_USED

        A constant string value to enumerate an error that the shooter is being controlled manually.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: KILLED

        A constant string value to enumerate an error that the shooter process was killed.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: success

        A constant string value to enumerate an error that the shooter process was successful.

        :type: bool

    .. attribute:: error

        If success was not had, then what went wrong.

        :type: str
