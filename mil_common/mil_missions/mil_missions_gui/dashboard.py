#!/usr/bin/env python3
import datetime
import json
import os
import threading
import weakref

import rospkg
import rospy
from actionlib import ActionClient, CommStateMachine, GoalManager, TerminalState
from actionlib_msgs.msg import GoalID, GoalStatus
from mil_missions.msg import DoMissionAction, DoMissionGoal
from python_qt_binding import QtCore, QtWidgets, loadUi
from qt_gui.plugin import Plugin


class ExtendedGoalManager(GoalManager):
    """
    Extends actionlib's goal manager with another function allowing monitoring
    a goal that did not originate in this node.
    """

    def init_observe_goal(self, action_goal, transition_cb=None, feedback_cb=None):
        csm = CommStateMachine(
            action_goal,
            transition_cb,
            feedback_cb,
            self.send_goal_fn,
            self.cancel_fn,
        )
        with self.list_mutex:
            self.statuses.append(weakref.ref(csm))
        return csm


class ObserveActionClient(ActionClient):
    """
    Extension of actionlib's actionclient which will also
    notice goals coming from other clients by subscribing to the goal topic.
    Used in the GUI to monitor the current mission even when it was triggered
    by a different client.
    """

    g_goal_id = 0

    def __init__(self, ns, ActionSpec):
        ActionClient.__init__(self, ns, ActionSpec)
        self.manager = ExtendedGoalManager(ActionSpec)
        self.manager.register_send_goal_fn(self.pub_goal.publish)
        self.manager.register_cancel_fn(self.pub_cancel.publish)
        self.observe_goals = {}
        self.observer_transition_cb = None
        self.observer_feedback_cb = None
        self.goal_sub = rospy.Subscriber(
            rospy.remap_name(ns) + "/goal",
            self.ActionGoal,
            callback=self.goal_cb,
            queue_size=5,
        )

    def register_observer_transition_cb(self, fn):
        self.observer_transition_cb = fn

    def register_observer_feedback_cb(self, fn):
        self.observer_feedback_cb = fn

    def _observer_transition_cb(self, goal, handler):
        # TODO: delete from self.observe_goals when very old.... shouldn't really be a problem though
        if self.observer_transition_cb:
            self.observer_transition_cb(goal, handler)

    def _observer_feedback_cb(self, goal, handler, feedback):
        if self.observer_feedback_cb:
            self.observer_feedback_cb(goal, handler, feedback)

    def send_goal(self, goal):
        now = rospy.get_rostime()
        action_goal = self.ActionGoal()
        action_goal.goal = goal
        action_goal.goal_id = GoalID()
        action_goal.goal_id.stamp = now
        id, self.g_goal_id = self.g_goal_id, self.g_goal_id + 1
        action_goal.goal_id.id = "%s-%i-%.3f" % (
            rospy.get_caller_id(),
            id,
            now.to_sec(),
        )
        action_goal.header.stamp = now
        self.pub_goal.publish(action_goal)  # Should still be added through goal_cb

    def goal_cb(self, msg):
        self.observe_goals[msg.goal_id.id] = self.manager.init_observe_goal(
            msg,
            transition_cb=lambda msg, _goal=msg.goal_id: self._observer_transition_cb(
                _goal,
                msg,
            ),
            feedback_cb=lambda handler, feedback, _goal=msg.goal_id: self._observer_feedback_cb(
                _goal,
                handler,
                feedback,
            ),
        )


class CenteredCheckBox(QtWidgets.QWidget):
    """
    Wrapper around a checkbox widget that properly displays it in the center
    of the parent widget, which is surprisingly complicated to do. Useful
    for putting a checkbox in a table item.
    """

    def __init__(self):
        super().__init__()
        checkbox = QtWidgets.QCheckBox()
        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(checkbox)
        layout.setAlignment(QtCore.Qt.AlignCenter)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

    def checkState(self):
        return self.checkbox().checkState()

    def setCheckState(self, state):
        return self.checkbox().setCheckState(state)

    def checked(self):
        """return boolean of if box is checked"""
        return self.checkState() == QtCore.Qt.Checked

    def setChecked(self, checked):
        """set checkbox to boolean checked or not"""
        state = QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked
        return self.setCheckState(state)

    def checkbox(self):
        """returns internal checkbox widget for custom usage"""
        return self.layout().itemAt(0).widget()


class Dashboard(Plugin):
    """
    RQT plugin to interface with a mil_missions server, allowing the user to graphicly monitor
    the current mission and result, cancel the current mission, run a mission with optional parameters,
    and create a linear chain of missions.
    """

    def __init__(self, context):
        super().__init__(context)

        # Lock used to ensure ROS callbacks are synced with Qt
        self.lock = threading.Lock()

        # Create the widget and name it
        self._widget = QtWidgets.QWidget()
        self._widget.setObjectName("Dashboard")
        self.setObjectName("Dashboard")

        # Extend the widget with all attributes and children in the UI file
        ui_file = os.path.join(
            rospkg.RosPack().get_path("mil_missions"),
            "resource",
            "dashboard.ui",
        )
        loadUi(ui_file, self._widget)

        self.mission_runner_client = ObserveActionClient("/mission", DoMissionAction)
        self.mission_runner_client.register_observer_transition_cb(self.transition_cb)
        self.mission_runner_client.register_observer_feedback_cb(self.feedback_cb)

        self.current_mission = None
        self.current_mission_status = ""
        self.current_mission_mission = ""
        self.current_mission_result = ""

        self.connect_ui()
        self.reload_available_missions(None)

        # Deals with problem when they're multiple instances of Dashboard plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number()),
            )

        # Add widget to the user interface
        context.add_widget(self._widget)

    def ui_log(self, string):
        """
        Adds a new line to the log pane with the current time and specified string.

        ex: ui_log('Hello world')
        12:25       Hello world
        """
        self.lock.acquire()
        date_time = datetime.datetime.fromtimestamp(rospy.Time.now().to_time())
        time_str = "{}:{}:{}".format(
            date_time.hour,
            date_time.minute,
            date_time.second,
        ).ljust(12, " ")
        formatted = time_str + string
        self.feedback_list.addItem(formatted)
        self.lock.release()

    def feedback_cb(self, goal, handler, feedback):
        """
        Add feedback from missions to the log
        """
        self.ui_log("FEEDBACK: " + feedback.message)

    def transition_cb(self, goal, handler):
        """
        When a mission "transitions" (canceled, started, aborted, succeeded), update
        the log and result pane
        """
        status = handler.get_goal_status()
        if status == GoalStatus.ACTIVE:
            if self.current_mission and goal.id != self.current_mission:
                pass
                # Another active goal happened, clear Feedback and stuff
            self.result_label.clear()
            self.current_mission = goal.id
            self.current_mission_mission = (
                handler.comm_state_machine.action_goal.goal.mission
            )
            self.current_mission_status = "In progress"
            self.current_mission_status_label.setText(self.current_mission_status)
            self.current_mission_result = ""
            self.current_mission_label.setText(self.current_mission_mission)
            self.ui_log(f"STARTING: new mission {self.current_mission_mission}")
        if goal.id == self.current_mission:
            terminal_state = TerminalState.to_string(status)
            if terminal_state == "NO_SUCH_STATE_1":
                return
            result = handler.get_result()
            if result and result.result != self.current_mission_result:
                self.result_label.setText(result.result)
                self.ui_log(f"RESULT: {result.result}")
            if terminal_state != self.current_mission_status:
                self.current_mission_status = terminal_state
                self.current_mission_status_label.setText(self.current_mission_status)
                self.ui_log(
                    "FINISHED: mission finished ({})".format(
                        self.current_mission_status,
                    ),
                )

    def reload_available_missions(self, _):
        """
        Load available missions from the ROS param set by the mission server. Called when the refresh
        button is hit and once on startup. Also clears the chained pane as it may now be invalid.
        """
        if not rospy.has_param(
            "/available_missions",
        ):  # If the param is not there, log this
            self.ui_log(
                "ERROR: /available_missions param not set. Perhaps mission runner is not running?",
            )
            return
        self.missions = rospy.get_param("/available_missions")
        self.missions = sorted(
            self.missions,
        )  # Ensure missions appear in lexographic order
        self.available_missions_list.clear()
        for i in reversed(range(self.chained_missions_table.rowCount())):
            self.chained_missions_table.removeRow(i)
        for i, mission in enumerate(self.missions):
            self.available_missions_list.insertItem(i, mission)
        return True

    def clear_log(self, event):
        """
        Clear the log pane when the button is pressed.
        """
        self.feedback_list.clear()
        return True

    def chained_missions_drop_cb(self, event):
        """
        Called when a mission is dropped from the available_missions_list to the chained_missions_table,
        or when a mission is moved (reordered) within the chained_missions_table.
        """
        idx = self.chained_missions_table.indexAt(event.pos()).row()
        if idx == -1:  # Handle insertion at end of table
            idx = self.chained_missions_table.rowCount()

        # If drop is from itself, do a reorder
        if event.source() == self.chained_missions_table:
            # Now swap the two rows
            selected_index = self.chained_missions_table.selectedIndexes()[0].row()
            if idx == selected_index:
                return
            self.chained_missions_table.insertRow(idx)
            if selected_index > idx:
                selected_index += 1
            for i in range(self.chained_missions_table.columnCount()):
                self.chained_missions_table.setCellWidget(
                    idx,
                    i,
                    self.chained_missions_table.cellWidget(selected_index, i),
                )
            self.chained_missions_table.removeRow(selected_index)

        # If drop is from available list, insert it at the dropped row
        elif event.source() == self.available_missions_list:
            selected_item = self.available_missions_list.selectedItems()[0]
            mission = QtWidgets.QLabel(selected_item.text())
            required = CenteredCheckBox()
            required.setChecked(True)  # Start with default required
            timeout = QtWidgets.QDoubleSpinBox()
            timeout.setValue(0)
            timeout.setMaximum(10000)
            timeout.setSuffix("s")
            parameters = QtWidgets.QLineEdit("")
            self.chained_missions_table.insertRow(idx)
            self.chained_missions_table.setCellWidget(idx, 0, mission)
            self.chained_missions_table.setCellWidget(idx, 1, required)
            self.chained_missions_table.setCellWidget(idx, 2, timeout)
            self.chained_missions_table.setCellWidget(idx, 3, parameters)

    def available_missions_drop_cb(self, event):
        """
        Handles drag and drops from the chained mission table to the available missions pane,
        which should just delete it from the table.
        """
        if (
            event.source() == self.chained_missions_table
        ):  # If dragged from table, delete from table
            selected_index = self.chained_missions_table.selectedIndexes()[0].row()
            self.chained_missions_table.removeRow(selected_index)

    def get_chained_missions(self):
        missions = []
        for i in range(self.chained_missions_table.rowCount()):
            mission = self.chained_missions_table.cellWidget(i, 0).text()
            required = self.chained_missions_table.cellWidget(i, 1).checked()
            timeout = self.chained_missions_table.cellWidget(i, 2).value()
            parameters = self.chained_missions_table.cellWidget(i, 3).text()
            missions.append(
                {
                    "mission": mission,
                    "timeout": timeout,
                    "required": required,
                    "parameters": parameters,
                },
            )
        return missions

    def load_chained_missions(self, list_of_missions):
        for i in reversed(range(self.chained_missions_table.rowCount())):
            self.chained_missions_table.removeRow(i)
        for idx, m in enumerate(list_of_missions):
            mission = m["mission"]
            timeout = m["timeout"]
            required = m["required"]
            parameters = m["parameters"]
            mission = QtWidgets.QLabel(m["mission"])
            required = CenteredCheckBox()
            required.setChecked(m["required"])  # Start with default required
            timeout = QtWidgets.QDoubleSpinBox()
            timeout.setValue(m["timeout"])
            timeout.setMaximum(10000)
            timeout.setSuffix("s")
            parameters = QtWidgets.QLineEdit(m["parameters"])
            self.chained_missions_table.insertRow(idx)
            self.chained_missions_table.setCellWidget(idx, 0, mission)
            self.chained_missions_table.setCellWidget(idx, 1, required)
            self.chained_missions_table.setCellWidget(idx, 2, timeout)
            self.chained_missions_table.setCellWidget(idx, 3, parameters)

    def run_chained_cb(self, event):
        """
        When the run chained button is pressed, parse the table contents as the parameters
        to the ChainedWithTimeouts mission and send this goal to the missions server.

        TODO: deal with possibility of table being changed in this loop, perhaps by freezing it
        """
        missions = self.get_chained_missions()
        goal_parameters = json.dumps({"missions": missions})
        goal = DoMissionGoal(mission="ChainWithTimeout", parameters=goal_parameters)
        self.mission_runner_client.send_goal(goal)
        return True

    def run_single_cb(self, event):
        """
        When the run mission button is pressed, run the selected mission in the available mission pane
        with the parameters in the textbox.
        """
        selected = self.available_missions_list.selectedItems()
        if len(selected) == 0:
            self.ui_log("ERROR: tried to run single mission with none selected")
            return False
        mission = selected[0].text()
        if mission not in self.missions:
            print("Unavailable mission selected somehow, ignoring")
            return
        parameters = self.single_mission_parameters.text()
        goal = DoMissionGoal(mission=mission, parameters=parameters)
        self.mission_runner_client.send_goal(goal)
        return True

    def cancel_mission_cb(self, event):
        """
        When the cancel button is pressed, send a goal to the mission server to cancel the current mission.
        """
        self.mission_runner_client.cancel_all_goals()
        return True

    def autoscroll(self, *args):
        auto = (
            self.feedback_list_scrollbar.value()
            == self.feedback_list_scrollbar.maximum()
        )
        if auto:  # Autoscroll to bottom if you were already there
            self.feedback_list.scrollToBottom()

    def load_file(self, event):
        name = QtWidgets.QFileDialog.getOpenFileName(self._widget, "Open File")[0]
        try:
            f = open(name)
        except OSError as e:
            rospy.logwarn(f"Error loading configuration from file: {e}")
            return
        with f:
            missions = json.load(f)
            self.load_chained_missions(missions)

    def save_file(self, event):
        name = QtWidgets.QFileDialog.getSaveFileName(self._widget, "Save File")[0]
        try:
            f = open(name, "w")
        except OSError as e:
            rospy.logwarn(f"Error saving configuration to file: {e}")
            return
        with f:
            missions = self.get_chained_missions()
            json.dump(missions, f)

    def connect_ui(self):
        """
        Stores various interactive widgets as member variables so we can get and set their contents.
        """
        self.save_button = self._widget.findChild(QtWidgets.QToolButton, "save_button")
        self.load_button = self._widget.findChild(QtWidgets.QToolButton, "load_button")
        self.save_button.setIcon(
            self._widget.style().standardIcon(QtWidgets.QStyle.SP_DialogSaveButton),
        )
        self.load_button.setIcon(
            self._widget.style().standardIcon(QtWidgets.QStyle.SP_DialogOpenButton),
        )
        self.save_button.clicked.connect(self.save_file)
        self.load_button.clicked.connect(self.load_file)

        self.chained_missions_table = self._widget.findChild(
            QtWidgets.QFrame,
            "chained_missions_table",
        )
        self.chained_missions_table.setDragDropMode(
            QtWidgets.QAbstractItemView.DragDrop,
        )
        self.chained_missions_table.dropEvent = self.chained_missions_drop_cb
        self.chained_missions_table.setColumnWidth(
            1,
            55,
        )  # Make required header just big enough for check box
        self.available_missions_list = self._widget.findChild(
            QtWidgets.QFrame,
            "available_missions",
        )
        self.available_missions_list.setDragEnabled(True)
        self.available_missions_list.setDragDropMode(
            QtWidgets.QAbstractItemView.DragDrop,
        )
        self.available_missions_list.dropEvent = self.available_missions_drop_cb
        self.run_chained_button = self._widget.findChild(
            QtWidgets.QPushButton,
            "run_chained_button",
        )
        self.run_chained_button.clicked.connect(self.run_chained_cb)
        self.single_mission_button = self._widget.findChild(
            QtWidgets.QPushButton,
            "single_mission_button",
        )
        self.single_mission_button.clicked.connect(self.run_single_cb)
        self.cancel_button = self._widget.findChild(
            QtWidgets.QPushButton,
            "cancel_mission_button",
        )
        self.cancel_button.clicked.connect(self.cancel_mission_cb)
        self.single_mission_parameters = self._widget.findChild(
            QtWidgets.QLineEdit,
            "single_mission_parameters",
        )
        self.current_mission_label = self._widget.findChild(
            QtWidgets.QLabel,
            "current_mission_label",
        )
        self.current_mission_status_label = self._widget.findChild(
            QtWidgets.QLabel,
            "current_status_label",
        )
        self.feedback_list = self._widget.findChild(QtWidgets.QListWidget, "log_list")
        self.feedback_list.setAlternatingRowColors(True)  # easier to read
        self.feedback_list.model().rowsInserted.connect(self.autoscroll)
        self.feedback_list_scrollbar = self.feedback_list.verticalScrollBar()
        self.result_label = self._widget.findChild(QtWidgets.QLabel, "result_label")
        self.clear_log_button = self._widget.findChild(
            QtWidgets.QPushButton,
            "clear_log_button",
        )
        self.clear_log_button.clicked.connect(self.clear_log)
        self.refresh_missions_button = self._widget.findChild(
            QtWidgets.QPushButton,
            "refresh_missions_button",
        )
        self.refresh_missions_button.clicked.connect(self.reload_available_missions)
