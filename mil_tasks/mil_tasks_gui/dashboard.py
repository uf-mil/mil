#!/usr/bin/env python
import os
import threading
from mil_tools import thread_lock
from ros_alarms import AlarmListener
from navigator_msgs.msg import Hosts, Host
from python_qt_binding import QtCore, QtWidgets, loadUi
from qt_gui.plugin import Plugin
from remote_control_lib import RemoteControl
import rospkg
import rospy
from copy import copy
from std_msgs.msg import Float32, String, Header
from mil_tasks.msg import DoTaskAction, DoTaskGoal
from actionlib_msgs.msg import GoalID, GoalStatus
from actionlib import ActionClient, GoalManager, CommStateMachine, ClientGoalHandle, TerminalState
import json
import weakref
import datetime


class ExtendedGoalManager(GoalManager):
    def init_observe_goal(self, action_goal, transition_cb=None, feedback_cb=None):
        csm = CommStateMachine(action_goal, transition_cb, feedback_cb,
                               self.send_goal_fn, self.cancel_fn)
        with self.list_mutex:
            self.statuses.append(weakref.ref(csm))
        return csm


class ObserveActionClient(ActionClient):
    g_goal_id = 0
    def __init__(self, ns, ActionSpec):
        ActionClient.__init__(self, ns, ActionSpec)
        self.manager = ExtendedGoalManager(ActionSpec)
        self.manager.register_send_goal_fn(self.pub_goal.publish)
        self.manager.register_cancel_fn(self.pub_cancel.publish)
        self.observe_goals = {}
        self.observer_transition_cb = None
        self.observer_feedback_cb = None
        self.goal_sub = rospy.Subscriber(rospy.remap_name(ns) + '/goal', self.ActionGoal,
                                         callback=self.goal_cb, queue_size=5)

    def register_observer_transition_cb(self, fn):
        self.observer_transition_cb = fn

    def register_observer_feedback_cb(self, fn):
        self.observer_feedback_cb = fn

    def _observer_transition_cb(self, goal, handler):
        # TODO: delete from self.observe_goals when very old.... shouldnt really be a problem though
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
        action_goal.goal_id.id = "%s-%i-%.3f" % (rospy.get_caller_id(), id, now.to_sec())
        action_goal.header.stamp = now
        self.pub_goal.publish(action_goal)  # Should still be added through goal_cb

    def goal_cb(self, msg):
        self.observe_goals[msg.goal_id.id] = self.manager.init_observe_goal(msg,
                transition_cb = lambda msg, _goal=msg.goal_id: self._observer_transition_cb(_goal, msg),
                feedback_cb = lambda handler, feedback,  _goal=msg.goal_id: self._observer_feedback_cb(_goal, handler, feedback))


class Dashboard(Plugin):
    def __init__(self, context):
        super(Dashboard, self).__init__(context)

        # Create the widget and name it
        self._widget = QtWidgets.QWidget()
        self._widget.setObjectName("Dashboard")
        self.setObjectName("Dashboard")

        # Extend the widget with all attributes and children in the UI file
        ui_file = os.path.join(rospkg.RosPack().get_path("mil_tasks"), "resource", "dashboard.ui")
        loadUi(ui_file, self._widget)

        self.task_runner_client = ObserveActionClient('/task', DoTaskAction)
        self.task_runner_client.register_observer_transition_cb(self.transition_cb)
        self.task_runner_client.register_observer_feedback_cb(self.feedback_cb)

        self.current_mission = None
        self.current_mission_status = ''
        self.current_mission_task = ''
        self.current_mission_result = ''

        self.connect_ui()
        self.reload_available_missions(None)

        # Deals with problem when they're multiple instances of Dashboard plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (" (%d)" % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)


    def ui_log(self, string):
        time = rospy.Time.now()
        date_time = datetime.datetime.fromtimestamp(rospy.Time.now().to_time())
        time_str = '{}:{}:{}'.format(date_time.hour, date_time.minute, date_time.second).ljust(12, ' ')
        formatted = time_str + string
        self.feedback_list.insertItem(0, formatted)

    def feedback_cb(self, goal, handler, feedback):
        self.ui_log('FEEDBACK: ' + feedback.message)

    def transition_cb(self, goal, handler):
        status = handler.get_goal_status()
        if status == GoalStatus.ACTIVE:
            if self.current_mission and goal.id != self.current_mission:
                pass
                # Another active goal happened, clear Feedback and stuff
            self.result_label.clear()
            self.current_mission = goal.id
            self.current_mission_task = handler.comm_state_machine.action_goal.goal.task
            self.current_mission_status = 'In progress'
            self.current_mission_status_label.setText(self.current_mission_status)
            self.current_mission_result = ''
            self.current_mission_label.setText(self.current_mission_task)
            self.ui_log('STARTING: new task {}'.format(self.current_mission_task))
        if goal.id == self.current_mission:
            terminal_state = TerminalState.to_string(status)
            if terminal_state == 'NO_SUCH_STATE_1':
                return
            result = handler.get_result()
            if result and result.result != self.current_mission_result:
                self.result_label.setText(result.result)
                self.ui_log('RESULT: {}'.format(result.result))
            if terminal_state != self.current_mission_status:
                self.current_mission_status = terminal_state
                self.current_mission_status_label.setText(self.current_mission_status)
                self.ui_log('FINISHED: task finished ({})'.format(self.current_mission_status))

    def reload_available_missions(self, _):
        if not rospy.has_param('/available_tasks'):
            self.ui_log('ERROR: /available_tasks param not set. Perhaps task runner is not running?')
            return
        self.missions = rospy.get_param('/available_tasks')
        self.available_missions_list.clear()
        for i in reversed(range(self.chained_missions_table.rowCount())):
            self.chained_missions_table.removeRow(i)
        for i, mission in enumerate(self.missions):
           self.available_missions_list.insertItem(i, mission)
        return True

    def clear_log(self, event):
        self.feedback_list.clear()
        return True

    def chained_missions_drop_cb(self, event):
        idx = self.chained_missions_table.indexAt(event.pos()).row()
        if idx == -1:
            idx = self.chained_missions_table.rowCount()
        if event.source() == self.chained_missions_table:
           # Now swap the two rows
            selected_index = self.chained_missions_table.selectedIndexes()[0].row()
            if idx == selected_index:
                return
            self.chained_missions_table.insertRow(idx)
            if selected_index > idx:
                selected_index += 1
            for i in range(self.chained_missions_table.columnCount()):
                self.chained_missions_table.setCellWidget(idx, i, self.chained_missions_table.cellWidget(selected_index, i))
            self.chained_missions_table.removeRow(selected_index)
        elif event.source() == self.available_missions_list:
            selected_item = self.available_missions_list.selectedItems()[0]
            mission = QtWidgets.QLabel(selected_item.text())
            timeout = QtWidgets.QDoubleSpinBox()
            timeout.setValue(0)
            timeout.setMaximum(10000)
            timeout.setSuffix('s')
            #required = QtWidgets.QCheckBox()
            #required.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
            parameters = QtWidgets.QLineEdit('')
            self.chained_missions_table.insertRow(idx)
            self.chained_missions_table.setCellWidget(idx, 0, mission)
            self.chained_missions_table.setCellWidget(idx, 1, timeout)
            #self.chained_missions_table.setCellWidget(idx, 2, required)
            #self.chained_missions_table.setCellWidget(idx, 3, parameters)
            self.chained_missions_table.setCellWidget(idx, 2, parameters)
        else:
            print 'hmmmmmm table', event.source(), self.chained_missions_table

    def available_missions_drop_cb(self, event):
        if event.source() == self.chained_missions_table:  # If dragged from table, delete from table
            selected_index = self.chained_missions_table.selectedIndexes()[0].row()
            self.chained_missions_table.removeRow(selected_index)

    def run_chained_cb(self, event):
        #TODO: deal with possibility of table being changed in this loop, perhaps by freezing it
        tasks = []
        for i in range(self.chained_missions_table.rowCount()):
            task = self.chained_missions_table.cellWidget(i, 0).text()
            timeout = self.chained_missions_table.cellWidget(i, 1).value()
            parameters = self.chained_missions_table.cellWidget(i, 2).text()
            try:
                parameters = json.loads(parameters)
            except ValueError:
                pass
            tasks.append({'task': task, 'timeout': timeout, 'parameters': parameters})
        goal_parameters = json.dumps(tasks)
        goal = DoTaskGoal(task='ChainWithTimeout', parameters=goal_parameters)
        self.task_runner_client.send_goal(goal)
        return True

    def run_single_cb(self, event):
        selected = self.available_missions_list.selectedItems()
        if len(selected) == 0:
            self.ui_log('ERROR: tried to run single mission with none selected')
            return False
        task = selected[0].text()
        if task not in self.missions:
            print 'Unavailable task selected somehow, ignoring'
            return
        parameters = self.single_mission_parameters.text()
        goal = DoTaskGoal(task=task, parameters=parameters)
        self.task_runner_client.send_goal(goal)
        return True

    def cancel_mission_cb(self, event):
        self.task_runner_client.cancel_all_goals()
        return True

    def connect_ui(self):
        '''
        Stores various interactive widgets as member variabes so we can get and set their contents.
        '''
        self.chained_missions_table = self._widget.findChild(QtWidgets.QFrame, 'chained_missions_table')
        self.chained_missions_table.setDragDropMode(QtWidgets.QAbstractItemView.DragDrop)
        self.chained_missions_table.dropEvent = self.chained_missions_drop_cb
        self.available_missions_list = self._widget.findChild(QtWidgets.QFrame, 'available_missions')
        self.available_missions_list.setDragEnabled(True)
        self.available_missions_list.setDragDropMode(QtWidgets.QAbstractItemView.DragDrop)
        self.available_missions_list.dropEvent = self.available_missions_drop_cb
        self.run_chained_button = self._widget.findChild(QtWidgets.QPushButton, 'run_chained_button')
        self.run_chained_button.clicked.connect(self.run_chained_cb)
        self.single_mission_button = self._widget.findChild(QtWidgets.QPushButton, 'single_mission_button')
        self.single_mission_button.clicked.connect(self.run_single_cb)
        self.cancel_button = self._widget.findChild(QtWidgets.QPushButton, 'cancel_mission_button')
        self.cancel_button.clicked.connect(self.cancel_mission_cb)
        self.single_mission_parameters = self._widget.findChild(QtWidgets.QLineEdit, 'single_mission_parameters')
        self.current_mission_label = self._widget.findChild(QtWidgets.QLabel, 'current_task_label')
        self.current_mission_status_label = self._widget.findChild(QtWidgets.QLabel, 'current_status_label')
        self.feedback_list = self._widget.findChild(QtWidgets.QListWidget, 'log_list')
        self.result_label = self._widget.findChild(QtWidgets.QLabel, 'result_label')
        self.clear_log_button = self._widget.findChild(QtWidgets.QPushButton, 'clear_log_button')
        self.clear_log_button.clicked.connect(self.clear_log)
        self.refresh_missions_button = self._widget.findChild(QtWidgets.QPushButton, 'refresh_missions_button')
        self.refresh_missions_button.clicked.connect(self.reload_available_missions)

