import os
import rospy
import rospkg

import functools
import datetime
import json
from sub8_msgs.msg import Alarm
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import python_qt_binding.QtGui as qtg
from python_qt_binding.QtCore import Qt


def newTableWidgetItem(contents):
    '''Make a new table widget item, and do some additional bookkeeping'''
    new_item = qtg.QTableWidgetItem(str(contents))
    new_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
    return new_item


def add_all_children(widget, elements):
    '''Helper function for pretty-printing json'''
    for element in elements:
        widget.addChild(element)


def recurse_on_tree(tree):
    '''Helper function for pretty-printing json'''
    children = []
    if tree is None:
        children.append(
            qtg.QTreeWidgetItem(["N/A"])
        )

    elif isinstance(tree, (str, int, float, unicode)):
        children.append(
            qtg.QTreeWidgetItem([str(tree)])
        )
    elif isinstance(tree, list):
        for item in tree:
            children.append(
                qtg.QTreeWidgetItem([str(item)])
            )

    elif isinstance(tree, dict):
        for name, item in tree.items():
            new_item = qtg.QTreeWidgetItem([name])
            items_children = recurse_on_tree(item)
            add_all_children(new_item, items_children)
            children.append(new_item)
    else:
        print "Encountered unparseable json entity of type {}".format(type(tree))
    return children


def build_tree_from_json(tree, json_hierarchy):
    '''Helper function for pretty-printing json'''
    root_tree = tree
    if not isinstance(json_hierarchy, dict):
        print "Couldn't render non-dict properly"
        return

    for name, item in json_hierarchy.items():
        table_item = qtg.QTreeWidgetItem([name])
        root_tree.addTopLevelItem(table_item)
        children = recurse_on_tree(item)
        add_all_children(table_item, children)
        tree.expandItem(table_item)


class AlarmPlugin(Plugin):

    _severity_mapping = {
        0: qtg.QColor(255, 0, 0),
        1: qtg.QColor(240, 100, 0),
        2: qtg.QColor(220, 200, 0),
        3: qtg.QColor(30, 255, 30),
    }
    _column_headers = ["Name", "Status", "Description", "Time Recieved"]

    def __init__(self, context):
        super(AlarmPlugin, self).__init__(context)

        self.setObjectName('AlarmPlugin')

        # Things seem to misbehave when this is missing
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('sub8_rqt'), 'resource', 'sub8_rqt_alarms.ui')

        # Extend the widget with all attributes and children from UI file
        self.ui = loadUi(ui_file, self._widget)

        self._widget.setObjectName('AlarmPlugin')
        self.alarm_tree = self._widget.findChild(qtg.QTreeWidget, 'alarm_tree')

        self.alarm_table = self._widget.findChild(qtg.QTableWidget, 'alarm_table')

        # Alarm parameter cache stores some additional information about the alarm
        # (for use when an alarm is clicked)
        self.alarm_parameter_cache = {}

        # Default row-count to 0
        self.alarm_table.setRowCount(0)
        self.alarm_table.setColumnCount(len(self._column_headers))
        self.alarm_table.setHorizontalHeaderLabels(self._column_headers)
        self.alarm_table.clicked.connect(self.open_alarm)

        # ---- ROS ----
        self.alarm_sub = rospy.Subscriber('alarm', Alarm, self.new_alarm_callback)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def new_alarm_callback(self, msg):
        '''React to a new alarm'''
        columns = [
            'alarm_name',
            'severity',
            'problem_description'
        ]
        row_elements = map(functools.partial(getattr, msg), columns)

        # Get pub time
        alarm_epoch = msg.header.stamp.to_time()
        formatted_time = datetime.datetime.fromtimestamp(alarm_epoch).strftime('%I:%M:%S.%f')
        row_elements.append(formatted_time)
        color = self._severity_mapping[msg.severity]
        self.alarm_table.insertRow(0)
        first_col = self.set_row(0, row_elements, color)
        self.alarm_parameter_cache[first_col] = msg.parameters

    def set_row(self, row, data, color):
        '''Set a whole row in the alarm table'''
        assert isinstance(data, list), "data must be a list"
        to_return = None
        for col, element in enumerate(data):
            table_item = newTableWidgetItem(element)
            table_item.setBackground(color)
            self.alarm_table.setItem(row, col, table_item)
            if col == 0:
                to_return = table_item
        return to_return

    def open_alarm(self, event):
        '''React when an alarm has been clicked
            1. Use event to determine which cell has been clicked
            2. Select that cell's whole row
            3. Use the cached json_descriptions of each alarm to get that row's json data

        When an alarm has been clicked, we'll display its json parameters in the box on the right
        '''
        items_selected = self.alarm_table.selectedItems()
        if len(items_selected) == 0:
            return
        row_selected = items_selected[0].row()
        self.alarm_table.selectRow(row_selected)
        key_item = self.alarm_table.itemAt(row_selected, 0)
        try:
            json_parameters = json.loads(self.alarm_parameter_cache[key_item])
        except ValueError:
            rospy.logwarn("Could not decode alarm message")
            return
        self.alarm_tree.clear()
        try:
            build_tree_from_json(self.alarm_tree, json_parameters)
        except AssertionError:
            rospy.logwarn("Could not draw json tree for alarm (Is it a dictionary?)")

    def shutdown_plugin(self):
        '''Unregister our subsribers'''
        self.alarm_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        rospy.logwarn('Saving does not currently do anything')
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        rospy.logwarn('Restoring does not currently do anything')
        pass
