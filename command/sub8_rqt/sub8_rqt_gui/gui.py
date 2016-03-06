import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import python_qt_binding.QtGui as qtg


def add_all_children(widget, elements):
    for element in elements:
        widget.addChild(element)


def recurse_on_tree(tree):
    children = []
    print tree
    if isinstance(tree, (str, int, float)):
        print 'base'
        children.append(
            qtg.QTreeWidgetItem([str(tree)])
        )
    elif isinstance(tree, list):
        print '----list'
        for item in tree:
            children.append(
                qtg.QTreeWidgetItem([str(item)])
            )

    elif isinstance(tree, dict):
        print 'dict'
        for name, item in tree.items():
            print 'name:', name
            new_item = qtg.QTreeWidgetItem([name])
            items_children = recurse_on_tree(item)
            add_all_children(new_item, items_children)
            # new_item.setExpanded(True)
            children.append(new_item)
    else:
        print "Encountered unparseable json entity of type {}".format(type(tree))
    return children


def build_tree_from_json(tree, json_hierarchy):
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


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('sub8_rqt'), 'resource', 'sub8_rqt_alarms.ui')
        # Extend the widget with all attributes and children from UI file
        self.ui = loadUi(ui_file, self._widget)
        # print dir(self.ui.alarm_list)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        self.alarm_tree = self._widget.findChild(qtg.QTreeWidget, 'alarm_tree')

        # a = qtg.QTreeWidgetItem('a')
        # p = self.alarm_tree.addTopLevelItems([a, qtg.QTreeWidgetItem('bbbbs')])
        # a.addChild(qtg.QTreeWidgetItem('g'))
        myjson = {
            "widget": {
                "debug": "on",
                "poop": [1, 2, 3],
                "window": {
                    "title": "Sample Konfabulator Widget",
                    "name": "main_window",
                    "width": 500,
                    "height": 500
                }
            }
        }
        build_tree_from_json(self.alarm_tree, myjson)

        self.alarm_table = self._widget.findChild(qtg.QTableWidget, 'alarm_table')

        self.alarm_table.setRowCount(5)
        self.alarm_table.setColumnCount(3)
        self.alarm_table.setHorizontalHeaderLabels(["Name", "Status", "Description"])

        red = qtg.QColor(255, 0, 0)
        new_item = qtg.QTableWidgetItem('Fuck')
        new_item.setBackground(red)
        self.alarm_table.setItem(1, 1, new_item),
        self.alarm_table.clicked.connect(self.banana)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def banana(self, event):
        # print self.alarm_widget.itemClicked
        print self.alarm_widget.currentRow()
        c = qtg.QBrush()
        c.setColor(qtg.QColor(255, 0, 255))
        # self.alarm_widget.currentItem().setBackground(c)

        self.alarm_widget.currentItem().setSelected(1)
        print 'zoop'

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog