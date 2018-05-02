#!/usr/bin/env python

import os
import rospy
import rospkg
import rosgraph.impl.graph

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QGraphicsScene, QWidget

from rqt_graph.interactive_graphics_view import InteractiveGraphicsView
from rqt_py_common.topic_completer import TopicCompleter
from qt_dotgraph.dot_to_qt import DotToQtGenerator

from .btdata import BTData
from .dotcode import RosBTDotcodeGenerator

class BTWidget(QWidget):
    _redraw_interval = 40
    _deferred_fit_in_view = Signal()

    def __init__(self):
        super(BTWidget, self).__init__()

        self.setObjectName('BTWidget')

        self._graph = None
        self._current_dotcode = None
        self._initialized = False

        # generator builds tree graph
        bt_sub_name = '/cyborg/bt/behavior_tree'
        bt_update_sub_name = '/cyborg/bt/behavior_tree_updates'
        self.bt_data = BTData(bt_sub_name, bt_update_sub_name)
        self.dotcode_generator = RosBTDotcodeGenerator(self.bt_data)

        # dot_to_qt transforms into Qt elements using dot layout
        self.dot_to_qt = DotToQtGenerator()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_bt'), 'resource', 'RosBT.ui')
        loadUi(ui_file, self, {'InteractiveGraphicsView': InteractiveGraphicsView})

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_rosgraph)
        self._refresh_timer.start(100)

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self.graphics_view.setScene(self._scene)

        self._bt_topic_completer = TopicCompleter(self.topic_line_edit)
        self._bt_topic_completer.update_topics()
        self.topic_line_edit.setCompleter(self._bt_topic_completer)

        self.namespace_cluster_check_box.clicked.connect(self._refresh_rosgraph)
        self.actionlib_check_box.clicked.connect(self._refresh_rosgraph)

        self.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self.refresh_graph_push_button.pressed.connect(self._update_rosgraph)

        self.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        self.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)
        self.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        self.fit_in_view_push_button.pressed.connect(self._fit_in_view)

        self.depth_spin_box.valueChanged.connect(self._refresh_rosgraph)
        self.depth_spin_box.setMinimum(-1)

        self.load_dot_push_button.setIcon(QIcon.fromTheme('document-open'))
        self.load_dot_push_button.pressed.connect(self._load_dot)
        self.save_dot_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self.save_dot_push_button.pressed.connect(self._save_dot)
        self.save_as_svg_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self.save_as_svg_push_button.pressed.connect(self._save_svg)
        self.save_as_image_push_button.setIcon(QIcon.fromTheme('image'))
        self.save_as_image_push_button.pressed.connect(self._save_image)

        self._update_rosgraph()
        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

    @Slot(str)
    def on_topic_line_edit_textChanged(self, topic_name):
        rospy.loginfo('text changed')
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._bt_topic_completer.update_topics()

        plottable, message = is_plottable(topic_name)
        self.subscribe_topic_button.setEnabled(plottable)
        self.subscribe_topic_button.setToolTip(message)

    @Slot()
    def on_topic_line_edit_returnPressed(self):
        rospy.loginfo('return pressed')
        if self.subscribe_topic_button.isEnabled():
            self.add_topic(str(self.topic_edit.text()))

    def _update_rosgraph(self):
        # re-enable controls customizing fetched ROS graph
        self.topic_line_edit.setEnabled(True)
        self.topic_update_line_edit.setEnabled(True)
        self.namespace_cluster_check_box.setEnabled(True)
        self.actionlib_check_box.setEnabled(True)

        self._graph = rosgraph.impl.graph.Graph()
        self._graph.set_master_stale(5.0)
        self._graph.set_node_stale(5.0)
        self._graph.update()
        self._refresh_rosgraph()

    def _refresh_rosgraph(self):
        if not self._initialized:
            return
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        topic_name = self.topic_line_edit.text()
        topic_update_name = self.topic_update_line_edit.text()
        orientation = 'LR'
        if self.namespace_cluster_check_box.isChecked():
            namespace_cluster = 1
        else:
            namespace_cluster = 0
        accumulate_actions = self.actionlib_check_box.isChecked()
        depth = self.depth_spin_box.value()

        return self.dotcode_generator.generate_dotcode(
                max_depth=depth)

    def _update_graph_view(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()

    def _generate_tool_tip(self, url):
        if url is not None and ':' in url:
            item_type, item_path = url.split(':', 1)
            if item_type == 'node':
                tool_tip = 'Node:\n  %s' % (item_path)
                service_names = rosservice.get_service_list(node=item_path)
                if service_names:
                    tool_tip += '\nServices:'
                    for service_name in service_names:
                        try:
                            service_type = rosservice.get_service_type(service_name)
                            tool_tip += '\n  %s [%s]' % (service_name, service_type)
                        except rosservice.ROSServiceIOException as e:
                            tool_tip += '\n  %s' % (e)
                return tool_tip
            elif item_type == 'topic':
                topic_type, topic_name, _ = rostopic.get_topic_type(item_path)
                return 'Topic:\n  %s\nType:\n  %s' % (topic_name, topic_type)
        return url

    def _redraw_graph_view(self):
        self._scene.clear()

        if self.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1

        # layout graph and create qt items
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level=highlight_level,
                                                            same_label_siblings=True)

        for node_item in nodes.values():
            self._scene.addItem(node_item)
        for edge_items in edges.values():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())
        if self.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _load_dot(self, file_name=None):
        if file_name is None:
            file_name, _ = QFileDialog.getOpenFileName(self, self.tr('Open graph from file'), None, self.tr('DOT graph (*.dot)'))
            if file_name is None or file_name == '':
                return

        try:
            fh = open(file_name, 'rb')
            dotcode = fh.read()
            fh.close()
        except IOError:
            return

        # disable controls customizing fetched ROS graph
        self.graph_type_combo_box.setEnabled(False)
        self.filter_line_edit.setEnabled(False)
        self.topic_filter_line_edit.setEnabled(False)
        self.namespace_cluster_check_box.setEnabled(False)
        self.actionlib_check_box.setEnabled(False)

        self._update_graph_view(dotcode)

    def _fit_in_view(self):
        self.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_dot(self):
        file_name, _ = QFileDialog.getSaveFileName(self, self.tr('Save as DOT'), 'rosgraph.dot', self.tr('DOT graph (*.dot)'))
        if file_name is None or file_name == '':
            return

        handle = QFile(file_name)
        if not handle.open(QIODevice.WriteOnly | QIODevice.Text):
            return

        handle.write(self._current_dotcode)
        handle.close()

    def _save_svg(self):
        file_name, _ = QFileDialog.getSaveFileName(self, self.tr('Save as SVG'), 'rosgraph.svg', self.tr('Scalable Vector Graphic (*.svg)'))
        if file_name is None or file_name == '':
            return

        generator = QSvgGenerator()
        generator.setFileName(file_name)
        generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

        painter = QPainter(generator)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()

    def _save_image(self):
        file_name, _ = QFileDialog.getSaveFileName(self, self.tr('Save as image'), 'rosgraph.png', self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0).toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic_line_edit_text', self.topic_line_edit.text())
        instance_settings.set_value('topic_update_line_edit_text', self.topic_update_line_edit.text())
        instance_settings.set_value('namespace_cluster_check_box_state', self.namespace_cluster_check_box.isChecked())
        instance_settings.set_value('actionlib_check_box_state', self.actionlib_check_box.isChecked())
        instance_settings.set_value('auto_fit_graph_check_box_state', self.auto_fit_graph_check_box.isChecked())
        instance_settings.set_value('highlight_connections_check_box_state', self.highlight_connections_check_box.isChecked())
        instance_settings.set_value('depth_spin_box_value', self.depth_spin_box.value())

    def restore_settings(self, plugin_settings, instance_settings):
        self.topic_line_edit.setText(instance_settings.value('topic_line_edit_text', '/'))
        self.topic_update_line_edit.setText(instance_settings.value('topic_update_line_edit_text', '/'))
        self.namespace_cluster_check_box.setChecked(instance_settings.value('namespace_cluster_check_box_state', True) in [True, 'true'])
        self.actionlib_check_box.setChecked(instance_settings.value('actionlib_check_box_state', True) in [True, 'true'])
        self.auto_fit_graph_check_box.setChecked(instance_settings.value('auto_fit_graph_check_box_state', True) in [True, 'true'])
        self.highlight_connections_check_box.setChecked(instance_settings.value('highlight_connections_check_box_state', True) in [True, 'true'])
        self.depth_spin_box.setValue(int(instance_settings.value('depth_spin_box_value', -1)))

        self._initialized = True
        self._refresh_rosgraph()
