"""Capture the actual Humble rqt_graph Qt widget and its native graph export.

Run with ROS_DOMAIN_ID matching the stack, under a real X display or xvfb-run.
This uses the installed rqt_graph plugin against live DDS discovery. No graph
nodes or edges are authored here. Services/HTTP are not shown by rqt_graph.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import rclpy
from python_qt_binding.QtCore import QObject, Qt
from python_qt_binding.QtGui import QImage, QPainter
from python_qt_binding.QtWidgets import QApplication, QMainWindow
from rqt_graph.ros_graph import RosGraph


class Context(QObject):
    def __init__(self, node, window):
        super().__init__()
        self.node, self.window = node, window

    def serial_number(self):
        return 1

    def add_widget(self, widget):
        self.window.setCentralWidget(widget)


class Settings:
    def __init__(self, values):
        self.values = values

    def value(self, name, default=None):
        return self.values.get(name, default)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/ws/output/ros2_graph')
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    app = QApplication([])
    rclpy.init()
    node = rclpy.create_node('rqt_graph')
    window = QMainWindow()
    window.resize(2400, 1550)
    window.setWindowTitle('rqt_graph | MuJoCo full ROS 2 stack | live DDS | domain 8')
    context = Context(node, window)
    try:
        until = time.monotonic() + 8
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
            app.processEvents()
        graph = RosGraph(context)
        window.show()
        records = []
        for name, mode, topic_filter, quiet in (
            ('rqt-nodes', 0, '/,-/clock,-/rosout,-/parameter_events,-/tf,-/tf_static', True),
            ('rqt-topics', 2, '/,-/clock,-/rosout,-/parameter_events', True),
            ('rqt-full', 2, '/', False),
        ):
            settings = dict(
                graph_type_combo_box_index=mode,
                filter_line_edit_text='/,-/rqt_graph',
                topic_filter_line_edit_text=topic_filter,
                namespace_cluster_spin_box_value=1,
                actionlib_check_box_state=True,
                dead_sinks_check_box_state=False,
                leaf_topics_check_box_state=False,
                quiet_check_box_state=quiet,
                unreachable_check_box_state=False,
                group_tf_check_box_state=False,
                hide_tf_nodes_check_box_state=False,
                group_image_check_box_state=False,
                hide_dynamic_reconfigure_check_box_state=False,
            )
            graph.restore_settings(None, Settings(settings))
            graph._update_rosgraph()
            app.processEvents()
            graph._fit_in_view()
            app.processEvents()
            window.grab().save(str(out / (name + '-screenshot.png')))
            (out / (name + '.dot')).write_text(graph._current_dotcode, encoding='utf-8')
            rect = graph._scene.sceneRect()
            # Native Qt scene, at 2x unless it exceeds a manageable 50M pixels.
            scale = min(2., (50_000_000 / max(1., rect.width() * rect.height())) ** .5)
            img = QImage((rect.size() * scale).toSize(), QImage.Format_ARGB32_Premultiplied)
            img.fill(Qt.white)
            painter = QPainter(img)
            painter.setRenderHint(QPainter.Antialiasing)
            graph._scene.render(painter)
            painter.end()
            img.save(str(out / (name + '-native.png')))
            records.append(dict(name=name, settings=settings,
                                scene_size=[rect.width(), rect.height()],
                                native_pixels=[img.width(), img.height()]))
            print(name, records[-1]['native_pixels'], flush=True)
        (out / 'rqt-capture.json').write_text(json.dumps(dict(
            captured_at=datetime.now(timezone.utc).isoformat(),
            method='Installed rqt_graph.RosGraph plugin, live DDS, native QWidget.grab and QGraphicsScene.render',
            captures=records), indent=2), encoding='utf-8')
    finally:
        window.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
