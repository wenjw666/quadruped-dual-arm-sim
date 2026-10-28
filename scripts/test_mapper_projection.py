"""Regression checks for timestamped, tilted legged-base laser projection."""
import math
import unittest
from unittest.mock import Mock
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from qda_perception.mapper_node import MapperNode

class ProjectionTest(unittest.TestCase):
    def mapper(self, pitch=0.):
        node=MapperNode.__new__(MapperNode)
        node.origin=(-10.,-10.);node.resolution=.05;node.width=node.height=400
        node.obstacle_z=(.08,.62);node.grid=np.full((400,400),-1,dtype=np.int8)
        transform=TransformStamped();transform.transform.translation.z=.5
        transform.transform.rotation.y=math.sin(pitch/2)
        transform.transform.rotation.w=math.cos(pitch/2)
        node.tf_buffer=Mock();node.tf_buffer.lookup_transform.return_value=transform
        return node

    def scan(self,distance):
        scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp.sec=12
        scan.header.stamp.nanosec=345000000;scan.range_min=.1;scan.range_max=12.
        scan.ranges=[distance];return scan

    def test_acquisition_timestamp_and_wall(self):
        node=self.mapper();node._on_scan(self.scan(2.))
        self.assertEqual(node.grid[node._cell(2.,0.)],100)
        requested=node.tf_buffer.lookup_transform.call_args.args[2]
        self.assertEqual(requested.nanoseconds,12345000000)

    def test_pitched_floor_return_is_not_an_obstacle(self):
        node=self.mapper(math.radians(15))
        node._on_scan(self.scan(.5/math.sin(math.radians(15))))
        self.assertFalse(np.any(node.grid==100))

    def test_missing_historical_transform_drops_scan(self):
        node=self.mapper();node.tf_buffer.lookup_transform.side_effect=RuntimeError('TF unavailable')
        node._on_scan(self.scan(2.))
        self.assertTrue(np.all(node.grid==-1))

if __name__=='__main__':unittest.main()
