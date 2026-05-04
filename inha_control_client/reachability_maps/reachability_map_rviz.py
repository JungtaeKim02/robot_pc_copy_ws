#!/usr/bin/env python3
"""
Reachability Map RViz Publisher (ROS 2)
"""

import argparse
import os
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# ---------- 기존 맵 로드 로직 (그대로 가져옴) ----------
class LoadedMap:
    def __init__(self, occupancy, resolution, bounds, base_link, source_path, extra):
        self.occupancy = occupancy
        self.resolution = resolution
        self.bounds = bounds
        self.base_link = base_link
        self.source_path = source_path
        self.extra = extra

    def cell_centers_world(self) -> np.ndarray:
        idx = np.argwhere(self.occupancy > 0)
        if idx.size == 0:
            return np.zeros((0, 3))
        origins = np.array([b[0] for b in self.bounds])
        return origins + (idx + 0.5) * self.resolution

def load_bin(path: str) -> LoadedMap:
    with open(path, 'rb') as f:
        header = f.read(40)
        (res, minx, maxx, miny, maxy, minz, maxz, dx, dy, dz) = struct.unpack('fffffffiii', header)
        total = dx * dy * dz
        body = f.read(total)
    occupancy = np.frombuffer(body, dtype=np.uint8).reshape((dx, dy, dz)).copy()
    return LoadedMap(occupancy, float(res), ((float(minx), float(maxx)), (float(miny), float(maxy)), (float(minz), float(maxz))), 'base_link', path, {})

def load_npz(path: str) -> LoadedMap:
    data = np.load(path, allow_pickle=True)
    occupancy = np.array(data['occupancy'], dtype=np.uint8)
    res = float(data['resolution'])
    bounds = tuple(map(tuple, np.array(data['workspace_bounds']).tolist()))
    base_link = str(data['base_link']) if 'base_link' in data.files else 'base_link'
    return LoadedMap(occupancy, res, bounds, base_link, path, {})

def load_any(path: str) -> LoadedMap:
    ext = os.path.splitext(path)[1].lower()
    if ext == '.bin': return load_bin(path)
    if ext == '.npz': return load_npz(path)
    raise ValueError(f"Unsupported extension: {ext}")

# ---------- ROS 2 노드 ----------
class ReachabilityRVizPublisher(Node):
    def __init__(self, map_path):
        super().__init__('reachability_rviz_publisher')
        
        # Transient Local: 늦게 켠 RViz에서도 데이터를 받을 수 있도록 설정
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, 'reachability_map', qos_profile)
        
        self.get_logger().info(f"Loading map: {map_path} ...")
        self.map_data = load_any(map_path)
        
        # Marker 생성
        self.marker = Marker()
        self.marker.header.frame_id = "link_torso_2"  # 로봇의 기준 좌표계 (필요시 수정)
        self.marker.ns = "reachability"
        self.marker.id = 0
        self.marker.type = Marker.CUBE_LIST  # 해상도 크기의 큐브들로 표현
        self.marker.action = Marker.ADD
        
        # 큐브(복셀)의 크기 설정 (맵의 resolution과 동일하게)
        self.marker.scale.x = self.map_data.resolution
        self.marker.scale.y = self.map_data.resolution
        self.marker.scale.z = self.map_data.resolution
        
        # 색상 설정 (알파 블렌딩이 너무 많으면 렌더링 부하가 커지므로 a=0.8 정도로 설정)
        self.marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
        
        pts = self.map_data.cell_centers_world()
        self.get_logger().info(f"Converting {len(pts)} points to ROS Marker. This may take a few seconds...")
        
        for p in pts:
            self.marker.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            
        # RViz가 노드를 발견(Discover)할 수 있도록 주기적으로 발행
        self.timer = self.create_timer(2.0, self.publish_marker)
        self.get_logger().info("Map is ready. Publishing every 2.0s to topic: /reachability_map")

    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)

def main(args=None):
    parser = argparse.ArgumentParser(description='Publish Reachability Map to RViz')
    parser.add_argument('--map', required=True, help='Map path (.bin or .npz)')
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ReachabilityRVizPublisher(parsed_args.map)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

