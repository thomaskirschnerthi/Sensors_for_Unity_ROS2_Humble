import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import socket
import numpy as np
import threading
import struct
import sensor_msgs_py.point_cloud2 as pc2

# 🔧 KONFIGURATION → HIER EINSTELLEN
KINECT_ID = 'kinect2'
DEPTH_PORT = 5009
RGB_PORT = 5010

class kin_empfaenger(Node):
    def __init__(self):
        super().__init__('kinect_tcp_receiver')

        self.de_pub = self.create_publisher(Image, f'/{KINECT_ID}/depth', 10)
        self.rgb_pub = self.create_publisher(Image, f'/{KINECT_ID}/rgb', 10)
        self.pc_pub = self.create_publisher(PointCloud2, f'/{KINECT_ID}/points', 10)

        self.width = 512
        self.height = 424

        self.fx = 365.456
        self.fy = 365.456
        self.cx = 254.878
        self.cy = 205.395

        self.latest_depth = None
        self.latest_rgb = None

        threading.Thread(target=self.start_depth_server, daemon=True).start()
        threading.Thread(target=self.start_rgb_server, daemon=True).start()

    def start_depth_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('0.0.0.0', DEPTH_PORT))
        server.listen(1)
        self.get_logger().info(f'TCP Depth Server läuft auf Port {DEPTH_PORT}')

        while rclpy.ok():
            conn, _ = server.accept()
            data = self.recv_all(conn, self.width * self.height * 4)
            conn.close()
            if data:
                self.latest_depth = np.frombuffer(data, dtype=np.float32).reshape((self.height, self.width))
                self.publish_depth(self.latest_depth)
                self.try_publish_pointcloud()

    def start_rgb_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('0.0.0.0', RGB_PORT))
        server.listen(1)
        self.get_logger().info(f'TCP RGB Server läuft auf Port {RGB_PORT}')

        while rclpy.ok():
            conn, _ = server.accept()
            data = self.recv_all(conn, self.width * self.height * 3)
            conn.close()
            if data:
                self.latest_rgb = np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3))
                self.publish_rgb(self.latest_rgb)
                self.try_publish_pointcloud()

    def recv_all(self, conn, size):
        buffer = b''
        while len(buffer) < size:
            chunk = conn.recv(size - len(buffer))
            if not chunk:
                return None
            buffer += chunk
        return buffer

    def publish_depth(self, array):
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = '32FC1'
        msg.step = self.width * 4
        msg.data = array.astype(np.float32).tobytes()
        self.de_pub.publish(msg)

    def publish_rgb(self, array):
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.step = self.width * 3
        msg.data = array.tobytes()
        self.rgb_pub.publish(msg)

    def try_publish_pointcloud(self):
        if self.latest_depth is None or self.latest_rgb is None:
            return

        points = []
        for v in range(self.height):
            for u in range(self.width):
                Z = self.latest_depth[v, u]
                if Z == 0.0 or not np.isfinite(Z):
                    continue
                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy
                r, g, b = self.latest_rgb[v, u]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
                points.append([X, Y, Z, rgb])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        pc_msg = pc2.create_cloud(header, fields, points)
        self.pc_pub.publish(pc_msg)

def main():
    rclpy.init()
    node = kin_empfaenger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

