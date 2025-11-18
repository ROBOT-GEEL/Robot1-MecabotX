#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
import cv2
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

# --- DEFINIEER HIER UW PIXELWAARDEN ---
# HSL(0, 0%, 100%) -> Wit
PIXEL_VAL_WEG = 255 
# HSL(0, 0%, 74%) -> Lichtgrijs
PIXEL_VAL_WEG_EXTENDED = 189 
# HSL(0, 0%, 46%) -> Donkergrijs (alles hieronder is ook obstakel)
PIXEL_VAL_OBSTAKEL_THRESHOLD = 117 

class DynamicObstaclePublisher(Node):

    def __init__(self, yaml_file):
        super().__init__('dynamic_obstacle_publisher')

        # --- 1. Parameters declareren ---
        self.declare_parameter('allow_extended_zone', True)
        
        # We gebruiken de waarden die we hierboven hebben gedefinieerd
        self.declare_parameter('pixel_val_weg', PIXEL_VAL_WEG)
        self.declare_parameter('pixel_val_weg_extended', PIXEL_VAL_WEG_EXTENDED)
        self.declare_parameter('pixel_val_obstakel_threshold', PIXEL_VAL_OBSTAKEL_THRESHOLD)
        
        # Lees de initiële waarden
        self.allow_extended_zone = self.get_parameter('allow_extended_zone').value
        self.read_parameters() # Lees de pixel-waarden in
        self.get_logger().info(f"Initiële waarden: allow_extended_zone={self.allow_extended_zone}")

        # --- 2. YAML en PGM inlezen ---
        try:
            with open(yaml_file, 'r') as f:
                map_yaml = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Kon YAML-bestand niet laden: {yaml_file}. Fout: {e}")
            rclpy.shutdown()
            return

        yaml_dir = os.path.dirname(yaml_file)
        pgm_file_relative = map_yaml['image']
        pgm_file_absolute = os.path.join(yaml_dir, pgm_file_relative)

        self.resolution = map_yaml['resolution']
        self.origin_x, self.origin_y, _ = map_yaml['origin']

        self.get_logger().info(f"Kaart: {pgm_file_absolute}, resolutie: {self.resolution}, origin: ({self.origin_x},{self.origin_y})")

        self.img = cv2.imread(pgm_file_absolute, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            self.get_logger().error(f"Kon {pgm_file_absolute} niet laden")
            rclpy.shutdown()
            return
            
        self.img_height = self.img.shape[0]

        # --- 3. Publisher met 'Latching' (Transient Local) ---
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Gebruik een unieke, conflictvrije topicnaam
        self.pub = self.create_publisher(PointCloud2, '/no_go_zones/floor_obstacles', qos_profile)

        # --- 4. Parameter callback instellen ---
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- 5. Publiceer de initiële obstakels ---
        self.publish_obstacles()

    def read_parameters(self):
        """Leest de pixelwaarden uit de parameters."""
        self.pixel_val_weg = self.get_parameter('pixel_val_weg').value
        self.pixel_val_weg_extended = self.get_parameter('pixel_val_weg_extended').value
        self.pixel_val_obstakel_threshold = self.get_parameter('pixel_val_obstakel_threshold').value

    def parameter_callback(self, params):
        """Wordt aangeroepen wanneer een parameter wordt gewijzigd."""
        for param in params:
            if param.name == 'allow_extended_zone':
                self.allow_extended_zone = param.value
                self.get_logger().info(f"Zone-instelling gewijzigd naar: {self.allow_extended_zone}")
        
        # Lees alle pixel-parameters opnieuw in, voor het geval ze ook zijn gewijzigd
        self.read_parameters()
        
        # Genereer en publiceer de obstakels opnieuw
        self.publish_obstacles()
        return SetParametersResult(successful=True)

    def publish_obstacles(self):
        """
        Genereert een PointCloud2 van ALLES op de kaart, 
        BEHALVE de toegestane 'weg' pixels.
        """
        self.get_logger().info(f"Obstakel-vloer genereren... (Extended zone: {self.allow_extended_zone})")

        # Maak een masker (kopie) van de hele kaart. 
        # Standaard is ALLES een obstakel (255).
        mask = np.full(self.img.shape, 255, dtype=np.uint8)

        # --- DE INVERSIE LOGICA ---
        # Nu maken we "gaten" in het masker voor de toegestane zones.
        
        # Gat 1: De standaard 'weg' (wit) is ALTIJD vrij (0).
        mask[self.img == self.pixel_val_weg] = 0

        # Gat 2: Als de booleaan AAN is, is de 'extended' weg (lichtgrijs) OOK vrij (0).
        if self.allow_extended_zone:
            mask[self.img == self.pixel_val_weg_extended] = 0
            self.get_logger().info(f"Pad gedefinieerd: WEG ({self.pixel_val_weg}) en EXTENDED ({self.pixel_val_weg_extended})")
        else:
            self.get_logger().info(f"Pad gedefinieerd: Alleen WEG ({self.pixel_val_weg})")

        # Resultaat: 'mask' is nu 255 (obstakel) overal, BEHALVE op de
        # toegestane 'weg', waar het 0 (vrij) is.

        # --- PUNTEN GENEREREN ---
        # Publiceer elke pixel die 255 is (obstakel)
        # np.where geeft (array_of_y_coords, array_of_x_coords)
        y_coords, x_coords = np.where(mask == 255)

        points = []
        # Converteer alle (x, y) pixelcoördinaten naar (world_x, world_y)
        for px, py in zip(x_coords, y_coords):
            # Converteer pixel (x, y) naar wereld (x, y)
            world_x = self.origin_x + px * self.resolution
            # De 'y' as is omgekeerd
            world_y = self.origin_y + (self.img_height - py - 1) * self.resolution
            points.append([world_x, world_y, 0.0]) # Z-as is 0.0

        if not points:
            self.get_logger().warn("Geen obstakelpunten gevonden. Is de kaart leeg?")
            pc2_msg = self.create_pointcloud2([])
        else:
            pc2_msg = self.create_pointcloud2(points)

        self.pub.publish(pc2_msg)
        self.get_logger().info(f"Obstakel-vloer gepubliceerd ({len(points)} punten).")

    def create_pointcloud2(self, points):
        """Maak een PointCloud2 bericht vanuit een lijst van [x, y, z]-punten"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map' # Obstakels zijn in het 'map' frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        data = []
        for x, y, z in points:
            # We sturen 3 floats (x, y, z)
            data.append(struct.pack('fff', x, y, z))
        data_binary = b"".join(data)

        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 12 # 3 floats * 4 bytes/float = 12 bytes
        pc2_msg.row_step = pc2_msg.point_step * len(points)
        pc2_msg.is_dense = True
        pc2_msg.data = data_binary
        return pc2_msg

def main(args=None):
    rclpy.init(args=args)

    try:
        # 1. Vind de 'install/share' map van DIT package
        package_share_dir = get_package_share_directory('no_go_zones')

        # 2. Bouw het correcte pad op naar het kaartbestand
        #    (Zorg dat deze naam klopt met wat in /map staat!)
        yaml_file = os.path.join(package_share_dir, 'map', 'WHEELTEC.yaml')

        # 3. Start de node
        node = DynamicObstaclePublisher(yaml_file)

        # 4. Spin de node
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Node afgesloten.")
        finally:
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        rclpy.logging.get_logger("no_go_zones_main").fatal(f"Kon node niet starten. Fout: {e}")
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
