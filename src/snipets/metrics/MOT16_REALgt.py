import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import os
import copy


BASE_FRAME = 'odom'
BOAT_FRAME = 'Obstacle_Gt'
GT_FILE = 'boatGT_real1.txt'
#bounding_boxes[boat_id] = (bb_width, bb_height)

#bounding_boxes[1] = (0.52*2, 0.66+0.73)
#bounding_boxes[3] = (1.25*2,6.461+1.677)
class MOT16Logger(Node):
    def __init__(self):
        super().__init__('mot16_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gt_file = open(GT_FILE, 'w')
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.written_ids = set()
        print('finished initing')
        self.last_trans=0
        self.frame=0

    def tf_callback(self, msg: TFMessage):
        


        try:
            trans = self.tf_buffer.lookup_transform(BASE_FRAME, BOAT_FRAME, rclpy.time.Time())
            if trans==self.last_trans:
                return
            self.last_trans = copy.deepcopy(trans)
            time_sec = round(trans.header.stamp.sec + (trans.header.stamp.nanosec/10**9),3)

            x,y=trans.transform.translation.x, trans.transform.translation.y
            

          


            
            # trans1 = self.tf_buffer.lookup_transform(BASE_FRAME, vert1, rclpy.time.Time())
            # trans2 = self.tf_buffer.lookup_transform(BASE_FRAME, vert2, rclpy.time.Time())

            # x1, y1 = trans1.transform.translation.x, trans1.transform.translation.y
            # x2, y2 = trans2.transform.translation.x, trans2.transform.translation.y
            # cx = (x1 + x2) / 2
            # cy = (y1 + y2) / 2

            # # Get bounding box only once per boat (constant over time)
            # if boat_id not in self.bounding_boxes:
                
            #     bb_width = abs(x2 - x1)
            #     bb_height = abs(y2 - y1) 
            #     self.bounding_boxes[boat_id] = (bb_width, bb_height)


            # bb_width, bb_height = self.bounding_boxes[boat_id]
            # bb_width = abs(x2 - x1)
            # bb_height = abs(y2 - y1) 
            # bb_left = min(x1, x2) 
            # bb_top = min(y1, y2) #smaller y is higher up, in image coordinates which is what MOT16 uses
            conf = 1
            z = 0.0
            self.frame+=1

            # Format: <time> <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>
            line = f"{time_sec},{self.frame},{1},{0:.2f},{0:.2f},{0:.2f},{0:.2f},{conf},{x:.2f},{y:.2f},{z}\n"
            self.gt_file.write(line)
            print(f"Logged Boat{1}_gt at frame {time_sec}")

        except Exception as e:
            # One or more transforms not available — skip
            print(f"Could not transform Boat{1}: {e}")
            

    def destroy_node(self):
        super().destroy_node()
        self.gt_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = MOT16Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

main()
