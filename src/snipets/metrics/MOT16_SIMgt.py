import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import os
from rclpy.parameter import Parameter
from tf_transformations import euler_from_quaternion
import copy


BOAT_IDS = [1, 2, 3]
BASE_FRAME = 'map_gt'
GT_FILE = 'SIMgt_OnlyDynamic_timecuts.txt'

#bounding_boxes[boat_id] = (bb_width, bb_height)
bounding_boxes = [None,
                  (0.52*2, 0.66+0.73),
                  (1.04*2,3.737+3.289),
                  (1.25*2,6.461+1.677)]
last_trans = [None, 0,0,0]

#bounding_boxes[1] = (0.52*2, 0.66+0.73)
#bounding_boxes[3] = (1.25*2,6.461+1.677)
class MOT16Logger(Node):
    def __init__(self):
        super().__init__('mot16_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])


        self.gt_file = open(GT_FILE, 'w')
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.written_ids = set()
        print('finished initing')
        self.frame=0
        self.last_time_sec=0

        """"
        trans = self.tf_buffer.lookup_transform(BASE_FRAME, BOAT_FRAME, rclpy.time.Time())
        if trans==self.last_trans:
            return
        self.last_trans = copy.deepcopy(trans)
        time_sec = round(trans.header.stamp.sec + (trans.header.stamp.nanosec/10**9),3)

        x,y=trans.transform.translation.x, trans.transform.translation.y
        """


    def tf_callback(self, msg: TFMessage):
        
        
        
        time_has_updated = False
        for boat_id in BOAT_IDS:
            boat_frame = f"Obstacles_Evolo/Boat{boat_id}_gt"
            vert1 = f"Obstacles_Evolo/Vertice 1_{boat_id}_gt"
            vert2 = f"Obstacles_Evolo/Vertice 2_{boat_id}_gt"
            try:
                
                trans = self.tf_buffer.lookup_transform(BASE_FRAME, boat_frame, rclpy.time.Time())
                if trans == last_trans[boat_id]:
                    continue
                
                last_trans[boat_id]=copy.deepcopy(trans)
                if not time_has_updated:
                    time_sec = round(trans.header.stamp.sec + (trans.header.stamp.nanosec/10**9),3)
                    time_has_updated=True
                    if abs(time_sec - self.last_time_sec)<0.4:
                        return
                    else:
                        self.last_time_sec = time_sec
                        self.frame+=1
                


                x,y=trans.transform.translation.x, trans.transform.translation.y
                

                bb_width, bb_height = bounding_boxes[boat_id][0], bounding_boxes[boat_id][1]
                q = trans.transform.rotation
                quat = [q.x, q.y, q.z, q.w]

                # Convert to Euler angles (roll, pitch, yaw)
                _, _, yaw = euler_from_quaternion(quat)
                #bb_left = x-bounding_boxes[boat_id][0]
                #bb_top = y-bounding_boxes[boat_id][1]


                
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

                # Format: <time> <frame>, <id>, <bb_centerX>, <bb_centerY>, <bb_width>, <bb_height>, <bb_angle>, <x>, <y>, <z>
                line = f"{time_sec},{self.frame},{boat_id},{x:.2f},{y:.2f},{bb_width:.2f},{bb_height:.2f},{yaw:.2f},{x:.2f},{y:.2f},{z}\n"
                self.gt_file.write(line)
                print(f"Logged Boat{boat_id}_gt at frame {self.frame}")

            except Exception as e:
                # One or more transforms not available — skip
                print(f"Could not transform Boat{boat_id}: {e}")
                continue

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
