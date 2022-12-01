# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

#UR IP Address is now 175.31.1.137
#Computer has to be 175.31.1.150

# Imports for ros
# from _typeshed import StrPath

import rclpy
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
import time
from conntact.assembly_algorithm_blocks import ConnTask
from transitions import Machine

IDLE_STATE           = 'state_idle'
CHECK_FEEDBACK_STATE = 'state_check_load_cell_feedback'
APPROACH_STATE       = 'state_finding_surface'
FIND_HOLE_STATE      = 'state_finding_hole'
INSERTING_PEG_STATE  = 'state_inserting_along_axis'
COMPLETION_STATE     = 'state_completed_insertion'
EXIT_STATE           = 'state_exit'
SAFETY_RETRACT_STATE = 'state_safety_retraction' 


#Trigger names
CHECK_FEEDBACK_TRIGGER     = 'check loadcell feedback'
APPROACH_SURFACE_TRIGGER   = 'start approach'
FIND_HOLE_TRIGGER          = 'surface found'
INSERT_PEG_TRIGGER         = 'hole found'
ASSEMBLY_COMPLETED_TRIGGER = 'assembly completed'
SAFETY_RETRACTION_TRIGGER  = 'retract to safety'
RESTART_TEST_TRIGGER       = 'restart test'
RUN_LOOP_TRIGGER           = 'run looped code'


class testing():
    def __init__(self, node: rclpy.Node):
        self._wrench_pub = node.create_publisher(WrenchStamped, '/cartesian_compliance_controller/target_wrench',
                                           10)
class CornerSearch(ConnTask, Machine):

    def __init__(self, node: rclpy.Node):
        # Override Assembly Tools config variables
        ROS_rate = 100 #setup for sleeping in hz
        start_time = node.get_clock().now()
        # start_time = rospy.get_rostime() #this gets current time as a time object
        
        ConnTask.__init__(self, ROS_rate, start_time, node)

        #Override Alg Blocks config variables:
        states = [
            IDLE_STATE, 
            CHECK_FEEDBACK_STATE,
            APPROACH_STATE, 
            FIND_HOLE_STATE, 
            INSERTING_PEG_STATE, 
            COMPLETION_STATE, 
            SAFETY_RETRACT_STATE
        ]
        # TODO: Implement sub-task while loops as reflexive transitions according to https://github.com/pytransitions/transitions#reflexive-from-multiple-states
        #TODO: Replace warnings and errors with colorized info according to https://stackoverflow.com/questions/287871/how-to-print-colored-text-to-the-terminal/3332860#3332860
        # TODO: Replace all transitions with dest "safety_retract_state" with a single one using 'source':'*'
        transitions = [
            {'trigger':CHECK_FEEDBACK_TRIGGER    , 'source':IDLE_STATE          , 'dest':CHECK_FEEDBACK_STATE   },
            {'trigger':APPROACH_SURFACE_TRIGGER  , 'source':CHECK_FEEDBACK_STATE, 'dest':APPROACH_STATE         },
            {'trigger':FIND_HOLE_TRIGGER         , 'source':APPROACH_STATE      , 'dest':FIND_HOLE_STATE        },
            {'trigger':INSERT_PEG_TRIGGER        , 'source':FIND_HOLE_STATE     , 'dest':INSERTING_PEG_STATE    },
            {'trigger':ASSEMBLY_COMPLETED_TRIGGER, 'source':INSERTING_PEG_STATE , 'dest':COMPLETION_STATE       },
            {'trigger':SAFETY_RETRACTION_TRIGGER , 'source':'*'                 , 'dest':SAFETY_RETRACT_STATE, 'unless':'is_already_retracting' },
            {'trigger':RESTART_TEST_TRIGGER      , 'source':SAFETY_RETRACT_STATE, 'dest':CHECK_FEEDBACK_STATE   },
            {'trigger':RUN_LOOP_TRIGGER      , 'source':'*', 'dest':None, 'after': 'run_loop'}

        ]
        Machine.__init__(self, states=states, transitions=transitions, initial=IDLE_STATE)

        self.tcp_selected = 'corner'

    def main(self,node):

        def callback(self, msg):
            if msg.data is None:
                node.get_logger().info("waiting for topic")

        # TODO: Remove following Sleep, use a rospy.wait command of some kind
        time.sleep(3)

        self._ft_sensor_sub_test = node.create_subscription(WrenchStamped, "/cartesian_compliance_controller/ft_sensor_wrench/", callback,10)

        while (type(None) in [type(self._ft_sensor_sub_test)]):
            self.rate.sleep()

        self.algorithm_execute()
        node.get_logger().info("Corner Search all done!")

if __name__ == '__main__':
    
    assembly_application = CornerSearch()
    assembly_application.main()
    
