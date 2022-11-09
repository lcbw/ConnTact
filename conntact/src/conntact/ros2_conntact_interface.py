# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

# Imports for ros
import rclpy
import time
import inspect

from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, \
    Transform
import tf2_ros, rospkg
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import String
from rclpy.clock import Clock, ROSClock
from rclpy.logging import LoggingSeverity
from rclpy.time import Time
import tf2_geometry_msgs
import tf2
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import (ListControllers, LoadController,
                                         SwitchController)
import numpy as np
from colorama import Fore, Back, Style
import yaml
from conntact.conntact_interface import ConntactInterface

class ConntactROS2Interface(ConntactInterface):
    def __init__(self, conntact_params="conntact_params", this_package_name=None):
        #read in conntact parameters

        self.params = {}
        if(this_package_name is None):
            this_package_name = "conntact"
        self.path = self.get_package_path(this_package_name)
        self.params.update(self.load_yaml_file(conntact_params))

        self._wrench_pub = self.create_publisher(WrenchStamped, '/cartesian_compliance_controller/target_wrench',
                                           10)
        self._pose_pub = self.create_publisher(PoseStamped,'cartesian_compliance_controller/target_frame',10)
        self._adj_wrench_pub = self.create_publisher(WrenchStamped, 'adjusted_wrench_force', 10)

        # for plotting node
        self.avg_wrench_pub = self.create_publisher(Wrench, '/cartesian_compliance_controller/target_wrench',
                                           10)
        self.avg_speed_pub = self.create_publisher(Point, "/conntext/avg_speed",
                                           10)
        self.rel_position_pub = self.create_publisher(Point, "/conntext/rel_position",
                                           10)
        self.status_pub = self.create_publisher(String, "/conntext/status",
                                           10)

        # self._ft_sensor_sub = self.create_subscription(WrenchStamped, "/force_torque_sensor_broadcaster/wrench", self.callback_update_wrench, 10)
        self._ft_sensor_sub = self.create_subscription(WrenchStamped, "/cartesian_compliance_controller/ft_sensor_wrench/", self.callback_update_wrench, 10)

        self.tf_buffer = Buffer(Clock,cache_time=tf2.Duration(1200.0)) # rclpy.duration.Duration(seconds=1200.0)
        # self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        rate_integer = self.params["framework"]["refresh_rate"]
        self._rate = self.create_rate(rate_integer) # rospy.Rate(rate_integer)  # setup for sleeping in hz
        self._start_time = self.get_clock().now() ##rospy.get_rostime() or ROSClock().now()
        # self._start_time = rospy.get_rostime() #this gets the current time as a time object
        self.framesDict = {}
        self.current_wrench = WrenchStamped()

        # Set up services:
        skip = False
        # skip = True
        if(not skip):
            try:
                zeroForceService = self.create_client(Trigger, "/ur_hardware_interface/zero_ftsensor")
                while not zeroForceService.wait_for_service(timeout_sec=5.0): #rospy.wait_for_service("/ur_hardware_interface/zero_ftsensor", 5)
                    self.get_logger().info('zero ft sensor service not available, waiting again...')
                self.send_info("connected to service zero_ftsensor")
                self.zero_ft_sensor()
            # except(rospy.ROSException):
            except Exception:
                self.send_info("failed to find service zero_ftsensor")

            # Set up controller:
            try:
                switch_ctrl_srv = self.create_client(SwitchController, "/controller_manager/switch_controller")
                while not switch_ctrl_srv.wait_for_service(timeout_sec=5.0):
                    self.get_logger().info('switchcontroller sensor service not available, waiting again...')
                controller_lister_srv = self.create_client(ListControllers, "/controller_manager/list_controllers")
                while not controller_lister_srv.wait_for_service(timeout_sec=5.0):
                    self.get_logger().info('list controllers service not available, waiting again...')
                # rospy.wait_for_service("/controller_manager/switch_controller", 5)
                # rospy.wait_for_service("/controller_manager/list_controllers", 5)
                # switch_ctrl_srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
                # controller_lister_srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
                def get_cart_ctrl(ctrl_list):
                    """
                    :param ctrl_list: List of controller objects
                    :return: either a ref. to the cartesian compliance controller object or None if none exists
                    """
                    for a in ctrl_list:
                        if (a.name == "cartesian_compliance_controller"):
                            return a
                    return None

                start_controllers = ['cartesian_compliance_controller']
                stop_controllers = ['pos_joint_traj_controller']
                strictness = 2
                start_asap = False
                timeout = 1.0
                # Let's try 3 times and then report error.
                a = 100
                while a > 0:
                    switch_ctrl_srv(start_controllers=start_controllers, stop_controllers=stop_controllers,
                                    strictness=strictness, start_asap=start_asap, timeout=timeout)
                    ctrl_list = controller_lister_srv().controller
                    # self.send_info(Back.LIGHTBLACK_EX +"Starting controller list: {}".format(ctrl_list))
                    if (not ctrl_list):
                        rclpy.logging._root_logger.log("No controllers in controller list! Controller list manager not ready.", LoggingSeverity.ERROR) ##what's the difference between these two lines?
                        # rospy.logerr("No controllers in controller list! Controller list manager not ready.")
                        time.sleep(0.5) # rospy.sleep(.5)
                        continue
                    else:
                        cart_ctrl = get_cart_ctrl(ctrl_list)
                        if (cart_ctrl is not None):
                            if cart_ctrl.state == 'running':
                                self.send_info("Switched to cartesian_compliance_controller successfully.")
                                break
                        if (a > 1):
                            self.send_info("Trying again to switch to compliance_controller...")
                            time.sleep(0.5)  # rospy.sleep(.5)
                        else:
                            self.send_info("Couldn't switch to compliance controller! Try switching manually.")
                    a -= 1
            except Exception:
                self.send_info("failed to find service switch_controller. Try switching manually to begin.")

    def load_yaml_file(self, filename):
        with open(self.path + '/config/' + filename + '.yaml') as stream:
            try:
                info = yaml.safe_load(stream)
                # print(info)
                return info
            except yaml.YAMLError as exc:
                print(exc)

    def get_unified_time(self, float=False):
        """
        :return: Current time. Conntact always measures periods relative to time since
        Conntext.__init__ ran by storing this value at that time; you can use this
        method to make Conntact timestamps correspond with other elements of your system.
        :rtype: :class: `double`
        """
        # return np.double(rospy.get_rostime().to_sec())
        if not float:
            return ROSClock().now() #rospy.get_rostime()  #rostime object
        return self.get_clock().now() #rospy.get_time() #float object

    def get_package_path(self, pkg):
        """ Returns the position of `end` frame relative to `start` frame.
        :return: (string) Path to the current package, under which /config/conntact_params can be found.
        :rtype: :class:`string`
        """
        return get_package_share_directory('conntact') #rospkg.RosPack().get_path(pkg)

    def callback_update_wrench(self, data: WrenchStamped):
        """Callback to update current wrench data whenever new data becomes available.
        """
        self.current_wrench = data

    def get_current_wrench(self):
        """ Returns the most recent wrench (force/torque) reading from the sensor.
        :return: (WrenchStamped) Most recent wrench.
        :rtype: :class:`geometry_msgs.msg.WrenchStamped`
        """
        return self.current_wrench

    def do_transform(self, input, target_frame):
        """Convert transform into the given frame-of-reference.
        :param input: (PoseStamped) Transform from a frame in the TF tree to a point of interest
        :param target_frame: (string) Frame in which to represent the input position
        """
        return self.tf_buffer.transform(input, target_frame, rclpy.duration.Duration(seconds=0.1))

    def get_transform(self, frame, origin):
        """ Returns the position of `end` frame relative to `start` frame.
        :param frame: (string) name of target frame
        :param origin: (string) name of origin frame
        :return: (geometry_msgs.TransformStamped)
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        # print(Fore.MAGENTA + "lookup tcps: {}, {}".format(frame, origin) + Style.RESET_ALL)
        try:
            output = self.tf_buffer.lookup_transform(origin, frame, self.get_clock().now(), rclpy.duration.Duration(seconds=1.0)) # rclpy.duration.Duration(seconds=0.1) #ROSClock().now()
            # print("Lookup successful! It's {} !!".format(output))
            # print("Lookup successful between {} and {} on behalf of method stack {}.".format(
            #     frame, origin, inspect.stack()[1][3]))
            return output
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error with getting TF!")
            return None

    def get_transform_change_over_time(self, frame, origin, delta_time):
        """ Returns the change in position of `end` frame relative to `start` frame over the last delta_time period. If the delta_time param matches this loop's frequency, the returned value will be equivalent to the instantaneous velocity.
        :param frame: (string) name of target frame
        :param origin: (string) name of origin frame
        :return: (geometry_msgs.TransformStamped)
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """

        try:
            earlier_position = self.tf_buffer.lookup_transform(
                origin, frame,
                self.get_clock().now() - rclpy.time.Duration(delta_time),
                rclpy.duration.Duration(seconds=1.0)) # ROSClock().now()
            current_position = self.tf_buffer.lookup_transform(
                origin, frame,
                self.get_clock().now() ,
                rclpy.duration.Duration(seconds=1.0)) # ROSClock().now()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return (None, None)

        # Speed Diff: distance moved / time between poses
        def as_array(vec):
            return np.array([vec.x, vec.y, vec.z])

        position_diff = as_array(current_position.transform.translation) \
                        - as_array(earlier_position.transform.translation)
        time_diff = (current_position.header.stamp - earlier_position.header.stamp).to_sec()
        return position_diff, time_diff

    def register_frames(self, framesDict):
        """ Adds one or more frames of reference to the environment.
        :param framesList: (List) List of `geometry_msgs.msg.TransformStamped` frames to be added to the environment for later reference with get_pose
        """
        # if(self.reference_frames['tcp'].header.frame_id != ''):
        #     # print("Broadcasting tfs: " + str(self.reference_frames))
        #     self._rate.sleep()
        #     self.broadcaster.sendTransform(list(self.reference_frames.values()))

        self.framesDict.update(framesDict)
        self.broadcaster.sendTransform(list(self.framesDict.values()))

    def send_error(self, message, delay=0.0):
        """Displays an error message for the user.
        :param message: (str) to display
        :param delay: (float) This particular message will not display again for this long.
        """
        rclpy.logging._root_logger.log(message,
            LoggingSeverity.ERROR,
            throttle_duration_sec=delay,
            throttle_time_source_type=ROSClock(),
        )

        # rospy.logerr_throttle(delay, message)

    def send_info(self, message, delay=0.0):
        """Displays an error message for the user.
        :param message: (str) to display
        :param delay: (float) This particular message will not display again for this long.
        """
        rclpy.logging._root_logger.log(message,
            LoggingSeverity.INFO,
            throttle_duration_sec=delay,
            throttle_time_source_type=ROSClock(),
        )
        # rospy.loginfo_throttle(delay, message)

    def publish_command_wrench(self, wrench: WrenchStamped):
        """Returns a force and torque command out of Conntact and into the calling environment so that the robot can act upon that command.
        :param wrench: (WrenchStamped) commanded force and torque object.
        """
        self._wrench_pub.publish(wrench)
        # print("Command wrench is {}".format(wrench))

    def publish_averaged_wrench(self, wrench: Wrench):
        self._adj_wrench_pub.publish(wrench)
        return

    def publish_plotting_values(self, items: dict):
        self.status_pub.publish(str(items["status_dict"]))
        self.avg_wrench_pub.publish(items["_average_wrench_world"])
        self.avg_speed_pub.publish(
            Point(items["average_speed"][0], items["average_speed"][1], items["average_speed"][2]))
        self.rel_position_pub.publish(items["current_pose"])

    def publish_command_position(self, pose: PoseStamped):
        """Returns a position command out of Conntact and into the calling environment so that the robot can act upon that command.
        :param pos: (PoseStamped) commanded pose object.
        """
        self._pose_pub.publish(pose)

    def sleep_until_next_loop(self):
        self._rate.sleep()

    def zero_ft_sensor(self):
        if self.zeroForceService():
            self.send_info("Successfully zeroed the force-torque sensor.")
        else:
            self.send_info("Warning: Unsuccessfully tried to zero the force-torque sensor.")

    # def change_compliance_params(self, new_gains = None, new_stiffness= None):
    #     rospy.wait_for_service("/cartesian_compliance_controller/pd_gains/trans_z/set_parameters", timeout=5)
    #     self.updateParamZ = rospy.ServiceProxy("/cartesian_compliance_controller/pd_gains/trans_z/set_parameters",
    #                                            Reconfigure)

    def print_not_found_error(self):
        """Whine about the abstract method not being overridden in the implementation.
        """
        print("Abstract Conntact method {} not yet implemented.".format(inspect.stack()[1][3]))
        raise NotImplementedError()

