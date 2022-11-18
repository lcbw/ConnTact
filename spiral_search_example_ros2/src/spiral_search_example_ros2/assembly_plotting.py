# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

# from _typeshed import NoneType
import numpy as np
from std_msgs.msg import String
from colorama import Fore, Back, Style
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
import tf2_ros
import tf2_geometry_msgs
import rclpy
import tf2
from rclpy.clock import Clock, ROSClock
from rclpy.logging import LoggingSeverity
from rclpy.time import Time

class PlotAssemblyData():

    def __init__(self, node: rclpy.Node):
        self.node = node
        self.average_wrench = None
        self.avg_wrench_sub = node.create_subscription(Wrench, "/conntext/avg_wrench", self.callback_update_wrench,
                                           10)
        self.average_speed = None
        self.avg_speed_sub = node.create_subscription(Point, "/conntext/avg_speed", self.callback_update_speed,
                                           10)
        self.pos = None
        self.rel_position_sub = node.create_subscription(Point, "/conntext/rel_position", self.callback_update_pos,
                                           10)
        self.status = None
        self.status_sub = node.create_subscription(String, "/conntext/status", self.callback_update_status,
                                           10)
        
        self.tf_buffer = tf2_ros.Buffer(Clock,cache_time=tf2.Duration(1200.0))
        
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        # Config variables
        rate_integer = self.params["framework"]["refresh_rate"]
        self._rate = self.create_rate(rate_integer) ##unsure about this
        # self.rate = rospy.Rate(50)
        self.recordInterval = rclpy.duration.Duration(seconds=0.1) #rclpy.time.Duration(seconds=0.1) #
        self.plotInterval = rclpy.duration.Duration(seconds=0.5)
        self.lastPlotted = rclpy.time.Duration(seconds=0.0) #rospy.Time(0)
        self.lastRecorded = rclpy.time.Duration(seconds=0.0) #rospy.Time(0)
        self.recordLength = 100
        self.surface_height = None

        # rclpy.logging._root_logger.log(message,
        #     LoggingSeverity.INFO,
        #     throttle_duration_sec=delay,
        #     throttle_time_source_type=ROSClock(),
        # )
        # rospy.loginfo(Fore.GREEN+Back.MAGENTA+"Plotter node active."+Style.RESET_ALL)
    

    def callback_update_wrench(self, data: Wrench):
        self.average_wrench = data
    
    def callback_update_speed(self, data: Point):
        self.average_speed = self.point_to_array(data)
    
    def callback_update_pos(self, data: Point):
        self.pos = self.point_to_array(data)
    
    def callback_update_status(self, data: String):
        
        self.status = dict(eval(str(data.data)))
        # If surface has been found we add a line to the plot:
        if('surface_height' in self.status and self.surface_height == None):
            self.surface_height = self.status['surface_height']

    def point_to_array(self, point):
        return np.array([point.x, point.y, point.z])

    def init_plot(self):
            #plt.axis([-50,50,0,10000])
        # Data containers
        self.speedHistory = np.array(self.average_speed)

        self.forceHistory = self.point_to_array(self.average_wrench.force)
        # 3xN array, vertically stacked positions
        self.posHistory = self.pos*1000
        self._start_time = self.node.get_clock().now() #ROSClock().now() was rospy.get_rostime()
        self.plotTimes = [0.0]
        plt.ion()
        # plt.show()
        # plt.draw()

        self.fig, (self.planView, self.sideView) = plt.subplots(1, 2, figsize=(8,6))
        self.fig.figsize = (7,4)
        self.sideViewTwin = self.sideView.twinx()
        self.fig.suptitle('Algorithm Path Plots')
        plt.subplots_adjust(left=.125, bottom=.2, right=.9, top=.8, wspace=.175, hspace=.1)

        # self.min_plot_window_offset = 2
        self.min_plot_window_size = 10
        self.min_force_window_size = 2
        self.min_z_pos_window_size = 20
        self.pointOffset = 1
        self.barb_interval = 5
        # self.fig.tight_layout()
    
    def update_plots(self):
        #Record data to the buffer
        if(self.node.get_clock().now() > self.lastRecorded + self.recordInterval):
            self.lastRecorded = self.node.get_clock().now()  #rospy.Time.now()

            #log all interesting data
            self.speedHistory = np.vstack((self.speedHistory, np.array(self.average_speed)*1000))
            self.forceHistory = np.vstack((self.forceHistory, self.point_to_array(self.average_wrench.force)))
            self.posHistory = np.vstack((self.posHistory, self.pos*1000))
            self.plotTimes.append((ROSClock().now() - self._start_time).to_sec())
            
            #limit list lengths
            if(len(self.speedHistory)>self.recordLength):
                self.speedHistory = self.speedHistory[1:self.recordLength-1]
                self.forceHistory = self.forceHistory[1:self.recordLength-1]
                self.posHistory = self.posHistory[1:self.recordLength-1]
                self.plotTimes = self.plotTimes[1:self.recordLength-1]
                self.pointOffset += 1
                

        #Actually plot the data; this is computation-heavy so we minimize the hz
        if(self.node.get_clock().now() > self.lastPlotted + self.plotInterval):
            self.lastPlotted = self.node.get_clock().now()

            self.planView.clear()
            self.sideView.clear()

            self.planView.set(xlabel='X Position',ylabel='Y Position')
            self.planView.set_title('XY Position and Force')
            self.sideView.set(xlabel='Time (s)',ylabel='Position (mm)')
            self.sideView.set_title('Vertical Position and Force')
            self.sideViewTwin.set_ylabel('Force (N)')
            self.planView.legend(["Force", "Position"])
            self.sideView.legend()
            self.sideViewTwin.legend()
            
            # self.sideView.plot(self.plotTimes[:], self.forceHistory[:,2], 'k', self.plotTimes[:], self.posHistory[:,2], 'b')
            self.sideViewTwin.plot(self.plotTimes[:], self.forceHistory[:,2], 'c')
            self.sideView.plot(self.plotTimes[:], self.posHistory[:,2], 'k')
            self.planView.plot(self.posHistory[:,0], self.posHistory[:,1], 'k')

            #self.planView.quiver(self.posHistory[0:-1:10,0], self.posHistory[0:-1:10,1], self.forceHistory[0:-1:10,0], self.forceHistory[0:-1:10,1], angles='xy', scale_units='xy', scale=.1, color='b')
            barb_increments = {"flag" :5, "full" : 1, "half" : 0.5}

            offset = self.barb_interval+1-(self.pointOffset % self.barb_interval)

            #determine how much of forceHistory to actually plot; caps the record length to actually show
            # barb_number = int(min([self.recordLength / 2, self.forceHistory.shape[0]]))
            # rospy.loginfo("barb number is " + str(barb_number) + " when list length is " + str(self.forceHistory.shape[0]))
            #Create a color differentiation from new to old barbs.
            colorList = [(.2, n, n*n) for n in np.linspace(.1,1,(self.recordLength/self.barb_interval))]
            
            #Set up plan view; decide the width limits
            xlims = [np.min(self.posHistory[:,0]) - self.min_plot_window_size, np.max(self.posHistory[:,0]) + self.min_plot_window_size]
            ylims = [np.min(self.posHistory[:,1]) - self.min_plot_window_size, np.max(self.posHistory[:,1]) + self.min_plot_window_size]

            current_force_barb_position = (.25* xlims[0] + .75*xlims[1], .25* ylims[0] + .75*ylims[1],)

            self.planView.barbs(
                self.posHistory[offset:-1:self.barb_interval,0], 
                self.posHistory[offset:-1:self.barb_interval,1], 
                self.forceHistory[offset:-1:self.barb_interval,0], 
                self.forceHistory[offset:-1:self.barb_interval,1],
                barb_increments=barb_increments, length = 6, color=colorList)

            self.planView.barbs(
                current_force_barb_position[0], 
                current_force_barb_position[1], 
                self.forceHistory[-1,0], 
                self.forceHistory[-1,1],
                barb_increments=barb_increments, length = 7, color=(0,.2,.8))

            #Apply view parameters
            self.planView.set_xlim(xlims)
            self.planView.set_ylim(ylims)

            self.sideViewTwin.set_ylim((np.min(self.forceHistory[:,2])-self.min_force_window_size, np.max(self.forceHistory[:,2])+self.min_force_window_size))
            self.sideView.set_ylim((np.min(self.posHistory[:,2])-self.min_z_pos_window_size, np.max(self.posHistory[:,2])+self.min_z_pos_window_size))
            self.sideView.set_xlim((self.plotTimes[0],self.plotTimes[-1]))

            # for ax in [self.sideView, self.sideViewTwin]:
            #     bottom, top = ax.get_ylim()
            #     if top < 1.0:
            #         ax.set_ylim(top=1)
            #     if bottom > -1.0:
            #         ax.set_ylim(bottom=-1)



            self.planView.annotate("State is " + self.status['state'], xy=(.05, .03), xycoords='figure fraction',size=10, ha="left", va="bottom",
            bbox=dict(boxstyle="round4",
            ec=(1., 0.5, 0.5),
            fc=(1., 0.8, 0.8)
            )
            )

            self.planView.annotate("TCP: " + self.status['tcp_name'], xy=(.95, .03), xycoords='figure fraction',size=10, ha="right", va="bottom",
            bbox=dict(boxstyle="round4",
            ec=(1., 0.5, 0.5),
            fc=(1., 0.8, 0.8)
            )
            )

            #Add some informative annotations
            force_message = "Force x: " + str(np.round(self.average_wrench.force.x,2)) + "\nForce y: " + str(np.round(self.average_wrench.force.y,2))

            self.planView.annotate(force_message, xy=(3, 3), xycoords='axes points',size=8, ha="left", va="bottom",
            )

            vert_message = "Force z: " + str(np.round(self.average_wrench.force.z,2)) + "\nPosition z :" + str(np.round(self.pos[2],2))

            self.sideView.annotate(vert_message, xy=(3, 3), xycoords='axes points',size=8, ha="left", va="bottom",
            )
            
            self.fig.canvas.draw()


    def main(self):

        # vital_data = [self.average_speed, self.average_wrench, self.status, self.pos]
        # #Wait till we have real values for everything
        self._ft_sensor_sub_test = self.node.create_subscription(WrenchStamped,
                                                            "/cartesian_compliance_controller/target_wrench",
                                                            None, 10)
        while self._ft_sensor_sub.msg.data is None:
            self.rate.sleep()
        # while (type(None) in [type(self.average_speed), type(self.average_wrench), type(self.status), type(self.pos)]):
        #     # vital_data = [self.average_speed, self.average_wrench, self.status, self.pos]
        #     # x = vital_data[:] == [2,2,2,2]
        #
        #     self.rate.sleep()

        self.node.get_logger().info("Plotter node starting")
        self.init_plot()

        while (rclpy.ok()):
            self.update_plots()
            self.rate.sleep()
