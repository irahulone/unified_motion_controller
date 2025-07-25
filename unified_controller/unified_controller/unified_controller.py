import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import csv
import os

import numpy
import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('pose_listener')
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/sim/p4/pose2D',
            self.listener_callback,
            10)
        self.pose_sub  # prevent unused variable warning
      

        self.pub_cmdvel = self.create_publisher(Twist, '/sim/p4/cmd_vel', 10)
        
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
               
        self.joy_sub
        
        self.leader_sub = self.create_subscription(
            Pose2D,
            '/sim/p3/pose2D',
            self.leader_callback,
            10)
        
        self.leader_sub
        
        self.curx = 0.0
        self.cury = 0.0
        self.j_lx = 0.0
        self.j_az = 0.0 
        self.lx_axisN = 1
        self.az_axisN = 0
        self.en_buttonN = 4
        self.r_heading = 0.0
        self.des_heading = 0.0
        self.wp_distance = 0.0
        self.des_follow_d= 5.0
        self.v=0.0
        self.lead_hdg=0.0
        self.t1=self.get_clock().now()
        self.t2=self.get_clock().now()
        self.tstart=self.get_clock().now()
        self.timer=self.get_clock().now()
        self.ulx=0.0
        self.uaz=0.0
        self.e_d=0.0
        self.e_h=0.0
        self.Eit=0.0
        self.Ect=0.0
        self.leadx=0.0
        self.leady=0.0
        self.testtime=0.0
        self.log_path="/home/rsl-ros/Unified_Controller_Test_Data/unified_control_log.csv"
        self.csv_file=open(self.log_path, mode='w',newline='')
        self.csv_writer=csv.writer(self.csv_file)
        self.csv_writer.writerow(["mode","time","x","y","theta","linear_vel","angular_vel","pos_err","ang_err","err_it","err_ct","leader_x","leader_y","hdg_to_leader"])
        self.csv_interval=self.create_timer(0.1, self.csv_record)


        #self.r_lat = 0.0
        #self.r_lon = 0.0

        self.des_x = 10.0
        self.des_y = 10.0        
        self.heading_set = 0.0
        self.d2= 0.0
        self.ctrl_mode = 4
        self.button = 0  
        self.last_mode=self.ctrl_mode
       


    def listener_callback(self, msg):
        self.r_heading=msg.theta
        #self.r_heading = msg.data
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #print(self.r_heading)
        #self.curx, self.cury=self.lonlat_2_xy(self.r_lat, self.r_lon, self.setlat0, self.setlon0)
        self.Eit=0.0
        self.Ect=0.0
        self.curx=msg.x
        self.cury=msg.y

        self.des_heading = self.get_bearing(self.curx, self.cury, self.des_x, self.des_y)
        #print(self.des_heading)

        self.wp_distance = self.get_distance(self.curx, self.cury, self.des_x, self.des_y)
        #print(self.wp_distance)
        print(" ")

        if self.ctrl_mode==1: #heading
            self.e_h = self.heading_set - self.r_heading 
            self.e_d = 0
            Kaz=1.0 #angular gain
            Klx=0.0 #translational gain
            Vx=0.4 #translational constant
        elif self.ctrl_mode==2: #wp
            self.e_h = self.des_heading - self.r_heading 
            self.e_d = self.wp_distance
            Kaz=1.0 #angular gain
            Klx=0.0 #translational gain
            if self.e_d<0.1:
                Vx=0.0
            elif self.e_d>=0.1:
                Vx=0.4
        elif self.ctrl_mode==3: #follower
            self.e_h = self.lead_hdg - self.r_heading 
            self.e_d = self.d2-self.des_follow_d
            Kaz=1.0 #angular gain
            Klx=0.2 #translational gain
            Vx=self.v #translational constant
        elif self.ctrl_mode==4: #path
            Kaz=0.4 #angular gain
            Klx=0.0 #translational gain
            self.xstart=0
            self.xfinal=20
            self.ystart=0
            self.yfinal=0
            t=0
            xend,yend,thetaend=self.path(self.xfinal,self.yfinal,t,self.ctrl_mode)
            min_dist=float('inf')
            step=0.001
            x=self.xstart
            while x<=self.xfinal:
            	y=self.ystart
            	while y<=self.yfinal:
            	    xpath,ypath,thetapath=self.path(x,y,t,self.ctrl_mode)
            	    dist=self.get_distance(self.curx,self.cury,xpath,ypath)
            	    if dist<min_dist:
            	    	min_dist=dist
            	    	x_des=xpath
            	    	y_des=ypath
            	    	path_dir=thetapath
            	    y=y+step
            	x=x+step
            if self.get_distance(x_des,y_des,xend,yend)<0.01:
            	x_des=xend
            	y_des=yend
            	Vx=0.0
            	self.e_d=self.get_distance(self.curx,self.cury,x_des,y_des)
            	self.e_h=self.get_bearing(self.curx,self.cury,x_des,y_des)-self.r_heading
            	if self.get_distance(self.curx,self.cury,xend,yend)<0.01:
                    Vx=0.0
                    self.e_h=0.0	
            else:
            	e,self.e_h,self.Eit,self.Ect=self.pathpoint(self.curx,self.cury,x_des,y_des,path_dir)
            	self.e_d=0.0
            	Vx=0.4
        elif self.ctrl_mode==5: #trajectory
            x=0.0
            y=0.0
            tfinal=20.0
            xend,yend,thetaend=self.path(x,y,tfinal,self.ctrl_mode)
            if self.last_mode!=5:
            	self.tstart=self.get_clock().now()
            curt=(self.get_clock().now()-self.tstart).nanoseconds/1e9
            if curt>=tfinal:
            	curt=tfinal
            x_des,y_des,path_dir=self.path(x,y,curt,self.ctrl_mode)
            self.e_d,self.e_h,self.Eit,self.Ect=self.pathpoint(self.curx,self.cury,x_des,y_des,path_dir)
            if curt>=tfinal:
            	self.e_d=self.get_distance(self.curx,self.cury,xend,yend)
            	self.e_h=self.get_bearing(self.curx,self.cury,xend,yend)-self.r_heading
            Kaz=0.4 #angular gain
            Klx=0.2 #translational gain
            if self.get_distance(self.curx,self.cury,xend,yend)<0.1:
                Vx=0.0
                self.e_d=0.0
                self.e_h=0.0
            else:
                Vx=0.2
        elif self.ctrl_mode==6: #joy
            self.e_d = self.j_lx #joy input translational
            self.e_h = self.j_az #joy input angular
            Kaz=0.5 #angular gain
            Klx=0.7 #translational gain
            Vx=0 #translational constant
        if self.last_mode !=self.ctrl_mode:
            self.timer=self.get_clock().now()
        self.last_mode=self.ctrl_mode
        self.testtime=(self.get_clock().now()-self.timer).nanoseconds/1e9

        self.uaz = Kaz*self.e_h
        if self.uaz > 0.6:
            self.uaz = 0.6
        elif self.uaz < -0.6:
            self.uaz = -0.6
        
        self.ulx = Vx+Klx*self.e_d
        if self.ulx > 0.4:
            self.ulx = 0.4
        elif self.ulx < -0.4:
            self.ulx = -0.4
            

        msg_cmd = Twist()
        msg_cmd.linear.x = self.ulx
        msg_cmd.angular.z = self.uaz
        self.pub_cmdvel.publish(msg_cmd)

    #def gps_agg_cb(self, msg):
    #    self.r_lat = msg.latitude
    #    self.r_lon = msg.longitude

    def get_bearing(self, x1, y1, x2, y2):
        dx = (x2 - x1)
        dy = (y2 - y1)
        brng = math.atan2(dy,dx)
        brng = (brng+math.pi)%(2*math.pi)-math.pi
        return brng


    def get_distance(self, x1, y1, x2, y2):
        dx = (x2-x1)
        dy = (y2-y1)
        distance = math.hypot(dx,dy)
        return distance
        
    #def lonlat_2_xy(self, lat, lon, lat0, lon0):
    #    R=6373000
    #    x=R*(math.radians(lon) - math.radians(lon0))*math.cos((math.radians(lat) + math.radians(lat0))/2)
    #    y=R*(math.radians(lat) - math.radians(lat0))
    #    return x, y

    def pathpoint(self, x1, y1, xpath, ypath, path_dir):
        Kpct=0.4
        theta=self.r_heading
        dx=x1-xpath
        dy=y1-ypath
        eit=dx*math.cos(path_dir)+dy*math.sin(path_dir)
        ect=dx*math.sin(path_dir)-dy*math.cos(path_dir)
        headingerr=path_dir+Kpct*ect-theta
        e_dist=eit
        return e_dist, headingerr, eit, ect
        
    def path(self, X, Y, t, mode): #define path, path tangent, and any time based parametrics here
    	if mode==4:
    	    y=0.0
    	    x=X
    	    dy=0.0
    	    dx=1.0
    	if mode==5:
    	    y=0.0
    	    x=t
    	    dy=0.0
    	    dx=1.0
    	path_dir=math.atan2(dy,dx)
    	return x,y,path_dir
        
    def joy_callback(self, msg):
        self.j_lx = msg.axes[self.lx_axisN]
        self.j_az = msg.axes[self.az_axisN] 
        press=msg.buttons[0]
        if press and not self.button:
            self.ctrl_mode += 1
            if self.ctrl_mode>6:
            	self.ctrl_mode = 1
            self.get_logger().info(f"Control mode: {self.ctrl_mode}")
        self.button=press
        
    def leader_callback(self, msg):   
    	self.leadx=msg.x
    	self.leady=msg.y
    	self.lead_hdg=self.get_bearing(self.curx, self.cury, self.leadx, self.leady)
    	self.d2=self.get_distance(self.curx, self.cury, self.leadx, self.leady)
    	#self.t2=self.get_clock().now()
    	#self.dt=(self.t2-self.t1).nanoseconds/1e9
    	#if self.d1 is not None and self.dt>0:
    	#    dd=self.d2-self.d1
    	#    self.v=abs(dd/self.dt)
    	#self.d1=self.d2
    	#self.t1=self.t2
    	
    def csv_record(self):
    	self.csv_writer.writerow([self.ctrl_mode,self.testtime,self.curx,self.cury,self.r_heading,self.ulx,self.uaz,self.e_d,self.e_h,self.Eit,self.Ect,self.leadx,self.leady,self.lead_hdg])

    		

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    minimal_subscriber.csv_file.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
