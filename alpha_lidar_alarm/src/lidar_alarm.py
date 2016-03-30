#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool

class LidarAlarm():
    # set alarm if anything is within 0.5m of the front of robot
    MIN_SAFE_DISTANCE = 1 
    
    # set what percentages of pings must be < than MIN_SAFE_DISTANCE
    # for our alarm to return a warning
    ALARM_TRIP_PERCENTAGE = .1

    # Set what width (in pings) we want to scan for obstacles
    ALARM_SCAN_WIDTH = 90
    
    # these values to be set within the laser callback
    ping_dist_in_front_ = 3  # global var to hold length of a SINGLE LIDAR ping--in front
    ping_index_ = -1  # NOT real; callback will have to find this
    angle_min_ = 0.0
    angle_max_ = 0.0
    angle_increment_ = 0.0
    range_min_ = 0.0
    range_max_ = 0.0
    laser_alarm = False
    min_index = -1
    max_index = -1

    lidar_alarm_publisher = rospy.Publisher('lidar_alarm', Bool, queue_size=1)
    lidar_dist_publisher_ = rospy.Publisher('lidar_dist', Float32, queue_size=1)

    def setScanProperties(self, laser_scan) :
        self.angle_min_ = laser_scan.angle_min
        self.angle_max_ = laser_scan.angle_max
        self.angle_increment_ = laser_scan.angle_increment
        self.range_min_ = laser_scan.range_min
        self.range_max_ = laser_scan.range_max

        rospy.loginfo("Minimum angle is %f", self.angle_min_)
        rospy.loginfo("Maximum angle is %f", self.angle_max_)
        rospy.loginfo("Angle increment is %f", self.angle_increment_)
        rospy.loginfo("Minimum range is %f", self.range_min_)
        rospy.loginfo("Maximum range is %f", self.range_max_)
        rospy.loginfo("Total number of pings is %d", len(laser_scan.ranges))

    def laserCallback(self, laser_scan) :
        
        if (self.ping_index_ < 0) : 
            self.setScanProperties(laser_scan)

            # Assume min/max angle are symmetrical and set up evenly on robot
            # This is the ping index that is straight ahead
            self.ping_index_ = int((0 - self.angle_min_) / self.angle_increment_)
            rospy.loginfo("LIDAR setup: ping_index = %d", self.ping_index_)
      
            # Set the beginning and end of our scan zone
            self.min_index = self.ping_index_ - int(self.ALARM_SCAN_WIDTH / 2)
            self.max_index = self.ping_index_ + int(self.ALARM_SCAN_WIDTH / 2)
            rospy.loginfo("min index is %d", self.min_index)
            rospy.loginfo("max index is %d", self.max_index)

        count = 0
        numPings = 0
        average = 0
        averageDanger = 0
        for  x in range(self.min_index, self.max_index) :
            if laser_scan.ranges[x] > self.range_min_ :
                if laser_scan.ranges[x] < self.range_max_ :
                    numPings += 1
                    average += laser_scan.ranges[x]
                    if laser_scan.ranges[x] < self.MIN_SAFE_DISTANCE :
                        count += 1
                        averageDanger += laser_scan.ranges[x]


        #rospy.loginfo("%d valid lidar pings with average distance %f", numPings, float(average) / float(numPings))
        #if count is not 0:
            #rospy.loginfo("%d dangerous pings with average distance %f", count, float(averageDanger) / float(count))

        if count > (numPings * self.ALARM_TRIP_PERCENTAGE) :
            self.laser_alarm = True
            rospy.loginfo("LIDAR ALARM! DANGER!") 
        else :
            self.laser_alarm = False

        lidar_alarm_msg = Bool()
        lidar_alarm_msg.data = self.laser_alarm
        self.lidar_alarm_publisher.publish(lidar_alarm_msg)

        lidar_dist_msg = Float32()
        lidar_dist_msg.data = self.ping_dist_in_front_
        self.lidar_dist_publisher_.publish(lidar_dist_msg)

def main():
    rospy.init_node('my_lidar_alarm')
    alarm = LidarAlarm()

    # create a Subscriber object and have it subscribe to the lidar topic
    lidar_subscriber = rospy.Subscriber('base_laser1_scan', LaserScan, alarm.laserCallback)
    rospy.spin()
    # this is essentially a "while(1)" statement, except it
    # forces refreshing wakeups upon new data arrival
    # main program essentially hangs here, but it must stay alive to keep the
    # callback function alive

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException :
        pass
