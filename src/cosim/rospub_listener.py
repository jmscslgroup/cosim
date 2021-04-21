#!/usr/bin/env python
# coding: utf-8

# Author : Rahul Bhadani
# Initial Date: Apr 20, 2021

#   Permission is hereby granted, free of charge, to any person obtaining
#   a copy of this software and associated documentation files
#   (the "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject
#   to the following conditions:

#   The above copyright notice and this permission notice shall be
#   included in all copies or substantial portions of the Software.

#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
#   ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
#   TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#   PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
#   SHALL THE AUTHORS, COPYRIGHT HOLDERS OR ARIZONA BOARD OF REGENTS
#   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
#   AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
#   OR OTHER DEALINGS IN THE SOFTWARE.


__author__ = 'Rahul Bhadani'
__email__  = 'rahulbhadani@email.arizona.edu'

import traci
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64, UInt8
from cosim.msg import Sumocar

class rospub_listener(traci.StepListener):
    """
    A subclass for implementing listeners that computes and publishes speed information of all vehicles available.

    """
    
    def __init__(self) -> None:
        
        # ROS publisher to publish Sumo real_time factor
        self.rtf_pub = rospy.Publisher('sumo_rtf', Float64, queue_size=1)
        self.car_pub = rospy.Publisher('cardata', Sumocar, queue_size=1)
        self.rtf_msg = Float64()
        self.car_msg = Sumocar()             
        self.tstep = 1
        self.last_clocktime =traci.time.time() # get the cloeck time
        self.last_simtime = traci.simulation.getTime() # get the simulation time in seconds
        self.current_reatimefactor = 0.0
        self.delta_clock = None
        self.delta_sim = None

        self.next_vel = 0.0

        super().__init__()

    def set_speedcb(self, data):
        self.next_vel = data.linear.x
        

    def step(self, t):
        """
        This function will be called in every simulation step

        """

        # Get the IDs of all vehicles
        veh_ids = traci.vehicle.getIDList()
        #print("Vehicle IDs={}".format(veh_ids))
        self.car_msg.id = list(veh_ids)
        speed =[]
        acceleration = []
        distance = []
        x = []
        y = []
        angle = []
        for vids in veh_ids:
            
            acceleration.append(traci.vehicle.getAcceleration(vehID = vids))
            speed.append(traci.vehicle.getSpeed(vehID = vids))
            distance.append(traci.vehicle.getDistance(vehID = vids))
            angle.append(traci.vehicle.getAngle(vehID = vids))
            pos =traci.vehicle.getPosition(vehID = vids)
            x.append(pos[0])
            y.append(pos[1])
            
        self.car_msg.angle = angle
        self.car_msg.acceleration = acceleration
        self.car_msg.distance = distance
        self.car_msg.x = x
        self.car_msg.y = y
        self.car_msg.speed = speed
        self.car_pub.publish(self.car_msg)

        #print('tstep={}'.format(self.tstep))
        self.tstep += 1

        current_clocktime =traci.time.time()
        current_simtime =  traci.simulation.getTime() 
        self.delta_clock = current_clocktime  - self.last_clocktime
        self.delta_sim = current_simtime- self.last_simtime

        self.last_clocktime = current_clocktime
        self.last_simtime = current_simtime

        # if there is no change in clock time, keep rtf to last value
        if self.delta_clock != 0:
            self.current_reatimefactor = self.delta_sim/self.delta_clock

        self.rtf_msg.data = self.current_reatimefactor
        self.rtf_pub.publish(self.rtf_msg.data)

        for vids in veh_ids:
            traci.vehicle.setSpeed(vids, self.next_vel)
        

        return True