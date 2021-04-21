#!/usr/bin/env python
# coding: utf-8

# Author : Rahul Bhadani
# Initial Date: Apr 19, 2021

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

import rospy
import traci
import os
import sys
import subprocess
from xml.sax.handler import ContentHandler
from xml.sax import make_parser

import xmltodict
from xml.dom import minidom
import datetime
import time
import networkx as nx
import matplotlib.pyplot as plt
import random
import rospkg, rostopic, roslaunch, rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from cosim import rospub_listener

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
package_path = rospack.get_path('cosim')
assets = package_path + '/assets'
        
def get_intersection_net(dt):
    '''
    Function to create net xml and returns its path after creation
    '''
    edge_length = 300

    init_x = 0
    init_y = -edge_length
    node_id = 1
    edge_id = 1
   
    junction_offset = -6.0
    # Creare Node XML
    root = minidom.Document()
    xml = root.createElement('nodes')
    root.appendChild(xml)
    while init_y >= -edge_length*5:
        node_child = root.createElement('node')
        node_child.setAttribute('id', '{}'.format(node_id))
        node_child.setAttribute('x', '{}'.format(init_x))
        node_child.setAttribute('y', '{}'.format(init_y+junction_offset))
        xml.appendChild(node_child)
        node_id = node_id + 1
        init_y = init_y - edge_length

    init_y = edge_length
    junction_offset = 6.0
    while init_y <= edge_length*5:
        node_child = root.createElement('node')
        node_child.setAttribute('id', '{}'.format(node_id))
        node_child.setAttribute('x', '{}'.format(init_x))
        node_child.setAttribute('y', '{}'.format(init_y+junction_offset))
        xml.appendChild(node_child)
        node_id = node_id + 1
        init_y = init_y + edge_length

    # Draw a road perpendicular to the above from middle
    init_px = -edge_length
    init_py = 0
    pnode_id = node_id
    junction_offset = -6.0
    while init_px >= -edge_length*5:
        node_child = root.createElement('node')
        node_child.setAttribute('id', '{}'.format(pnode_id))
        node_child.setAttribute('x', '{}'.format(init_px+junction_offset))
        node_child.setAttribute('y', '{}'.format(init_py))
        xml.appendChild(node_child)
        pnode_id = pnode_id + 1
        init_px = init_px - edge_length

    init_px = edge_length
    junction_offset = 6.0
    while init_px <= edge_length*5:
        node_child = root.createElement('node')
        node_child.setAttribute('id', '{}'.format(pnode_id))
        node_child.setAttribute('x', '{}'.format(init_px+junction_offset))
        node_child.setAttribute('y', '{}'.format(init_py))
        xml.appendChild(node_child)
        pnode_id = pnode_id + 1
        init_px = init_px + edge_length

    node_child = root.createElement('node')
    node_child.setAttribute('id', 'cluster00')
    node_child.setAttribute('x', '0')
    node_child.setAttribute('y', '0')
    node_child.setAttribute('type', 'traffic_light')
    node_child.setAttribute('tl', '01')
    xml.appendChild(node_child)

    xml_str = root.toprettyxml(indent ="\t")
    print(xml_str)

    nodexml_file = "/tmp/tsmo_{}.nod.xml".format(dt)
    with open(nodexml_file, "w") as f:
        f.write(xml_str)
    
    # Create Edge XML
    root = minidom.Document()
    xml = root.createElement('edges')
    root.appendChild(xml)

    edge_cnt = 1

    nid = 1
    # Vertical Edges
    while edge_cnt <= 16:
        edge_child = root.createElement('edge')
        edge_child.setAttribute('id', '{}'.format(edge_cnt))
        edge_child.setAttribute('from', '{}'.format(nid))
        edge_child.setAttribute('to', '{}'.format(nid+1))
        edge_child.setAttribute('numLanes', '2')
        edge_child.setAttribute('speed', '0.5')
        xml.appendChild(edge_child)

        edge_child = root.createElement('edge')
        edge_child.setAttribute('id', '{}'.format(-edge_cnt))
        edge_child.setAttribute('to', '{}'.format(nid))
        edge_child.setAttribute('from', '{}'.format(nid+1))
        edge_child.setAttribute('numLanes', '2')
        edge_child.setAttribute('speed', '0.5')
        xml.appendChild(edge_child)
        edge_cnt =edge_cnt +1
        nid = nid + 1
        if nid%5 == 0:
            nid = nid + 1

    edge_child = root.createElement('edge')
    edge_child.setAttribute('id', '{}'.format(edge_cnt))
    edge_child.setAttribute('to', 'cluster00')
    edge_child.setAttribute('from', '{}'.format(1))
    edge_child.setAttribute('numLanes', '2')
    edge_child.setAttribute('speed', '0.5')
    xml.appendChild(edge_child)

    edge_child = root.createElement('edge')
    edge_child.setAttribute('id', '{}'.format(-edge_cnt))
    edge_child.setAttribute('from', 'cluster00')
    edge_child.setAttribute('to', '{}'.format(1))
    edge_child.setAttribute('numLanes', '2')
    edge_child.setAttribute('speed', '0.5')
    xml.appendChild(edge_child)

    edge_cnt =edge_cnt +1
    edge_child = root.createElement('edge')
    edge_child.setAttribute('id', '{}'.format(edge_cnt))
    edge_child.setAttribute('to', 'cluster00')
    edge_child.setAttribute('from', '{}'.format(6))
    edge_child.setAttribute('numLanes', '2')
    edge_child.setAttribute('speed', '0.5')
    xml.appendChild(edge_child)

    edge_child = root.createElement('edge')
    edge_child.setAttribute('id', '{}'.format(-edge_cnt))
    edge_child.setAttribute('from', 'cluster00')
    edge_child.setAttribute('to', '{}'.format(6))
    edge_child.setAttribute('numLanes', '2')
    edge_child.setAttribute('speed', '0.5')
    xml.appendChild(edge_child)

    edge_cnt =edge_cnt +1
    edge_child = root.createElement('edge')
    edge_child.setAttribute('id', '{}'.format(edge_cnt))
    edge_child.setAttribute('to', 'cluster00')
    edge_child.setAttribute('from', '{}'.format(11))
    edge_child.setAttribute('numLanes', '2')
    edge_child.setAttribute('speed', '0.5')
    xml.appendChild(edge_child)

    edge_child = root.createElement('edge')
    edge_child.setAttribute('id', '{}'.format(-edge_cnt))
    edge_child.setAttribute('from', 'cluster00')
    edge_child.setAttribute('to', '{}'.format(11))
    edge_child.setAttribute('numLanes', '2')
    edge_child.setAttribute('speed', '0.5')
    xml.appendChild(edge_child)

    edge_cnt =edge_cnt +1
    edge_child = root.createElement('edge')
    edge_child.setAttribute('id', '{}'.format(edge_cnt))
    edge_child.setAttribute('to', 'cluster00')
    edge_child.setAttribute('from', '{}'.format(16))
    edge_child.setAttribute('numLanes', '2')
    edge_child.setAttribute('speed', '0.5')
    xml.appendChild(edge_child)

    edge_child = root.createElement('edge')
    edge_child.setAttribute('id', '{}'.format(-edge_cnt))
    edge_child.setAttribute('from', 'cluster00')
    edge_child.setAttribute('to', '{}'.format(16))
    edge_child.setAttribute('numLanes', '2')
    edge_child.setAttribute('speed', '0.5')
    xml.appendChild(edge_child)

    xml_str = root.toprettyxml(indent ="\t")
    print(xml_str)
    edgexml_file = "/tmp/tsmo_{}.edg.xml".format(dt)
    with open(edgexml_file, "w") as f:
        f.write(xml_str)

    # Now we generate net.xml

    net_xml = "/tmp/tsmo_{}.net.xml".format(dt)
    netconvert_call = ["netconvert", "--node-files", nodexml_file, "--edge-files", edgexml_file,  #"--junctions.internal-link-detail", "40",
    "-o", net_xml]

    netconvert_proc = subprocess.Popen( netconvert_call, stdin=subprocess.PIPE,  stdout=subprocess.PIPE,  stderr=subprocess.PIPE)
    stdout = netconvert_proc.communicate()
    print('Netconvert Output:{}'.format(stdout))

    return net_xml

def get_example_route(dt):
    # Create a custom vehicle type
    root = minidom.Document()
    xml = root.createElement('routes')
    root.appendChild(xml)
    node_child = root.createElement('vType')
    node_child.setAttribute('id', 'toyota')
    node_child.setAttribute('accel', '0.5')
    node_child.setAttribute('decel', '1.5')
    node_child.setAttribute('apparentDecel', '1.5')
    node_child.setAttribute('emergencyDecel', '9.0')
    node_child.setAttribute('vClass', 'passenger')
    node_child.setAttribute('guiShape', "passenger")
    node_child.setAttribute('color', "1,0.5,0.1")
    node_child.setAttribute('imgFile', str(assets) + "/toyota.png")
    xml.appendChild(node_child)
    xml_str = root.toprettyxml(indent ="\t")
    routexml_file = "/tmp/tsmo_{}.rou.xml".format(dt)
    with open(routexml_file, "w") as f:
        f.write(xml_str)

    return routexml_file

def get_example_traffic_signal(dt):
    # additional files
    root = minidom.Document()
    xml = root.createElement('additional')
    root.appendChild(xml)
    tlLogic = root.createElement('tlLogic')
    tlLogic.setAttribute('id', '01')
    tlLogic.setAttribute('type', 'static')
    tlLogic.setAttribute('programID', '111')
    tlLogic.setAttribute('offset', '0')

    phase = root.createElement('phase')
    phase.setAttribute('duration', '1200000')
    phase.setAttribute('state', 'GGGggrrrrrGGGggrrrrr')
    #phase.setAttribute('state', 'GgGgGrrrrrGgGgGrrrrr')
    tlLogic.appendChild(phase)

    phase = root.createElement('phase')
    phase.setAttribute('duration', '600000')
    phase.setAttribute('state', 'yyyyyrrrrryyyyyrrrrr')
    tlLogic.appendChild(phase)

    phase = root.createElement('phase')
    phase.setAttribute('duration', '1200000')
    phase.setAttribute('state', 'rrrrrGGGggrrrrrGGGgg')
    #phase.setAttribute('state', 'rrrrrGgGgGrrrrrGgGgG')
    tlLogic.appendChild(phase)

    phase = root.createElement('phase')
    phase.setAttribute('duration', '600000')
    phase.setAttribute('state', 'rrrrryyyyyrrrrryyyyy')
    tlLogic.appendChild(phase)

    xml.appendChild(tlLogic)

    xml_str = root.toprettyxml(indent ="\t")
    print(xml_str)
    tlLogicxml_file = "/tmp/tsmo_{}.add.xml".format(dt)
    with open(tlLogicxml_file, "w") as f:
        f.write(xml_str)

    return tlLogicxml_file
class sumo_spawn:
    def __init__(self, ns='', n_vehicles=20, vehicle_separation=15, sim_step = 0.05, **kwargs):
        # ROS Part
        self.ns = ns
        self.n_vehicles = n_vehicles
        self.n_vehicles_separation = vehicle_separation
        self.sim_step = sim_step
        rospy.init_node("sum_ros", anonymous=True)
        self.sumo_call =  ['sumo-gui',  "--start", "--step-length", "{}".format(self.sim_step)]

        dt_object = datetime.datetime.fromtimestamp(time.time())
        dt = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')
        
        self.routexml_file = get_example_route(dt)
        self.tlLogicxml_file = get_example_traffic_signal(dt)
        self.net_xml = get_intersection_net(dt)
        
        self.assets = assets
        self.sumo_call.append("--gui-settings-file")
        self.sumo_call.append(str(self.assets) + "/tsmo_gui.xml")
        self.sumo_call.append("--net-file")
        self.sumo_call.append(self.net_xml)
        self.sumo_call.append("--route-files")
        self.sumo_call.append(self.routexml_file)
        self.sumo_call.append("--additional-files")
        self.sumo_call.append(self.tlLogicxml_file)

        print("Sumo call is {}".format(self.sumo_call))
        traci.start(self.sumo_call)
        traci.route.add("trip_h", ["-12", "16"])

        for  i in range(0, self.n_vehicles):
            traci.vehicle.add("newVeh{}".format(i), "trip_h", typeID="toyota", departPos='{:.3f}'.format(200+ i*self.n_vehicles_separation))
            traci.vehicle.setSpeedMode("newVeh{}".format(i), 0)

        r_listener = rospub_listener()
        traci.addStepListener(r_listener)
        rospy.Subscriber('cmd_vel',Twist, r_listener.set_speedcb)



def main(argv):
    ns = rospy.get_namespace() #Retrieve namespace this way appends '/' at the end as well,
    ns = ns[0:-1]

    #step_size = argv[0]

    n_vehicles = 2
    sumo_node = sumo_spawn(ns = ns, n_vehicles = n_vehicles)
    traci.simulationStep()
    while not rospy.is_shutdown():
        traci.simulationStep()

if __name__ == '__main__':
    main(sys.argv[1:])

