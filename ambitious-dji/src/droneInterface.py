import rospy, sys, subprocess, json, os, time, threading
from dji_osdk_ros.msg import (
    WaypointV2InitSetting,
    WaypointV2,
    WaypointV2MissionStatePush
)
from dji_osdk_ros.srv import (
    PauseWaypointV2Mission,
    ResumeWaypointV2Mission,
    ObtainControlAuthority,
    GetDroneType,
    SubscribeWaypointV2Event,
    SubscribeWaypointV2State,
    InitWaypointV2Setting,
    UploadWaypointV2Mission,
    StartWaypointV2Mission,
    SetLocalPosRef,
    SetCurrentAircraftLocAsHomePoint,
    FlightTaskControl,
)
from std_msgs.msg import UInt8
from sensor_msgs.msg import BatteryState, NavSatFix
from geometry_msgs.msg import PointStamped
from numpy import cos, pi
from agents import Agent
from MQTTtoDrone import MQTTClient
import paho.mqtt.client as mqtt 
import numpy as np

RATE = 2

class DroneInterface():
    def __init__(self):
        super().__init__()

        self.mqtt_initialized = False
        self.ros_initialized = False
        
        self.agent = Agent()
        self.mqtt = MQTTClient(self)

        self.ros_init()

        self.status: int = 0
        self.battery: int = 999
        self.mission: list = []
        self.gps_location = NavSatFix()
        self.localPose = PointStamped()
        self.maxSpeed = 2
        self.alt = 10
        


    def ros_init(self):

        self.agent.ros_services = {
            'pause': rospy.ServiceProxy(f"/dji_osdk_ros/waypointV2_pauseMission", PauseWaypointV2Mission),
            'resume': rospy.ServiceProxy(f"/dji_osdk_ros/waypointV2_resumeMission", ResumeWaypointV2Mission),
            'goHome': rospy.ServiceProxy(f"/flight_task_control", FlightTaskControl),
            'auth': rospy.ServiceProxy(f"/obtain_release_control_authority", ObtainControlAuthority),
            'dronetype': rospy.ServiceProxy(f"/get_drone_type", GetDroneType),
            'substate': rospy.ServiceProxy(f"/dji_osdk_ros/waypointV2_subscribeMissionState", SubscribeWaypointV2State),
            'subevent': rospy.ServiceProxy(f"/dji_osdk_ros/waypointV2_subscribeMissionEvent", SubscribeWaypointV2Event),
            'init_setting': rospy.ServiceProxy(f"/dji_osdk_ros/waypointV2_initSetting", InitWaypointV2Setting),
            'upload_mission': rospy.ServiceProxy(f"/dji_osdk_ros/waypointV2_uploadMission", UploadWaypointV2Mission),
            'start_mission': rospy.ServiceProxy(f"/dji_osdk_ros/waypointV2_startMission", StartWaypointV2Mission),
            'set_local_ref': rospy.ServiceProxy(f"/set_local_pos_reference", SetLocalPosRef),
            'set_curr_pos_as_home': rospy.ServiceProxy(f"/set_current_aircraft_point_as_home", SetCurrentAircraftLocAsHomePoint)
            }
        
        self.agent.ros_topics = {
            'flight_status': rospy.Subscriber("/dji_osdk_ros/flight_status", UInt8, self.status_callback),
            'battery_status': rospy.Subscriber("/dji_osdk_ros/battery_state", BatteryState, self.battery_callback),
            'gps_position' : rospy.Subscriber("/dji_osdk_ros/gps_position", NavSatFix, self.gps_callback),
            'local_position' : rospy.Subscriber("/dji_osdk_ros/local_position", PointStamped, self.localPose_callback)
        }
        
        try:
            if self.agent.ros_services['substate'](True):
                print("Mission state topic enabled")
                
            else:
                print("Failed to enable mission state topic ")
        except:
            rospy.logerr("Mission subscription service not available!")
        
        try:

            if self.agent.ros_services['set_local_ref']():
                print("Local reference set!")
        except:
            rospy.logerr("Set local reference service not available!")

        self.ros_initialized = True

    def send_path(self, agent):
        
        if not self.mqtt.mqtt_initialized:
            return False, "MQTT not initialized!"
        
        if not self.ros_initialized:
            return False, "ROS not initialized!"

        if len(agent.waypoints) < 1:
            return False, "No waypoints"
        
        wplist = []
        for i in range(len(agent.waypoints)):
            wplist += [agent.waypoints[i]]
        try:
            
            # if agent.ros_services['dronetype']().drone_type != 2:
            #     rospy.loginfo("Drone type must be M300")
            #     return False, "Drone type must be M300"

            # ===================================== Error =====================================
            mission = []
            for i, wp in enumerate(wplist):
                mission.append(self.create_wpv2(wp))
            

            mission.insert(
                0,
                self.create_wpv2({
                    'lat': self.gps_location.latitude,
                    'lng': self.gps_location.longitude,
                    'alt': wplist[0]['alt'],
                    'heading': 0
                })
            )
            
            self.mission = mission
            print(self.mission)

            #Set pos a locale reference
            if agent.ros_services['set_local_ref']():
                rospy.loginfo("Local reference at aircraft position set")
            else:
                rospy.loginfo("Failed to set aircraft position as local reference")

            #Set Home point
            if agent.ros_services['set_curr_pos_as_home']():
                rospy.loginfo("Current aircraft position set to home")
            else:
                rospy.loginfo("Failed to set current aircraft position as home")

            # ================================== Init Setting ==================================
            polygonNum = len(mission)
            radius = 250
            actionNum = 0
            setting = WaypointV2InitSetting()
            setting.repeatTimes = 1
            setting.finishedAction = WaypointV2InitSetting.DJIWaypointV2MissionFinishedGoHome
            setting.maxFlightSpeed = 2
            setting.autoFlightSpeed = 2
            setting.exitMissionOnRCSignalLost = 1
            setting.gotoFirstWaypointMode = WaypointV2InitSetting.DJIWaypointV2MissionGotoFirstWaypointModePointToPoint
            setting.mission = mission
            setting.missTotalLen = polygonNum
            
            if agent.ros_services['init_setting'](setting, polygonNum, radius, actionNum):
                rospy.loginfo("Init setting success")
            else:
                rospy.logerr("Init setting failed")
                return False, "Init setting failed"

            # ================================== Upload Mission ==================================
            if agent.ros_services['upload_mission']():
                rospy.loginfo("Upload mission success")
            else:
                rospy.logerr("Upload mission failed")
                return False, "Upload mission failed"

            # ================================== Start Mission ==================================
            if agent.ros_services['start_mission']():
                rospy.loginfo("Start mission success")
            else:
                rospy.logerr("Start mission failed")
                return False, "Upload mission failed"

        except rospy.service.ServiceException as e:
            return False, str(e)
        except KeyError as e:
            print(f"IndexError: {e}\n\t{agent.ros_services}")

        return True, "Success"
    
    #Creates the waypoints for the mission
    def create_wpv2(self, wp):
        res = WaypointV2()
        res.waypointType = WaypointV2.DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop
        res.headingMode = WaypointV2.DJIWaypointV2HeadingModeAuto
        # res.config.useLocalCruiseVel = 0
        # res.config.useLocalMaxVel = 0
        res.dampingDistance = 40
        res.heading = 0
        res.turnMode = WaypointV2.DJIWaypointV2TurnModeClockwise
        res.positionX = 0
        res.positionY = 0
        res.positionZ = 0
        res.maxFlightSpeed = 2
        res.autoFlightSpeed = 2
        res.latitude = wp['lat'] * pi / 180
        res.longitude = wp['lng'] * pi / 180
        res.relativeHeight = wp['alt']

        return res

    #Function gets triggered when data is recieved from MQTT    
    def send_path_trigger(self):
        code, reason = self.send_path(self.agent)
        if not code:
            rospy.logerr(f'Failed to send path and start agent: {reason}')


    #Callbacks
    def status_callback(self, msg):
        self.status = msg.data
        return self.status

    def battery_callback(self, msg):
        self.battery = msg.percentage
        return self.battery

    def gps_callback(self, msg):
        self.gps_location = msg

    def localPose_callback(self, msg):
        self.localPose = msg

    #Run node
    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.mqtt.publish_to_mqtt(self.mqtt.dronetopic, self.gps_location)
            rate.sleep()
        self.mqtt_client.loop_stop() 


def main(args=None):
    rospy.init_node('ambitious_dji')
    interface = DroneInterface()
    interface.run()
 
