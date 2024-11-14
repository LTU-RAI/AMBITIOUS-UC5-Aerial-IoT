import paho.mqtt.client as mqtt
import json, rospy

class MQTTClient():
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.dronetopic = "ambitious/aida/uc5/dji-1/telemetry"
        self.alarmtopic = "ambitious/aida/uc5/dji-1/alarm"

        self.mqtt_client = mqtt.Client()
        
        self.mqtt_client.username_pw_set("ltu-user", "tV21_4gakauy-15352")
        self.mqtt_client.connect("ambitious.ddns.net", 1883, 60)
        self.mqtt_client.subscribe(self.alarmtopic)
        
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        self.mqtt_client.loop_start()
     

        


    def on_mqtt_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"Connected to MQTT broker with result code {rc}")
        if rc == 0:
            self.mqtt_initialized = True

    def on_mqtt_message(self, client, userdata, msg):
        rospy.loginfo(f"Received MQTT message on {msg.topic}")
        payload = msg.payload.decode('utf-8')
        try:
            self.process_alarm(payload)
        except json.JSONDecodeError:
            rospy.logwarn("Received non-JSON MQTT message")

    def publish_to_mqtt(self, topic, msg):
        navsatfix_data = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": self.parent.localPose.point.z,
            "Battery" : self.parent.battery,
            "Status" : self.parent.status
        }
        self.mqtt_client.publish(topic, json.dumps(navsatfix_data))

    def process_alarm(self, payload):
        data = json.loads(payload)
        
        for idx, sensor_id in enumerate(data["sensors"]["sensor_id"]):
            wp = {
                "sensor_id": sensor_id,
                "lat": float(data["sensors"]["sensor_gnss"][idx].get("latitude")),
                "lng": float(data["sensors"]["sensor_gnss"][idx].get("longitude")),
                "alt": float(data["sensors"]["sensor_alt"][idx])
            }
            self.parent.agent.waypoints[idx] = wp
        
        self.parent.send_path_trigger()
