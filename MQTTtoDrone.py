import paho.mqtt.client as mqtt
import json
import time
from datetime import datetime
from typing import List, Optional
import threading

# MQTT Broker details with authentication
broker = 'ambitious.ddns.net'
port = 1883
keepalive = 60
client_id = "DJI-1"
username = "ltu-user"
password = "xxx"

# Topics
TOPIC_ALARM = "ambitious/aida/uc5/dji-1/alarm"
TOPIC_TELEMETRY = "ambitious/aida/uc5/dji-1/telemetry"
refresh = 5  # Telemetry refresh rate in seconds
running = True

#Example of message from this topic: 
#{"timestamp": "2022-10-29 14:17:00.563885", "sensors": {"sensor_id": ["11234", "34521","....."], "sensor_gnss": [{"latitude":"45.0044316","longitude": "10.4691617"}, {"latitude": "45.0044215", "longitude": "10.4691345"}, {"latitude": ".....", "longitude": "....."}], "sensor_alt": ["34.23","12.89","...."], "sensor_int": ["UWB","Wi-Fi","..."]}}

#Example of telemetry message sent from the drone
#{"timestamp": "2022-10-29 14:17:00.563885", "drone_id": "DJI-1", "telemetry": {"mode": "LOITER", "status": "STANDBY", "battery": "24.529", "gps_param": {"latitude": "45.0044316", "longitude": "10.4691617", "altitude": "13.64", "groundspeed": "0.0469114035367965", "head": "14", "hdop": "63", "vdop": "83", "satellites": "16", "fix_type": "3"}}}

# Define data structure
class SensorData:
    def __init__(self, sensor_id: str, latitude: Optional[float], longitude: Optional[float], altitude: Optional[float], interface: Optional[str]):
        self.sensor_id = sensor_id
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.interface = interface

# Publish JSON telemetry
def publish_json_message(client):
    while running:
        message = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
            "drone_id": "DJI-1",
            "telemetry": {
                "mode": "LOITER",
                "status": "STANDBY",
                "battery": "24.529",
                "gps_param": {
                    "latitude": "45.0044316",
                    "longitude": "10.4691617",
                    "altitude": "13.64",
                    "groundspeed": "0.089",
                    "head": "14",
                    "hdop": "63",
                    "vdop": "83",
                    "satellites": "16",
                    "fix_type": "3"
                }
            }
        }
        client.publish(TOPIC_TELEMETRY, json.dumps(message))
        time.sleep(refresh)

# Callback functions
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC_ALARM)
    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()
    print(f"Received message from {topic}: {payload}")
    
    if topic == TOPIC_ALARM:
        process_alarm(payload)

# Processing functions
def process_alarm(payload):
    try:
        data = json.loads(payload)
        sensors_data = []
        for idx, sensor_id in enumerate(data["sensors"]["sensor_id"]):
            latitude = data["sensors"]["sensor_gnss"][idx].get("latitude")
            longitude = data["sensors"]["sensor_gnss"][idx].get("longitude")
            altitude = data["sensors"]["sensor_alt"][idx]
            interface = data["sensors"]["sensor_int"][idx]
            latitude = float(latitude) if latitude and latitude != "....." else None
            longitude = float(longitude) if longitude and longitude != "....." else None
            altitude = float(altitude) if altitude and altitude != "...." else None
            sensors_data.append(SensorData(sensor_id, latitude, longitude, altitude, interface))

        for sensor in sensors_data:
            if sensor.latitude and sensor.longitude:
                print(f"Sensor {sensor.sensor_id} at ({sensor.latitude}, {sensor.longitude})")

    except json.JSONDecodeError as e:
        print(f"Failed to decode alarm message: {payload}, error: {e}")


# Initialize MQTT client
client = mqtt.Client(client_id=client_id)
client.username_pw_set(username, password)
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker, port, keepalive)

# Start MQTT loop in separate thread
client.loop_start()

# Start publishing thread
publish_thread = threading.Thread(target=publish_json_message, args=(client,))
publish_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Disconnecting...")
    running = False
    publish_thread.join()
    client.loop_stop()
    client.disconnect()
