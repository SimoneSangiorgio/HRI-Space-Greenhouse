#!/usr/bin/env python
import rospy
import random
import json
from std_msgs.msg import String

# --- Constants ---
NODE_NAME = 'environment_simulator'
PUBLISH_RATE_HZ = 1
TOPIC_ROBOT_SENSORS = '/robot/sensors'
TOPIC_SERRA_STATE = '/serra/state'

# --- Simulation Parameters ---
BATTERY_START = 100.0
BATTERY_DEPLETION = 0.02
ENERGY_START = 85.0
ENERGY_DEPLETION = 0.01

# --- Global State ---
current_battery = BATTERY_START
current_solar_energy = ENERGY_START

def generate_robot_sensors_data():
    global current_battery
    current_battery = max(0, current_battery - BATTERY_DEPLETION)
    return {
        'battery_percent': round(current_battery, 2),
        'temperature_celsius': round(random.uniform(21.5, 22.5), 2),
        'humidity_percent': round(random.uniform(55.0, 60.0), 2),
        'co2_ppm': random.randint(450, 500)
    }

def generate_serra_state_data():
    global current_solar_energy
    current_solar_energy = max(0, current_solar_energy - ENERGY_DEPLETION)
    return {
        'water_tank_1_percent': 50, # Static for simplicity in this example
        'water_tank_2_percent': 20, # Static
        'stored_solar_energy_percent': round(current_solar_energy, 2),
        'plants_info': [
            {'bay_id': 1, 'plant_name': 'Basil', 'soil_moisture_percent': 18, 'leaf_appearance': 'seems dry'},
            {'bay_id': 2, 'plant_name': 'Lemon', 'soil_moisture_percent': 90, 'leaf_appearance': 'seems weak'},
            {'bay_id': 3, 'plant_name': 'Moss', 'soil_moisture_percent': 45, 'leaf_appearance': 'seems okay'}
        ]
    }

def environment_publisher():
    rospy.init_node(NODE_NAME, anonymous=True)
    robot_pub = rospy.Publisher(TOPIC_ROBOT_SENSORS, String, queue_size=10)
    serra_pub = rospy.Publisher(TOPIC_SERRA_STATE, String, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE_HZ)
    rospy.loginfo(f"'{NODE_NAME}' started.")

    while not rospy.is_shutdown():
        robot_data = generate_robot_sensors_data()
        serra_data = generate_serra_state_data()
        robot_pub.publish(json.dumps(robot_data))
        serra_pub.publish(json.dumps(serra_data))
        rate.sleep()

if __name__ == '__main__':
    try:
        environment_publisher()
    except rospy.ROSInterruptException:
        pass