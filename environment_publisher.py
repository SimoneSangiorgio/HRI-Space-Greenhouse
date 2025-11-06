#!/usr.bin/env python

import rospy
import random
import json
from std_msgs.msg import String

# --- Constants and Configuration ---

# ROS Configuration
NODE_NAME = 'environment_simulator'
PUBLISH_RATE_HZ = 1

# Topic Names
TOPIC_SENSORS = '/robot/sensors'
# TOPIC_UFFICIO eliminato
TOPIC_SERRA = '/serra/state' # Rinominato per riflettere meglio il contenuto

# --- Simulation Parameters ---

# Lighting (costante)
LIGHTING_LUX_CONSTANT = 5500

# Temperature Simulation
TEMP_MIN, TEMP_MAX = 20.0, 26.0
TEMP_INITIAL = 22.5
TEMP_MAX_STEP = 0.05
PULL_FACTOR = 0.01

# Humidity Simulation
HUMIDITY_MIN, HUMIDITY_MAX = 50.0, 70.0
HUMIDITY_INITIAL = 60.0
HUMIDITY_MAX_STEP = 0.2

# Oxygen Simulation
OX_MIN, OX_MAX = 20.0, 21.0
OX_INITIAL = 20.8
OX_MAX_STEP = 0.05      
 
# CO2 Simulation
CO2_MIN, CO2_MAX = 400, 800
CO2_INITIAL = 450
CO2_MAX_STEP = 2

# Battery Simulation (del robot)
BATTERY_START_LEVEL = 100.0
BATTERY_DEPLETION_RATE = 0.01

# NUOVO: Solar Energy Simulation (della serra)
ENERGY_START_LEVEL = 85.0
ENERGY_DEPLETION_RATE = 0.001 # Variazione al secondo

# --- Global State ---
current_battery = BATTERY_START_LEVEL
current_temperature = TEMP_INITIAL
current_humidity = HUMIDITY_INITIAL
current_co2 = CO2_INITIAL
current_ox = OX_INITIAL
current_solar_energy = ENERGY_START_LEVEL # Nuova variabile di stato per l'energia

def update_realistic_value(current_val, min_val, max_val, max_step, pull_factor):
    """Calcola il prossimo valore di un sensore in modo realistico."""
    center = (min_val + max_val) / 2
    step = random.uniform(-max_step, max_step)
    pull = (center - current_val) * pull_factor
    new_val = current_val + step + pull
    new_val = max(min_val, min(max_val, new_val))
    return round(new_val, 2)

def generate_sensor_data():
    """Genera i dati dei sensori a bordo del robot."""
    global current_battery, current_temperature, current_humidity, current_co2, current_ox
    
    current_temperature = update_realistic_value(current_temperature, TEMP_MIN, TEMP_MAX, TEMP_MAX_STEP, PULL_FACTOR)
    current_humidity = update_realistic_value(current_humidity, HUMIDITY_MIN, HUMIDITY_MAX, HUMIDITY_MAX_STEP, PULL_FACTOR)
    current_co2 = update_realistic_value(current_co2, CO2_MIN, CO2_MAX, CO2_MAX_STEP, PULL_FACTOR)
    current_ox = update_realistic_value(current_ox, OX_MIN, OX_MAX, OX_MAX_STEP, PULL_FACTOR)
    
    current_battery = max(0, current_battery - BATTERY_DEPLETION_RATE)
    
    sensors_data = {
        'temperature_celsius': current_temperature,
        'humidity_percent': current_humidity,
        'lighting_lux': LIGHTING_LUX_CONSTANT,
        'oxygen_percent': current_ox,
        'co2_ppm': int(current_co2),
        'battery_percent': round(current_battery, 2)
    }
    return sensors_data

def generate_serra_data():
    """
    Genera dati combinati sullo stato della serra: piante, risorse idriche ed energetiche.
    """
    global current_solar_energy
    
    # Aggiorna l'energia solare immagazzinata
    current_solar_energy = max(0, current_solar_energy - ENERGY_DEPLETION_RATE)

    serra_data = {
        # Dati migrati da "ufficio"
        'water_tank_1_percent': 50,
        'water_tank_2_percent': 20,
        # Nuovo dato sull'energia
        'stored_solar_energy_percent': round(current_solar_energy,2),
        # Dati originali sulle piante
        'plants_info': [
            {'bay_id': 1, 'plant_name': 'basil', 'soil_moisture_percent': 20, 'leaf_appearance': 'seems all right'},
            {'bay_id': 2, 'plant_name': 'lemon', 'soil_moisture_percent': 90, 'leaf_appearance': 'seems weak'},
            {'bay_id': 3, 'plant_name': 'moss', 'soil_moisture_percent': 10, 'leaf_appearance': 'seems brown'}
        ]
    }
    return serra_data

def environment_publisher():
    """
    Nodo ROS principale che pubblica lo stato dei sensori del robot e lo stato della serra.
    """
    rospy.init_node(NODE_NAME, anonymous=True)

    sensors_pub = rospy.Publisher(TOPIC_SENSORS, String, queue_size=10)
    serra_pub = rospy.Publisher(TOPIC_SERRA, String, queue_size=10)

    rate = rospy.Rate(PUBLISH_RATE_HZ)
    rospy.loginfo(f"'{NODE_NAME}' started. Publishing simulated data...")

    while not rospy.is_shutdown():
        # Genera i due dizionari di dati
        sensors_dict = generate_sensor_data()
        serra_dict = generate_serra_data()

        # Pubblica i dati sui rispettivi topic
        sensors_pub.publish(json.dumps(sensors_dict))
        serra_pub.publish(json.dumps(serra_dict))
        
        rospy.loginfo(f"Published Sensors: {json.dumps(sensors_dict)}")
        #rospy.loginfo(f"Published Serra State: {json.dumps(serra_dict)}")
        #rospy.loginfo("---")

        rate.sleep()

if __name__ == '__main__':
    try:
        environment_publisher()
    except rospy.ROSInterruptException:
        pass