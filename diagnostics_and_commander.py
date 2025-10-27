#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
import threading

class DiagnosticsAndCommanderNode:
    def __init__(self):
        """
        Costruttore del nodo. Inizializza le variabili di stato e i subscriber.
        """
        rospy.loginfo("Initializing Diagnostics and Commander Node...")

        # --- Archiviazione dello Stato ---
        # Variabili per memorizzare i dati più recenti dai nuovi topic.
        self.serra_state = {}
        self.robot_sensors = {}
        
        # Lock per la gestione sicura dei dati tra i thread.
        self.data_lock = threading.Lock()

        # --- Subscribers ---
        # Il nodo si sottoscrive ai nuovi topic pubblicati dal simulatore (environment_publisher.py).
        rospy.Subscriber("/serra/state", String, self.serra_state_callback)
        rospy.Subscriber("/robot/sensors", String, self.robot_sensors_callback)
        
        rospy.loginfo("Subscribers created. Waiting for data...")

    # --- Funzioni di Callback ---
    def serra_state_callback(self, msg):
        """
        Eseguita ogni volta che arriva un messaggio su /serra/state.
        """
        with self.data_lock:
            self.serra_state = json.loads(msg.data)
        # rospy.logdebug("Received Serra data: %s", self.serra_state)

    def robot_sensors_callback(self, msg):
        """
        Eseguita ogni volta che arriva un messaggio su /robot/sensors.
        """
        with self.data_lock:
            self.robot_sensors = json.loads(msg.data)
        # rospy.logdebug("Received Robot Sensor data: %s", self.robot_sensors)

    # --- Processore dei Comandi ---
    def process_command(self, command):
        """
        Interpreta i comandi dell'utente e risponde usando i dati memorizzati.
        """
        command = command.lower().strip()

        with self.data_lock: # Blocca i dati per una lettura sicura
            
            # Comando per un report completo sulla serra
            if "status" in command or "report" in command or "serra" in command:
                if not self.serra_state or not self.robot_sensors:
                    print("Non ho ancora ricevuto tutti i dati necessari per un report.")
                else:
                    # Estrai i dati ambientali dai sensori del robot
                    temp = self.robot_sensors.get('temperature_celsius', 'N/A')
                    humidity = self.robot_sensors.get('humidity_percent', 'N/A')
                    
                    # Estrai i dati delle risorse dallo stato della serra
                    energy = self.serra_state.get('stored_solar_energy_percent', 'N/A')
                    tank1 = self.serra_state.get('water_tank_1_percent', 'N/A')
                    tank2 = self.serra_state.get('water_tank_2_percent', 'N/A')

                    print("\n--- Greenhouse Status Report ---")
                    print(f"Ambient: Temperature {temp}°C, Humidity {humidity}%.")
                    print(f"Resources: Solar energy at {energy}%, Water Tank 1 at {tank1}%, Water Tank 2 at {tank2}%.")

                    # Aggiungiamo un'analisi "intelligente" delle piante
                    plants = self.serra_state.get('plants_info', [])
                    if plants:
                        # Trova la pianta con l'umidità del suolo più bassa
                        driest_plant = min(plants, key=lambda p: p['soil_moisture_percent'])
                        print(f"Plant Health: The driest plant is '{driest_plant['plant_name']}' in bay {driest_plant['bay_id']} with {driest_plant['soil_moisture_percent']}% soil moisture.")
                    print("--------------------------------\n")

            # Comando per la batteria del ROBOT
            elif "battery" in command:
                if not self.robot_sensors:
                    print("Non conosco ancora il livello della batteria del robot.")
                else:
                    battery = self.robot_sensors.get('battery_percent', 'N/A')
                    print(f"The robot's battery level is {battery}%.")

            # NUOVO comando per l'energia della SERRA
            elif "energy" in command:
                if not self.serra_state:
                    print("Non conosco ancora il livello di energia della serra.")
                else:
                    energy = self.serra_state.get('stored_solar_energy_percent', 'N/A')
                    print(f"The greenhouse's stored solar energy is at {energy}%.")
            
            # NUOVO comando per visualizzare tutti i sensori
            elif "sensors" in command:
                if not self.robot_sensors:
                    print("Non ho ancora ricevuto dati dai sensori.")
                else:
                    print("Current Robot Sensor Readings:")
                    # Stampa ogni coppia chiave-valore in modo ordinato
                    for key, value in self.robot_sensors.items():
                        print(f"  - {key.replace('_', ' ').title()}: {value}")

            else:
                print("Sorry, I don't understand. Try 'status', 'battery', 'energy', or 'sensors'.")

    def run(self):
        """
        Il ciclo principale del nodo, in attesa di input dall'utente.
        """
        print("\n--- TIAGo Gardener Robot Interface ---")
        print("Enter a command (e.g., 'status', 'battery', 'energy', 'sensors', or 'quit')")
        
        while not rospy.is_shutdown():
            try:
                command = input("> ") 
                if command.lower() == 'quit':
                    break
                self.process_command(command)
            except EOFError:
                break

if __name__ == '__main__':
    try:
        rospy.init_node('diagnostics_and_commander', anonymous=True)
        # N.B.: Il costruttore in Python si chiama __init__, non _init_
        node = DiagnosticsAndCommanderNode()
        node.run()
    except rospy.ROSInterruptException:
        print("\nProgram interrupted. Shutting down.")
        pass