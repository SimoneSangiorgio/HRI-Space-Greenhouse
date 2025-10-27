#!/usr/bin/env python

import rospy
import time
import json
import threading
from std_msgs.msg import String
from kg_manager import KnowledgeGraphManager
from llm_manager import LLMManager, ALL_TASKS_LIST, TASKS_ROBOT_GREENHOUSE

class MainRobotController:
    def __init__(self):
        rospy.init_node('main_robot_controller', anonymous=True)
        
        self.robot_sensors_data, self.serra_state_data = {}, {}
        self.data_lock = threading.Lock()
        rospy.Subscriber('/robot/sensors', String, self.robot_sensors_callback)
        rospy.Subscriber('/serra/state', String, self.serra_state_callback)

        self.kg = KnowledgeGraphManager()
        self.llm = LLMManager()

        self.current_user_id, self.current_user_role, self.current_user_profile, self.conversation_context = None, None, {}, {}

        self.task_handlers = {
            "self_diagnostic_check": self.handle_placeholder, "check_battery_level": self.handle_check_battery,
            "navigate_to_charging_station": self.handle_placeholder, "map_and_localize": self.handle_placeholder,
            "plan_optimal_path": self.handle_placeholder, "read_temperature_humidity_sensors": self.handle_read_environment,
            "read_CO2_level": self.handle_read_environment, "read_light_intensity": self.handle_read_environment,
            "read_soil_moisture_at_plant": self.handle_placeholder, "report_environmental_anomalies": self.handle_placeholder,
            "image_analysis_for_health": self.handle_image_analysis, "detect_pests_and_diseases": self.handle_image_analysis,
            "execute_precision_watering": self.handle_placeholder, "apply_targeted_nutrients": self.handle_placeholder,
            "execute_selective_pruning": self.handle_placeholder, "identify_and_harvest_ripe_fruit": self.handle_placeholder,
            "clean_floor_and_pathways": self.handle_placeholder, "sanitize_robotic_arm_tools": self.handle_placeholder,
            "monitor_water_and_nutrient_tanks": self.handle_monitor_tanks, "transport_harvest_to_storage": self.handle_placeholder,
            "general_query": self.handle_general_query, "list_capabilities": self.handle_list_capabilities,
            "ask_for_recommendation": self.handle_recommendation, "query_user_profile": self.handle_query_user_profile,
            "query_rules_and_permissions": self.handle_query_rules_and_permissions,
            "acknowledgement": self.handle_acknowledgement,
        }
        
        rospy.loginfo("Controller waiting for sensor data...")
        while not (self.robot_sensors_data and self.serra_state_data) and not rospy.is_shutdown(): time.sleep(0.5)
        rospy.loginfo("Sensor data received. Controller is ready.")

    def robot_sensors_callback(self, msg):
        with self.data_lock: self.robot_sensors_data = json.loads(msg.data)
    def serra_state_callback(self, msg):
        with self.data_lock: self.serra_state_data = json.loads(msg.data)
    def say_and_print(self, message):
        print(message)
    def get_confirmation(self, prompt):
        self.say_and_print(prompt)
        response = input("> ").lower().strip()
        return response in ['yes', 'y', 'ok', 'sure', 'fine', 'confirm', 'go ahead']

    def run(self):
        self.say_and_print("\n--- TIAGo HRI System Initialized ---")
        self.identify_user()
        self.check_for_proactive_alerts()
        self.main_loop()

    def identify_user(self):
        self.say_and_print("Unit online. Please state your role and name for authorization.")
        role_input = input("Role > ").strip().title()
        name_input = input("Name > ").strip().title()
        self.current_user_id = f"{role_input}_{name_input}"
        self.current_user_role = role_input
        
        self.current_user_profile = self.kg.get_user_profile(self.current_user_id)
        interaction_count = self.current_user_profile.get("interaction_count", 0)
        
        if interaction_count == 0:
            if self.current_user_role == "Commander":
                self.say_and_print(f"Commander {name_input}, authorization confirmed. SERRA at your service.")
            elif self.current_user_role == "Scientist":
                self.say_and_print(f"Dr. {name_input}, it is a pleasure to collaborate. System data is available for your review.")
            else:
                self.say_and_print(f"Hello {name_input}! I'm SERRA. Welcome to the greenhouse! It's nice to meet you.")
        else:
            if self.current_user_role == "Commander":
                self.say_and_print(f"Commander {name_input}, welcome back. SERRA ready for tasking.")
            elif self.current_user_role == "Scientist":
                 self.say_and_print(f"Welcome back, Dr. {name_input}. Ready to resume our analysis?")
            else:
                self.say_and_print(f"Hi {name_input}, it's great to see you again!")

    def check_for_proactive_alerts(self):
        self.say_and_print("\nPerforming system status check...")
        time.sleep(1)
        # (Logica alert invariata)
        self.say_and_print("All systems nominal. Awaiting command.")

    def main_loop(self):
        while not rospy.is_shutdown():
            user_command_text = input(f"{self.current_user_id} > ")
            if user_command_text.lower() in ["exit", "quit", "done", "end session"]:
                self.say_and_print("Session terminated. Goodbye.")
                break
            
            if not user_command_text.strip(): continue

            self.kg.update_interaction_style(self.current_user_id, user_command_text)
            self.current_user_profile = self.kg.get_user_profile(self.current_user_id)
            
            parsed_command = None
            user_command_lower = user_command_text.lower()
            if any(phrase in user_command_lower for phrase in ["remember me", "who am i", "my profile"]):
                parsed_command = {"intent": "query_user_profile", "original_request": user_command_text}
                rospy.loginfo(f"Hardcoded intent override: {parsed_command}")

            if not parsed_command:
                parsed_command = self.llm.parse_intent(
                    user_command_text, context=self.conversation_context, user_profile=self.current_user_profile
                )
            
            rospy.loginfo(f"LLM parsed command as: {parsed_command}")
            
            self.execute_command(parsed_command)

    def execute_command(self, parsed_command):
        intent = parsed_command.get("intent")
        handler_function = self.task_handlers.get(intent, self.handle_placeholder)
        
        if intent and intent not in ["unknown", "error"]:
            if intent != "acknowledgement":
                self.kg.log_interaction(self.current_user_id, intent)
            handler_function(parsed_command)
            self.conversation_context = {'last_intent': intent}
        elif intent == "unknown":
            self.say_and_print("Command not understood. Please rephrase or state capabilities.")
        else:
            self.say_and_print(f"Error: {parsed_command.get('reason', 'Unknown')}")

    def generate_and_speak_response(self, question, context_data):
        response = self.llm.get_conversational_response(
            question, context_data, user_profile=self.current_user_profile
        )
        self.say_and_print(f"SERRA: {response}")

    def handle_placeholder(self, command):
        self.say_and_print(f"Executing: {command.get('intent')}... Acknowledged.")
        time.sleep(1)
        self.say_and_print("Task complete.")

    def handle_check_battery(self, command):
        with self.data_lock: battery = self.robot_sensors_data.get('battery_percent', 'N/A')
        self.generate_and_speak_response("Report battery status.", {"battery_percent": battery})

    def handle_read_environment(self, command):
        with self.data_lock: data = self.robot_sensors_data.copy()
        self.generate_and_speak_response(command.get("original_request"), {"sensor_data": data})

    def handle_list_capabilities(self, command):
        self.say_and_print("My operational parameters are divided into these categories:")
        for category, tasks in TASKS_ROBOT_GREENHOUSE.items():
            self.say_and_print(f"\n--- {category.replace('_', ' ').upper()} ---")
            for task in tasks: self.say_and_print(f"- {task}")

    def handle_recommendation(self, command):
        self.say_and_print("Analyzing data to recommend optimal action...")
        with self.data_lock: context = {**self.robot_sensors_data, **self.serra_state_data}
        recommendation_response = self.llm.get_recommendation(
            json.dumps(context), user_goal=command.get("original_request"), user_profile=self.current_user_profile
        )
        
        potential_task, reason = None, "No specific reason provided."
        lines = recommendation_response.strip().split('\n')
        if lines and lines[0].startswith("TASK:"):
            potential_task = lines[0].replace("TASK:", "").strip()
            if len(lines) > 1 and lines[1].startswith("REASON:"):
                reason = lines[1].replace("REASON:", "").strip()
        
        if potential_task and potential_task != "none" and potential_task in ALL_TASKS_LIST:
            self.say_and_print(f"Recommendation: {potential_task}.")
            self.say_and_print(f"Justification: {reason}")
            
            if self.get_confirmation("Confirm execution?"):
                self.say_and_print(f"Confirmed. Executing '{potential_task}'.")
                self.execute_command({"intent": potential_task, "original_request": recommendation_response})
            else:
                self.say_and_print("Recommendation denied.")
        else:
            self.say_and_print(f"Analysis complete. No direct action recommended.")
            if reason and potential_task == "none": self.say_and_print(f"Details: {reason}")

    def handle_image_analysis(self, command):
        with self.data_lock: context = {"plants_info": self.serra_state_data.get('plants_info', [])}
        self.generate_and_speak_response(command.get("original_request"), context)

    def handle_monitor_tanks(self, command):
        with self.data_lock: context = {'tanks': self.serra_state_data}
        self.generate_and_speak_response("Report water tank status.", context)

    def handle_query_user_profile(self, command):
        preferred_task, _ = self.kg.get_preferred_task(self.current_user_id)
        profile_summary = {
            "User ID": self.current_user_id, "Role": self.current_user_role,
            "Interaction Count": self.current_user_profile.get("interaction_count", 0),
            "Learned Style": self.current_user_profile.get("interaction_style", {}).get("verbosity_preference", "normal"),
            "Frequent Task": preferred_task or "None"
        }
        self.generate_and_speak_response(command.get("original_request"), {"user_profile_summary": profile_summary})

    def handle_general_query(self, command):
        self.say_and_print("Accessing data...")
        time.sleep(1)
        with self.data_lock: context = { "robot_status": self.robot_sensors_data, "greenhouse_status": self.serra_state_data }
        
        # 1. Ottieni e stampa la risposta dell'LLM
        response_text = self.llm.get_conversational_response(
            command.get("original_request"), context, user_profile=self.current_user_profile
        )
        self.say_and_print(f"SERRA: {response_text}")

        # 2. Analizza la risposta del robot per trovare raccomandazioni proattive
        recommended_task = None
        for task in ALL_TASKS_LIST:
            if task in response_text:
                recommended_task = task
                break
        
        # 3. Se una raccomandazione è stata trovata, agisci!
        if recommended_task:
            time.sleep(0.5)
            prompt = f"My analysis suggests the action '{recommended_task}'. Shall I proceed?"
            if self.get_confirmation(prompt):
                self.say_and_print(f"Confirmed. Executing '{recommended_task}'.")
                # Crea un nuovo comando e eseguilo direttamente
                new_command = {"intent": recommended_task, "original_request": f"Confirmation for: {recommended_task}"}
                self.execute_command(new_command)
            else:
                self.say_and_print("Acknowledged. Awaiting further command.")

    def handle_query_rules_and_permissions(self, command):
        rules = self.kg.get_safety_rules()
        context = {"relevant_rules": rules, "user_question": command.get("original_request")}
        self.generate_and_speak_response(command.get("original_request"), context)

    def handle_acknowledgement(self, command):
        user_input = command.get("original_request")
        prompt_question = f"The user expressed thanks or approval with: '{user_input}'. Provide a brief, role-appropriate acknowledgement."
        self.generate_and_speak_response(prompt_question, {})

if __name__ == '__main__':
    try:
        MainRobotController().run()
    except rospy.ROSInterruptException:
        pass