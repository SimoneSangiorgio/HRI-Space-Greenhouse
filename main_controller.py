#!/usr/bin/env python

import rospy
import time
import json
import random
import threading
from collections import deque
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
        self.conversation_history = deque(maxlen=6)

        self.current_task_thread = None
        self.task_stop_event = threading.Event()

        self.task_handlers = {
            "stop_current_task": self.handle_stop_task,
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
    
    def say_and_print(self, message, is_robot_response=False):
        print(message)
        if is_robot_response:
            self.conversation_history.append({"role": "assistant", "content": message})

    def get_confirmation(self, prompt):
        self.say_and_print(prompt, is_robot_response=True)
        response = input("> ").lower().strip()
        self.conversation_history.append({"role": "user", "content": response})
        return response in ['yes', 'y', 'ok', 'sure', 'fine', 'confirm', 'go ahead']

    def run(self):
        self.say_and_print("\n--- SERRA HRI System Initialized ---")
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
        welcome_message = ""
        if interaction_count == 0:
            if self.current_user_role == "Commander": welcome_message = f"Commander {name_input}, authorization confirmed. SERRA at your service."
            elif self.current_user_role == "Scientist": welcome_message = f"Dr. {name_input}, a pleasure to collaborate. System data is available."
            else: welcome_message = f"Hello {name_input}! I'm SERRA. Welcome to the greenhouse!"
        else:
            if self.current_user_role == "Commander": welcome_message = f"Commander {name_input}, welcome back. SERRA ready for tasking."
            elif self.current_user_role == "Scientist": welcome_message = f"Welcome back, Dr. {name_input}. Ready to resume our analysis?"
            else: welcome_message = f"Hi {name_input}, it's great to see you again!"
        self.say_and_print(welcome_message, is_robot_response=True)

        preferred_task, count = self.kg.get_preferred_task(self.current_user_id)
        if count > 2 and preferred_task == "general_query":
            prompt = ""
            if self.current_user_role == "Commander": prompt = "Standard procedure is a general status report. Shall I proceed?"
            elif self.current_user_role == "Scientist": prompt = "Would you like to begin with a full data overview, as is customary?"
            if prompt and self.get_confirmation(prompt):
                self.execute_command({"intent": "general_query", "original_request": "Provide a full status report."})

    def check_for_proactive_alerts(self):
        self.say_and_print("\nPerforming system status check...", is_robot_response=True)
        time.sleep(1)
        self.say_and_print("All systems nominal. Awaiting command.", is_robot_response=True)

    def main_loop(self):
        while not rospy.is_shutdown():
            user_command_text = input(f"{self.current_user_id} > ")
            self.conversation_history.append({"role": "user", "content": user_command_text})
            
            if user_command_text.lower() in ["exit", "quit", "done", "end session"]:
                self.handle_stop_task({})
                self.say_and_print("Session terminated. Goodbye.", is_robot_response=True)
                break
            if not user_command_text.strip(): continue

            self.kg.update_interaction_style(self.current_user_id, user_command_text)
            self.current_user_profile = self.kg.get_user_profile(self.current_user_id)
            
            parsed_command = None
            user_command_lower = user_command_text.lower()

            # --- GERARCHIA DI CONTROLLO ---
            # 1. COMANDI CRITICI E DI NAVIGAZIONE
            if any(phrase in user_command_lower for phrase in ["stop", "halt", "cancel"]):
                parsed_command = {"intent": "stop_current_task", "original_request": user_command_text}
            elif any(phrase in user_command_lower for phrase in ["remember me", "who am i", "my profile"]):
                parsed_command = {"intent": "query_user_profile", "original_request": user_command_text}
            # --- NUOVA REGOLA PER I PERMESSI ---
            elif any(phrase in user_command_lower for phrase in ["can i", "am i allowed", "is it okay", "play with"]):
                parsed_command = {"intent": "query_rules_and_permissions", "original_request": user_command_text}

            # 2. MATCHING DIRETTO DEI TASK
            if not parsed_command:
                user_keywords = set(user_command_lower.replace("ok", "").replace("please", "").strip().split())
                for task in ALL_TASKS_LIST:
                    task_keywords = set(task.split('_'))
                    if user_keywords.intersection(task_keywords):
                        parsed_command = {"intent": task, "original_request": user_command_text}
                        rospy.loginfo(f"Direct task match found: {task}")
                        break
            
            # 3. LLM COME FALLBACK
            if not parsed_command:
                parsed_command = self.llm.parse_intent(
                    user_command_text, context=self.conversation_context, 
                    user_profile=self.current_user_profile, history=list(self.conversation_history)
                )
            
            rospy.loginfo(f"Final parsed command: {parsed_command}")
            self.execute_command(parsed_command)

    def execute_command(self, parsed_command):
        intent = parsed_command.get("intent")
        handler_function = self.task_handlers.get(intent, self.handle_placeholder)
        if intent and intent not in ["unknown", "error"]:
            if intent not in ["acknowledgement", "stop_current_task"]: 
                self.kg.log_interaction(self.current_user_id, intent)
            
            is_long_running_task = intent in ["clean_floor_and_pathways", "navigate_to_charging_station", "execute_precision_watering"]
            if is_long_running_task:
                if self.current_task_thread and self.current_task_thread.is_alive():
                    self.say_and_print("Cannot start a new task while another is running.", is_robot_response=True)
                    return
                self.task_stop_event.clear()
                self.current_task_thread = threading.Thread(target=handler_function, args=(parsed_command,))
                self.current_task_thread.start()
            else:
                handler_function(parsed_command)

            self.conversation_context = {'last_intent': intent}
        elif intent == "unknown": self.say_and_print("Command not understood. Please rephrase.", is_robot_response=True)
        else: self.say_and_print(f"Error: {parsed_command.get('reason', 'Unknown')}", is_robot_response=True)

    def generate_and_speak_response(self, question, context_data):
        response = self.llm.get_conversational_response(
            question, context_data, 
            user_profile=self.current_user_profile, history=list(self.conversation_history)
        )
        self.say_and_print(f"SERRA: {response}", is_robot_response=True)

    def handle_stop_task(self, command):
        if self.current_task_thread and self.current_task_thread.is_alive():
            self.say_and_print("Stop command received. Terminating current task...", is_robot_response=True)
            self.task_stop_event.set()
            self.current_task_thread.join(timeout=2.0)
        else:
            self.say_and_print("No task is currently running.", is_robot_response=True)

    def handle_placeholder(self, command):
        task_name = command.get('intent')
        self.say_and_print(f"Executing: {task_name}... This will take a few seconds.", is_robot_response=True)
        for i in range(5):
            if self.task_stop_event.is_set():
                self.say_and_print(f"Task '{task_name}' cancelled by user.", is_robot_response=True)
                return
            self.say_and_print(f"...{task_name} in progress ({i+1}/5)...")
            time.sleep(1)
        self.say_and_print(f"Task '{task_name}' complete.", is_robot_response=True)

    def handle_acknowledgement(self, command):
        responses = {
            "Commander": ["Acknowledged, Commander.", "Affirmative.", "Understood."],
            "Scientist": ["Of course, Doctor.", "Noted.", "Glad to assist."],
            "default": ["You're welcome!", "Happy to help!", "Okay!"]
        }
        role_responses = responses.get(self.current_user_role, responses["default"])
        self.say_and_print(f"SERRA: {random.choice(role_responses)}", is_robot_response=True)

    def handle_general_query(self, command):
        self.say_and_print("Accessing data...", is_robot_response=True)
        time.sleep(1)
        with self.data_lock: context = { "robot_status": self.robot_sensors_data, "greenhouse_status": self.serra_state_data }
        
        response_text = self.llm.get_conversational_response(
            command.get("original_request"), context, 
            user_profile=self.current_user_profile, history=list(self.conversation_history)
        )
        self.say_and_print(f"SERRA: {response_text}", is_robot_response=True)

        recommended_task = next((task for task in ALL_TASKS_LIST if task in response_text), None)
        if recommended_task:
            time.sleep(0.5)
            prompt = f"My analysis suggests the action '{recommended_task}'. Shall I proceed?"
            if self.get_confirmation(prompt):
                self.execute_command({"intent": recommended_task, "original_request": f"Confirmation for: {recommended_task}"})
            else:
                self.say_and_print("Acknowledged. Awaiting further command.", is_robot_response=True)
    
    def handle_recommendation(self, command):
        self.say_and_print("Analyzing data to recommend optimal action...", is_robot_response=True)
        with self.data_lock: context = {**self.robot_sensors_data, **self.serra_state_data}
        recommendation_response = self.llm.get_recommendation(
            json.dumps(context), user_goal=command.get("original_request"), 
            user_profile=self.current_user_profile, history=list(self.conversation_history)
        )
        
        potential_task, reason = None, "No reason provided."
        lines = recommendation_response.strip().split('\n')
        if lines and lines[0].startswith("TASK:"):
            potential_task = lines[0].replace("TASK:", "").strip()
            if len(lines) > 1 and lines[1].startswith("REASON:"): reason = lines[1].replace("REASON:", "").strip()
        
        if potential_task and potential_task != "none" and potential_task in ALL_TASKS_LIST:
            self.say_and_print(f"Recommendation: {potential_task}.", is_robot_response=True)
            self.say_and_print(f"Justification: {reason}", is_robot_response=True)
            if self.get_confirmation("Confirm execution?"):
                self.execute_command({"intent": potential_task, "original_request": recommendation_response})
            else:
                self.say_and_print("Recommendation denied.", is_robot_response=True)
        else:
            self.say_and_print(f"Analysis complete. No direct action recommended.", is_robot_response=True)
            if reason and potential_task == "none": self.say_and_print(f"Details: {reason}", is_robot_response=True)
        
    def handle_list_capabilities(self, command):
        self.say_and_print("My operational parameters are divided into these categories:", is_robot_response=True)
        for category, tasks in TASKS_ROBOT_GREENHOUSE.items():
            self.say_and_print(f"\n--- {category.replace('_', ' ').upper()} ---")
            for task in tasks: self.say_and_print(f"- {task}")
    
    def handle_check_battery(self, command): self.generate_and_speak_response("Report battery status.", {"battery_percent": self.robot_sensors_data.get('battery_percent', 'N/A')})
    def handle_read_environment(self, command): self.generate_and_speak_response(command.get("original_request"), {"sensor_data": self.robot_sensors_data})
    def handle_image_analysis(self, command): self.generate_and_speak_response(command.get("original_request"), {"plants_info": self.serra_state_data.get('plants_info', [])})
    def handle_monitor_tanks(self, command): self.generate_and_speak_response("Report water tank status.", {'tanks': self.serra_state_data})
    def handle_query_user_profile(self, command):
        preferred_task, _ = self.kg.get_preferred_task(self.current_user_id)
        profile_summary = {"User ID": self.current_user_id, "Role": self.current_user_role, "Interaction Count": self.current_user_profile.get("interaction_count", 0), "Learned Style": self.current_user_profile.get("interaction_style", {}).get("verbosity_preference", "normal"), "Frequent Task": preferred_task or "None"}
        self.generate_and_speak_response(command.get("original_request"), {"user_profile_summary": profile_summary})
    def handle_query_rules_and_permissions(self, command):
        rules = self.kg.get_safety_rules()
        self.generate_and_speak_response(command.get("original_request"), {"relevant_rules": rules})

if __name__ == '__main__':
    try:
        MainRobotController().run()
    except rospy.ROSInterruptException:
        pass