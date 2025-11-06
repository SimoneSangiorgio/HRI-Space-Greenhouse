#!/usr/bin/env python

import rospy
import time
import json
import random
import threading
from collections import deque
from datetime import datetime
from std_msgs.msg import String
from kg_manager import KnowledgeGraphManager
from llm_manager import LLMManager, ALL_TASKS_LIST, TASKS_ROBOT_GREENHOUSE
from speech_manager import SpeechManager
from whisper_manager import WhisperManager


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
        
        self.console_lock = threading.Lock()

        self.speaker = SpeechManager()

        self.listener = WhisperManager(model_size="base") 


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
            "remember_fact": self.handle_remember_fact
        }
        
        rospy.loginfo("Controller waiting for sensor data...")
        while not (self.robot_sensors_data and self.serra_state_data) and not rospy.is_shutdown():
            time.sleep(0.5)
        rospy.loginfo("Sensor data received. Controller is ready.")

    def robot_sensors_callback(self, msg):
        with self.data_lock: self.robot_sensors_data = json.loads(msg.data)
    def serra_state_callback(self, msg):
        with self.data_lock: self.serra_state_data = json.loads(msg.data)
    
    def say_and_print(self, message, is_robot_response=False):
        with self.console_lock:
            print(message)
            # self.speaker.say(message)
        if is_robot_response:
            self.conversation_history.append({"role": "assistant", "content": message})

    def get_confirmation(self, prompt):
        with self.console_lock:
            print(prompt)
            if not rospy.is_shutdown():
                response = input("> ").lower().strip()
            else:
                response = "no"
        self.conversation_history.append({"role": "assistant", "content": prompt})
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
        self.current_user_profile['name_from_id'] = name_input 
        
        interaction_count = self.current_user_profile.get("relationship", {}).get("interaction_count", 0)
        welcome_message = ""
        address_name = f"{self.current_user_role} {name_input}"

        if interaction_count == 0:
            if self.current_user_role == "Commander": welcome_message = f"{address_name}, authorization confirmed. SERRA at your service."
            elif self.current_user_role == "Scientist": welcome_message = f"Dr. {name_input}, a pleasure to collaborate. System data is available."
            else: welcome_message = f"Hello {name_input}! I'm SERRA. Welcome to the greenhouse!"
        else:
            last_session_summary = self.current_user_profile.get("memory", {}).get("last_session_summary")
            welcome_message = f"Welcome back, {address_name}. "
            if last_session_summary:
                welcome_message += f"In our last session, we {last_session_summary}. "
            
            if self.current_user_role == "Commander": welcome_message += "SERRA ready for tasking."
            elif self.current_user_role == "Scientist": welcome_message += "Ready to resume our analysis?"
            else: welcome_message += "It's great to see you again!"
        self.say_and_print(welcome_message, is_robot_response=True)

        preferred_task, count = self.kg.get_preferred_task(self.current_user_id)
        is_routine_query = preferred_task == "general_query" and self.current_user_role in ["Commander", "Scientist"]
        is_preferred_action = preferred_task and preferred_task not in ["acknowledgement", "general_query"]

        if count > 3 and (is_routine_query or is_preferred_action):
            prompt = ""
            if is_routine_query:
                prompt = "Standard procedure is a general status report. Shall I proceed?"
            else:
                prompt = f"I notice you often perform '{preferred_task}'. Would you like me to start with that?"
            
            if self.get_confirmation(prompt):
                self.execute_command({"intent": preferred_task, "original_request": f"Start my usual task: {preferred_task}"})

    def check_for_proactive_alerts(self):
        self.say_and_print("\nPerforming system status check...", is_robot_response=True)
        time.sleep(1)
        
        alerts = []
        with self.data_lock:
            battery_level = self.robot_sensors_data.get("battery_percent")
        
        world_knowledge = self.kg.get_world_knowledge()
        critical_thresholds = world_knowledge.get("critical_thresholds", {})
        battery_threshold = critical_thresholds.get("battery_percent", 20)

        if battery_level and battery_level < battery_threshold:
            alert_msg = f"CRITICAL ALERT: Battery level is at {battery_level}%, which is below the {battery_threshold}% threshold."
            alerts.append(alert_msg)
            if self.get_confirmation(f"{alert_msg} Shall I navigate to the charging station?"):
                self.execute_command({"intent": "navigate_to_charging_station", "original_request": "Charge battery due to critical alert."})

        if not alerts:
            self.say_and_print("All systems nominal. Awaiting command.", is_robot_response=True)

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.current_task_thread and self.current_task_thread.is_alive():
                time.sleep(0.5)
                continue
            
            # --- MODIFICATION STARTS HERE ---
            
            # The prompt now happens OUTSIDE the console lock.
            self.say_and_print(f"Awaiting your command, {self.current_user_profile['name_from_id']}.", is_robot_response=True)
            try:
                # This input() call now safely blocks without holding any locks.
                input(">> Press Enter to start speaking, you will have 5 seconds...")
            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("Exit signal received during prompt.")
                user_command_text = "exit" # Treat Ctrl+C here as an exit command

            # Now we acquire the lock just for the business logic part
            with self.console_lock:
                try:
                    # The listener function now just records and transcribes, no more input().
                    user_command_text = self.listener.listen_for_command()
                    print(f"\n{self.current_user_id} (heard) > {user_command_text}")
                except (EOFError, KeyboardInterrupt):
                    # This is a fallback in case the interrupt happens during recording
                    user_command_text = "exit"
            # --- MODIFICATION ENDS HERE ---
            
            self.conversation_history.append({"role": "user", "content": user_command_text})
            
            user_command_lower = user_command_text.lower()
            exit_keywords = ["exit", "quit", "end session"]
            
            if user_command_lower == "done" or any(keyword in user_command_lower for keyword in exit_keywords):
                self.handle_stop_task({})
                
                # 1. Genera il riassunto
                self.say_and_print("Summarizing session...", is_robot_response=False)
                summary = self.llm.get_session_summary(self.conversation_history)
                
                # 2. Salva il riassunto nel KG (in memoria)
                if summary:
                    self.kg.update_session_summary(self.current_user_id, summary)

                # 3. Salva l'intero KG su file
                self.say_and_print("Saving session data...", is_robot_response=True)
                self.kg.save_kg()
                
                self.say_and_print("Session terminated. Goodbye.", is_robot_response=True)
                rospy.signal_shutdown("User ended the session.")
                break

            if not user_command_text.strip(): continue

            self.kg.update_interaction_style(self.current_user_id, user_command_text)
            self.current_user_profile = self.kg.get_user_profile(self.current_user_id)
            self.current_user_profile['name_from_id'] = self.current_user_id.split('_')[1]

            parsed_command = None
            remember_keywords = ["remember", "note", "recall", "keep in mind", "always", "ever", "forever"]
            play_keywords = ["play", "game", "toy"]

            if any(keyword in user_command_lower for keyword in play_keywords):
                rospy.loginfo("'Play' keyword detected. Forcing 'query_rules_and_permissions' intent.")
                parsed_command = {"intent": "query_rules_and_permissions", "original_request": user_command_text}
            elif any(keyword in user_command_lower for keyword in remember_keywords):
                rospy.loginfo("'Remember' keyword detected. Forcing 'remember_fact' intent.")
                parsed_command = self.llm.parse_intent(user_command_text, context=self.conversation_context, user_profile=self.current_user_profile, history=list(self.conversation_history))
                if 'fact' in parsed_command:
                    parsed_command['intent'] = 'remember_fact'
                else:
                    rospy.logwarn("LLM failed to extract a fact. Performing manual extraction.")
                    fact_content = user_command_text
                    for keyword in remember_keywords:
                        fact_content = fact_content.lower().replace(keyword, "").replace("that", "").strip()
                    parsed_command = {'intent': 'remember_fact', 'fact': fact_content.capitalize(), 'original_request': user_command_text}

            if not parsed_command:
                parsed_command = self.llm.parse_intent(user_command_text, context=self.conversation_context, user_profile=self.current_user_profile, history=list(self.conversation_history))
            
            rospy.loginfo(f"Final parsed command: {parsed_command}")
            self.execute_command(parsed_command)

    def execute_command(self, parsed_command):
        intent = parsed_command.get("intent")
        if not intent or intent in ["unknown", "error"]:
            reason = parsed_command.get('reason', 'Command not understood or error occurred.')
            self.say_and_print(f"Error: {reason}", is_robot_response=True)
            self.conversation_context = {'last_intent': intent or 'error', 'reason': reason}
            self.kg.log_interaction(self.current_user_id, intent or 'error', success=False)
            return

        handler_function = self.task_handlers.get(intent, self.handle_placeholder)
        
        if intent not in ["acknowledgement", "stop_current_task"]:
            self.kg.log_interaction(self.current_user_id, intent, success=True) 

        self.conversation_context = {'last_intent': intent, 'last_intent_status': 'running'}
        is_long_running_task = intent in ["clean_floor_and_pathways", "navigate_to_charging_station", "execute_precision_watering", "map_and_localize"]

        def task_runner(command):
            try:
                handler_function(command)
            except Exception as e:
                rospy.logerr(f"Task {intent} failed with exception: {e}")
                self.say_and_print(f"An error occurred during task: {intent}.", is_robot_response=True)
                self.conversation_context['last_intent_status'] = 'failed'
                self.kg.log_interaction(self.current_user_id, intent, success=False)
        
        if is_long_running_task:
            if self.current_task_thread and self.current_task_thread.is_alive():
                self.say_and_print("Cannot start a new task while another is running.", is_robot_response=True)
                self.conversation_context['last_intent_status'] = 'failed'
                self.kg.log_interaction(self.current_user_id, intent, success=False)
                return
            self.task_stop_event.clear()
            self.current_task_thread = threading.Thread(target=task_runner, args=(parsed_command,))
            self.current_task_thread.start()
        else:
            task_runner(parsed_command)

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
            self.conversation_context['last_intent_status'] = 'cancelled'
        else:
            self.say_and_print("No task is currently running.", is_robot_response=True)

    def handle_placeholder(self, command):
        task_name = command.get('intent')
        self.say_and_print(f"Executing: {task_name}... This will take a few seconds.", is_robot_response=True)
        for i in range(5):
            if self.task_stop_event.is_set():
                self.say_and_print(f"Task '{task_name}' cancelled by user.", is_robot_response=True)
                self.conversation_context['last_intent_status'] = 'cancelled'
                return
            self.say_and_print(f"...{task_name} in progress ({i+1}/5)...")
            time.sleep(1)
        self.say_and_print(f"Task '{task_name}' complete.", is_robot_response=True)
        self.conversation_context['last_intent_status'] = 'success'

    def handle_acknowledgement(self, command):
        style = self.current_user_profile.get("interaction_style", {})
        rel_level = self.current_user_profile.get("relationship", {}).get("relationship_level", 0)
        
        if style.get("formality") == "informal" and rel_level > 0.5:
            responses = ["You got it!", "No problem.", "Done."]
        elif self.current_user_role == "Commander":
            responses = ["Acknowledged, Commander.", "Affirmative.", "Understood."]
        elif self.current_user_role == "Scientist":
            responses = ["Of course, Doctor.", "Noted.", "Glad to assist."]
        else:
            responses = ["You're welcome!", "Happy to help!", "Okay!"]
        self.say_and_print(f"SERRA: {random.choice(responses)}", is_robot_response=True)

    def handle_general_query(self, command):
        self.say_and_print("Accessing data...", is_robot_response=True)
        time.sleep(1)
        with self.data_lock: context = { "robot_status": self.robot_sensors_data, "greenhouse_status": self.serra_state_data }
        
        response_text = self.llm.get_conversational_response(
            command.get("original_request"), context, 
            user_profile=self.current_user_profile, history=list(self.conversation_history)
        )
        self.say_and_print(f"SERRA: {response_text}", is_robot_response=True)
        self.conversation_context['last_intent_status'] = 'success'
    
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
                self.conversation_context['last_intent_status'] = 'cancelled'
        else:
            self.say_and_print(f"Analysis complete. No direct action recommended.", is_robot_response=True)
            if reason and potential_task == "none": self.say_and_print(f"Details: {reason}", is_robot_response=True)
            self.conversation_context['last_intent_status'] = 'success'
        
    def handle_list_capabilities(self, command):
        self.say_and_print("My operational parameters are divided into these categories:", is_robot_response=True)
        for category, tasks in TASKS_ROBOT_GREENHOUSE.items():
            self.say_and_print(f"\n--- {category.replace('_', ' ').upper()} ---")
            for task in tasks: self.say_and_print(f"- {task}")
        self.conversation_context['last_intent_status'] = 'success'

    def handle_remember_fact(self, command):
        fact_to_remember = command.get("fact")
        
        if not fact_to_remember:
            self.say_and_print("I understood you want me to remember something, but I didn't catch what. Please try again.", is_robot_response=True)
            self.conversation_context['last_intent_status'] = 'failed'
            return

        added = self.kg.add_key_fact(self.current_user_id, fact_to_remember)

        if added:
            confirmation_prompt = f"Confirm to the user that you have successfully remembered the following fact: '{fact_to_remember}'"
        else:
            confirmation_prompt = f"Inform the user that you already have the following fact on record: '{fact_to_remember}'"

        self.generate_and_speak_response(confirmation_prompt, {})
        self.conversation_context['last_intent_status'] = 'success'

    def handle_query_rules_and_permissions(self, command):
        rules = self.kg.get_safety_rules()
        original_request = command.get("original_request")
        
        context_prompt = (
            f"The user, whose role is '{self.current_user_role}', made this request: '{original_request}'.\n"
            f"Here are the general safety rules of this facility: {json.dumps(rules)}\n"
            "Your task is to respond to the user based on their role:\n"
            "1. Find the most relevant safety rule that applies to their request.\n"
            "2. Explain this rule kindly and simply. DO NOT just quote the rule verbatim.\n"
            "3. If the user is a 'Child', be extra gentle and suggest a simple, safe, and engaging alternative activity that I can do. Good examples are: 'we can count how many red plants there are', 'I can show you my lights', or 'we can check if the plants have enough water together'.\n"
            "4. If the user is an adult, be professional and explain the rule clearly."
        )

        self.generate_and_speak_response(context_prompt, {"relevant_rules": rules})
        self.conversation_context['last_intent_status'] = 'success'

    def handle_check_battery(self, command): 
        self.generate_and_speak_response("Report battery status.", {"battery_percent": self.robot_sensors_data.get('battery_percent', 'N/A')})
        self.conversation_context['last_intent_status'] = 'success'

    def handle_read_environment(self, command): 
        self.generate_and_speak_response(command.get("original_request"), {"sensor_data": self.robot_sensors_data})
        self.conversation_context['last_intent_status'] = 'success'

    def handle_image_analysis(self, command):
        self.generate_and_speak_response(command.get("original_request"), {"plants_info": self.serra_state_data.get('plants_info', [])})
        time.sleep(0.5)
        if self.get_confirmation("Shall I proceed with a detailed image analysis now?"):
            self.handle_placeholder({"intent": "image_analysis_for_health"})
        else:
            self.say_and_print("Understood. Awaiting next command.", is_robot_response=True)
            self.conversation_context['last_intent_status'] = 'cancelled'

    def handle_monitor_tanks(self, command): 
        self.generate_and_speak_response("Report water tank status.", {'tanks': self.serra_state_data})
        self.conversation_context['last_intent_status'] = 'success'

    def handle_query_user_profile(self, command):
        profile = self.current_user_profile
        preferred_task, _ = self.kg.get_preferred_task(self.current_user_id)
        profile_summary = {
            "User ID": self.current_user_id,
            "Role": profile.get("profile", {}).get("role"),
            "Interaction Count": profile.get("relationship", {}).get("interaction_count", 0),
            "Learned Style": profile.get("interaction_style", {}).get("verbosity_preference", "normal"),
            "Relationship Level": f"{profile.get('relationship', {}).get('relationship_level', 0):.2f}",
            "Frequent Task": preferred_task or "None"
        }
        self.generate_and_speak_response(command.get("original_request"), {"user_profile_summary": profile_summary})
        self.conversation_context['last_intent_status'] = 'success'

if __name__ == '__main__':
    try:
        MainRobotController().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Shutting down controller.")
        pass