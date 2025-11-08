#!/usr/bin/env python

import rospy
import time
import json
import random
import threading
from collections import deque
from datetime import datetime
from std_msgs.msg import String

# MERGED: Imports for physical actions remain from the main script
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import math
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

# MERGED: Core manager imports remain
from kg_manager import KnowledgeGraphManager
from llm_manager import LLMManager, ALL_TASKS_LIST, TASKS_ROBOT_GREENHOUSE

# MERGED: Imports for speech and listening are added from the voice-enabled script
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

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action server found.")

        self.play_motion_client = actionlib.SimpleActionClient('play_motion', PlayMotionAction)
        rospy.loginfo("Waiting for play_motion action server...")
        self.play_motion_client.wait_for_server()
        rospy.loginfo("play_motion action server found.")

        self.current_user_id, self.current_user_role, self.current_user_profile, self.conversation_context = None, None, {}, {}
        self.conversation_history = deque(maxlen=6)

        self.current_task_thread = None
        self.task_stop_event = threading.Event()
        
        self.console_lock = threading.Lock()

        # MERGED: Initialize Speech and Listening Managers
        self.speaker = SpeechManager()
        self.listener = WhisperManager(model_size="base")

        self.CONVERSATIONAL_INTENTS = ["greeting", "farewell", "small_talk", "contextual_question", "general_query"]

        self.task_handlers = {
            "execute_task_sequence": self.handle_execute_task_sequence,
            "stop_current_task": self.handle_stop_task,
            "self_diagnostic_check": self.handle_simulated_action,
            "check_battery_level": self.handle_check_battery,
            "navigate_to_charging_station": self.handle_navigate_to_charging_station,
            "navigate_to_location": self.handle_navigate_to_location,
            "map_and_localize": self.handle_simulated_action,
            "plan_optimal_path": self.handle_simulated_action,
            "read_temperature_humidity_sensors": self.handle_read_environment,
            "read_CO2_level": self.handle_read_environment,
            "read_light_intensity": self.handle_read_environment,
            "read_soil_moisture_at_plant": self.handle_simulated_action,
            "report_environmental_anomalies": self.handle_simulated_action,
            "image_analysis_for_health": self.handle_image_analysis,
            "detect_pests_and_diseases": self.handle_image_analysis,
            "execute_precision_watering": self.handle_simulated_action,
            "apply_targeted_nutrients": self.handle_simulated_action,
            "execute_selective_pruning": self.handle_simulated_action,
            "identify_and_harvest_ripe_fruit": self.handle_simulated_action,
            "clean_floor_and_pathways": self.handle_simulated_action,
            "sanitize_robotic_arm_tools": self.handle_simulated_action,
            "monitor_water_and_nutrient_tanks": self.handle_monitor_tanks,
            "transport_harvest_to_storage": self.handle_simulated_action,
            "general_query": self.handle_general_query,
            "list_capabilities": self.handle_list_capabilities,
            "ask_for_recommendation": self.handle_recommendation,
            "query_user_profile": self.handle_query_user_profile,
            "query_rules_and_permissions": self.handle_query_rules_and_permissions,
            "acknowledgement": self.handle_acknowledgement,
            "remember_fact": self.handle_remember_fact,
            "simulated_action_pickup_water": self.handle_simulated_action,
            "simulated_action_dispense_water": self.handle_simulated_action,
            "simulated_action_image_analysis": self.handle_simulated_action
        }
        
        rospy.loginfo("Controller waiting for sensor data...")
        while not (self.robot_sensors_data and self.serra_state_data) and not rospy.is_shutdown():
            time.sleep(0.5)
        rospy.loginfo("Sensor data received. Controller is ready.")

    def robot_sensors_callback(self, msg):
        with self.data_lock: self.robot_sensors_data = json.loads(msg.data)
    def serra_state_callback(self, msg):
        with self.data_lock: self.serra_state_data = json.loads(msg.data)
    
    # MERGED: This function now enables text-to-speech output
    def say_and_print(self, message, is_robot_response=False):
        with self.console_lock:
            prefix = "SERRA: " if is_robot_response else ""
            print(f"{prefix}{message}")
            # This line gives the robot its voice
            self.speaker.say(message)
        if is_robot_response:
            self.conversation_history.append({"role": "assistant", "content": message})

    # MERGED: Replaced text-based confirmation with a voice-based one
    def get_confirmation(self, prompt):
        self.say_and_print(prompt, is_robot_response=True)
        
        try:
            input(">> Press Enter to START speaking your answer (yes/no)...")
            self.listener.start_listening()
            input("   ...Recording... Press Enter to STOP.")
            response = self.listener.stop_listening_and_transcribe().lower().strip()
            print(f"\n{self.current_user_id} (heard) > {response}")
        except (EOFError, KeyboardInterrupt):
            rospy.loginfo("Exit signal received during confirmation.")
            response = "no"

        self.conversation_history.append({"role": "user", "content": response})
        return response in ['yes', 'y', 'ok', 'sure', 'fine', 'confirm', 'go ahead']

    def run(self):
        self.say_and_print("\n--- SERRA HRI System Initialized ---")
        self.identify_user()
        self.check_for_proactive_alerts()
        self.main_loop()

    def identify_user(self):
        # NOTE: User identification remains text-based for initial setup simplicity.
        self.say_and_print("Unit online. Please state your role and name for authorization.")
        role_input = input("Role > ").strip().title()
        name_input = input("Name > ").strip().title()
        self.current_user_id = f"{role_input}_{name_input}"
        self.current_user_role = role_input
        self.current_user_profile = self.kg.get_user_profile(self.current_user_id)
        self.current_user_profile['name_from_id'] = name_input 
        
        interaction_count = self.current_user_profile.get("relationship", {}).get("interaction_count", 0)
        address_name = f"{self.current_user_role} {name_input}"

        if interaction_count == 0:
            if self.current_user_role == "Commander": welcome_message = f"{address_name}, authorization confirmed. SERRA at your service."
            elif self.current_user_role == "Scientist": welcome_message = f"Dr. {name_input}, a pleasure to collaborate. System data is available."
            else: welcome_message = f"Hello {name_input}! I'm SERRA. Welcome to the greenhouse!"
            
            self.say_and_print(welcome_message, is_robot_response=True)
            
            roles_to_exclude_greeting = ["Commander", "Scientist"]
            if self.current_user_role not in roles_to_exclude_greeting:
                self._perform_physical_greeting()
            else:
                rospy.loginfo(f"Skipping physical greeting for formal role: {self.current_user_role}")

        else:
            last_session_summary = self.current_user_profile.get("memory", {}).get("last_session_summary")
            welcome_message = f"Welcome back, {address_name}. "
            
            allowed_roles_for_summary = ["Commander", "Scientist"]
            if self.current_user_role in allowed_roles_for_summary and last_session_summary:
                welcome_message += f"In our last session, {last_session_summary}. "
            
            if self.current_user_role == "Commander": welcome_message += "SERRA ready for tasking."
            elif self.current_user_role == "Scientist": welcome_message += "Ready to resume our analysis?"
            else: welcome_message += "It's great to see you again!"
            self.say_and_print(welcome_message, is_robot_response=True)

        is_child = (self.current_user_role == "Child")
        if is_child and interaction_count > 0:
            self._perform_physical_greeting()

        preferred_task, count = self.kg.get_preferred_task(self.current_user_id)
        is_routine_query = preferred_task == "general_query" and self.current_user_role in ["Commander", "Scientist"]
        is_preferred_action = preferred_task and preferred_task not in ["acknowledgement", "general_query"]

        if count > 5 and (is_routine_query or is_preferred_action):
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
            
            self.say_and_print(f"Awaiting your command, {self.current_user_profile['name_from_id']}.", is_robot_response=True)
            
            # MERGED: Replaced text input with the event-driven voice listening logic
            try:
                input(">> Press Enter to START speaking...")
                self.listener.start_listening()
                
                input("   ...Recording... Press Enter to STOP.")
                user_command_text = self.listener.stop_listening_and_transcribe()

                print(f"\n{self.current_user_id} (heard) > {user_command_text}")

            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("Exit signal received during prompt.")
                user_command_text = "exit"

            self.conversation_history.append({"role": "user", "content": user_command_text})
            
            user_command_lower = user_command_text.lower().strip()
            exit_keywords = ["exit", "quit", "end session"]
            
            if user_command_lower == "done" or any(keyword in user_command_lower for keyword in exit_keywords):
                self.handle_stop_task({})
                self.say_and_print("Summarizing session...", is_robot_response=False)
                summary = self.llm.get_session_summary(self.conversation_history)
                if summary: self.kg.update_session_summary(self.current_user_id, summary)
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
            
            status_keywords = ["status", "report", "system status", "general status"]
            remember_keywords = ["remember", "note", "recall", "keep in mind", "always", "ever", "forever"]
            suggest_keywords = ["suggest", "recommend", "what should i do", "what can you do for me"]
            
            found_remember_keyword = next((keyword for keyword in remember_keywords if keyword in user_command_lower), None)

            is_status_query = user_command_lower in status_keywords
            contains_suggest_keyword = any(keyword in user_command_lower for keyword in suggest_keywords)

            if is_status_query:
                rospy.loginfo("Status keyword detected. Forcing 'general_query' intent.")
                parsed_command = {"intent": "general_query", "original_request": user_command_text}
            
            elif contains_suggest_keyword:
                 rospy.loginfo("Suggestion keyword detected. Forcing 'ask_for_recommendation' intent.")
                 parsed_command = {"intent": "ask_for_recommendation", "original_request": user_command_text}

            elif found_remember_keyword:
                rospy.loginfo(f"'{found_remember_keyword}' keyword detected. Forcing 'remember_fact' intent.")
                fact_content = user_command_lower.split(found_remember_keyword, 1)[1]
                fact_content = fact_content.strip().lstrip("that").lstrip("to").strip()
                fact = fact_content.capitalize()
                
                parsed_command = { "intent": "remember_fact", "fact": fact, "original_request": user_command_text }
            
            else:
                parsed_command = self.llm.parse_intent(user_command_text, context=self.conversation_context, user_profile=self.current_user_profile, history=list(self.conversation_history))
            
            rospy.loginfo(f"Final parsed command: {parsed_command}")
            self.execute_command(parsed_command)

    def _plan_task_from_model(self, command, task_model):
        target = command.get("target")
        if not target:
            self.say_and_print(f"Error: The task '{command.get('intent')}' requires a target, but none was provided.", is_robot_response=True)
            return None
        
        resource_location = task_model.get("resource_location")
        
        planned_tasks = []
        for step in task_model.get("steps", []):
            new_step = step.copy()
            if new_step.get("location") == "<target_location>":
                new_step["location"] = target
            if new_step.get("location") == "<resource_location>":
                if not resource_location:
                    self.say_and_print(f"Error: Planning failed. Task model for '{command.get('intent')}' needs a resource, but none is defined.", is_robot_response=True)
                    return None
                new_step["location"] = resource_location
            
            planned_tasks.append(new_step)
        
        return planned_tasks

    def execute_command(self, parsed_command):
        intent = parsed_command.get("intent")
        if not intent or intent in ["unknown", "error"]:
            reason = parsed_command.get('reason', 'Command not understood or error occurred.')
            self.say_and_print(f"Error: {reason}", is_robot_response=True)
            self.conversation_context = {'last_intent': intent or 'error', 'reason': reason}
            self.kg.log_interaction(self.current_user_id, intent or 'error', success=False)
            return

        if intent in self.CONVERSATIONAL_INTENTS:
            rospy.loginfo(f"Handling '{intent}' as a conversational turn.")
            self.kg.log_interaction(self.current_user_id, intent, success=True)
            self.conversation_context = {'last_intent': intent, 'last_intent_status': 'success'}
            self.generate_and_speak_response(parsed_command.get("original_request"), {})
            return
        
        if intent == 'execute_task_sequence':
            expanded_tasks = []
            for task in parsed_command.get('tasks', []):
                task_intent = task.get('intent')
                task_model = self.kg.get_task_model(task_intent)
                if task_model:
                    rospy.loginfo(f"Expanding sub-task '{task_intent}' within a sequence.")
                    sub_sequence = self._plan_task_from_model(task, task_model)
                    if sub_sequence: expanded_tasks.extend(sub_sequence)
                    else:
                        self.say_and_print(f"Failed to plan for sub-task '{task_intent}'. Aborting sequence.", is_robot_response=True)
                        return
                else:
                    expanded_tasks.append(task)
            parsed_command['tasks'] = expanded_tasks
        else:
            task_model = self.kg.get_task_model(intent)
            if task_model:
                self.say_and_print(f"Understood. Planning how to perform '{intent}'...", is_robot_response=True)
                planned_sequence = self._plan_task_from_model(parsed_command, task_model)
                
                if planned_sequence:
                    parsed_command = {
                        "intent": "execute_task_sequence",
                        "tasks": planned_sequence,
                        "original_request": parsed_command.get("original_request")
                    }
                    intent = "execute_task_sequence"
                else:
                    self.say_and_print(f"Sorry, I could not create a valid plan for your request.", is_robot_response=True)
                    self.kg.log_interaction(self.current_user_id, intent, success=False)
                    return
        
        handler_function = self.task_handlers.get(intent, self.handle_simulated_action)
        
        if intent not in ["acknowledgement", "stop_current_task"] + self.CONVERSATIONAL_INTENTS:
            self.kg.log_interaction(self.current_user_id, intent, success=True)

        self.conversation_context = {'last_intent': intent, 'last_intent_status': 'running'}
        
        is_long_running_task = intent in [
            "clean_floor_and_pathways", "navigate_to_charging_station", 
            "execute_precision_watering", "map_and_localize", "navigate_to_location",
            "execute_task_sequence"
        ]

        def task_runner(command):
            try:
                success = handler_function(command)
                if intent == "execute_task_sequence" and not success:
                    rospy.logwarn(f"Task sequence '{command.get('original_request')}' failed or was cancelled.")
                    self.conversation_context['last_intent_status'] = 'failed'
                    self.kg.log_interaction(self.current_user_id, intent, success=False)
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
        self.say_and_print(response, is_robot_response=True)

    def _send_navigation_goal(self, x, y, yaw_degrees):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, math.radians(yaw_degrees))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.say_and_print(f"Navigating to (x={x}, y={y})...", is_robot_response=True)
        self.move_base_client.send_goal(goal)
        while not self.move_base_client.wait_for_result(rospy.Duration(1.0)):
            if self.task_stop_event.is_set():
                self.move_base_client.cancel_goal()
                self.say_and_print("Navigation cancelled by user.", is_robot_response=True)
                return False
        state = self.move_base_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.say_and_print("Destination reached successfully.", is_robot_response=True)
            return True
        else:
            self.say_and_print(f"Failed to reach the destination. Final state: {state}", is_robot_response=True)
            return False

    def _perform_physical_greeting(self):
        self.say_and_print("[Performs a welcoming wave motion]", is_robot_response=False)
        rospy.loginfo("Executing 'wave' motion...")
        wave_goal = PlayMotionGoal(motion_name='wave', skip_planning=True)
        self.play_motion_client.send_goal(wave_goal)
        wave_success = self.play_motion_client.wait_for_result(rospy.Duration(10.0))
        if not wave_success or self.play_motion_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("Greeting motion 'wave' failed or timed out.")
        else:
            rospy.loginfo("Greeting motion 'wave' completed successfully.")
        rospy.loginfo("Returning arm to 'home' position to ensure a safe state.")
        home_goal = PlayMotionGoal(motion_name='home', skip_planning=True)
        self.play_motion_client.send_goal(home_goal)
        home_success = self.play_motion_client.wait_for_result(rospy.Duration(10.0))
        if not home_success or self.play_motion_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logerr("CRITICAL: Failed to return arm to 'home' position.")
        else:
            rospy.loginfo("Arm returned to 'home' position successfully.")

    def handle_execute_task_sequence(self, command):
        task_list = command.get("tasks")
        if not task_list:
            self.say_and_print("Error: The command required a sequence, but I couldn't find any tasks to execute.", is_robot_response=True)
            return False
        self.say_and_print(f"Executing sequence of {len(task_list)} tasks.", is_robot_response=True)
        for i, task_command in enumerate(task_list):
            if self.task_stop_event.is_set():
                self.say_and_print("Sequence aborted by user.", is_robot_response=True)
                return False
            intent = task_command.get("intent")
            if not intent:
                rospy.logwarn(f"Skipping invalid task in sequence: {task_command}")
                continue
            handler = self.task_handlers.get(intent)
            if not handler:
                rospy.logwarn(f"No handler found for intent '{intent}' in sequence. Skipping.")
                continue
            self.say_and_print(f"--- Sequence Step {i+1}/{len(task_list)}: Executing '{intent}' ---", is_robot_response=True)
            task_success = handler(task_command)
            if not task_success:
                self.say_and_print(f"A step in the sequence failed. Aborting the sequence.", is_robot_response=True)
                return False
            time.sleep(1)
        self.say_and_print("Sequence completed successfully.", is_robot_response=True)
        self.conversation_context['last_intent_status'] = 'success'
        return True

    def handle_stop_task(self, command):
        if self.current_task_thread and self.current_task_thread.is_alive():
            self.say_and_print("Stop command received. Terminating current task...", is_robot_response=True)
            self.task_stop_event.set()
            self.current_task_thread.join(timeout=3.0) 
            self.conversation_context['last_intent_status'] = 'cancelled'
        else:
            self.say_and_print("No task is currently running.", is_robot_response=True)
        return True # Stop is always a "successful" action
    
    def handle_simulated_action(self, command):
        task_name = command.get('intent')
        self.say_and_print(f"Executing: {task_name}... This will take 2 seconds.", is_robot_response=True)
        for _ in range(2):
            if self.task_stop_event.is_set():
                self.say_and_print(f"Task '{task_name}' cancelled by user.", is_robot_response=True)
                return False
            time.sleep(1)
        self.say_and_print(f"Task '{task_name}' complete.", is_robot_response=True)
        self.conversation_context['last_intent_status'] = 'success'
        return True

    def handle_acknowledgement(self, command):
        style = self.current_user_profile.get("interaction_style", {})
        rel_level = self.current_user_profile.get("relationship", {}).get("relationship_level", 0)
        
        if style.get("formality") == "informal" and rel_level > 0.5: responses = ["You got it!", "No problem.", "Done."]
        elif self.current_user_role == "Commander": responses = ["Acknowledged, Commander.", "Affirmative.", "Understood."]
        elif self.current_user_role == "Scientist": responses = ["Of course, Doctor.", "Noted.", "Glad to assist."]
        else: responses = ["You're welcome!", "Happy to help!", "Okay!"]
        self.say_and_print(random.choice(responses), is_robot_response=True)
        return True

    def handle_navigate_to_location(self, command):
        location_name = command.get("location")
        if not location_name:
            self.say_and_print("I understood you want me to go somewhere, but I need a specific destination.", is_robot_response=True)
            return False
        coords = self.kg.get_location_coords(location_name)
        if not coords:
            self.say_and_print(f"I'm sorry, I don't know the location '{location_name}'.", is_robot_response=True)
            return False
        success = self._send_navigation_goal(coords['x'], coords['y'], coords['yaw'])
        self.conversation_context['last_intent_status'] = 'success' if success else 'failed'
        return success

    def handle_navigate_to_charging_station(self, command):
        self.say_and_print("Navigating to the charging station.", is_robot_response=True)
        command['location'] = 'charging station'
        return self.handle_navigate_to_location(command)

    def handle_check_battery(self, command): 
        self.generate_and_speak_response("Report battery status.", {"battery_percent": self.robot_sensors_data.get('battery_percent', 'N/A')})
        return True

    def handle_read_environment(self, command): 
        self.generate_and_speak_response(command.get("original_request"), {"sensor_data": self.robot_sensors_data})
        return True
    
    def handle_image_analysis(self, command):
        if command.get('target'):
            rospy.logwarn("handle_image_analysis called directly, but should have been planned.")
            return self.handle_simulated_action({'intent': 'simulated_action_image_analysis'})
        else:
            self.say_and_print("Where should I perform the image analysis?", is_robot_response=True)
            return False

    def handle_monitor_tanks(self, command): 
        self.generate_and_speak_response("Report water tank status.", {'tanks': self.serra_state_data})
        return True

    def handle_general_query(self, command):
        self.say_and_print("Accessing data...", is_robot_response=True)
        time.sleep(1)
        with self.data_lock: context = { "robot_status": self.robot_sensors_data, "greenhouse_status": self.serra_state_data }
        self.generate_and_speak_response(command.get("original_request"), context)
        return True

    def handle_recommendation(self, command):
        self.say_and_print("Analyzing data to recommend an optimal action...", is_robot_response=True)
        with self.data_lock:
            context = {**self.robot_sensors_data, **self.serra_state_data}
        recommendation_json_str = self.llm.get_recommendation(json.dumps(context), user_goal=command.get("original_request"), user_profile=self.current_user_profile, history=list(self.conversation_history))
        try:
            recommendation = json.loads(recommendation_json_str)
            task_to_execute = recommendation.get("task")
            reason = recommendation.get("reason", "No reason provided.")
            if task_to_execute and task_to_execute.get("intent"):
                self.say_and_print(f"Based on my analysis, I have a recommendation.", is_robot_response=True)
                self.say_and_print(f"Reason: {reason}", is_robot_response=True)
                intent_name = task_to_execute.get('intent')
                target_name = task_to_execute.get('target')
                prompt = f"The proposed task is '{intent_name}'"
                if target_name: prompt += f" on target '{target_name}'"
                prompt += ". Shall I proceed?"
                if self.get_confirmation(prompt):
                    self.execute_command(task_to_execute)
                else:
                    self.say_and_print("Recommendation denied.", is_robot_response=True)
            else:
                self.say_and_print(f"Analysis complete. {reason}", is_robot_response=True)
        except json.JSONDecodeError:
            rospy.logerr(f"Failed to parse recommendation JSON: {recommendation_json_str}")
            self.say_and_print("I had trouble formulating a recommendation. Please try again.", is_robot_response=True)
        return True

    def handle_list_capabilities(self, command):
        self.say_and_print("My operational parameters are divided into these categories:", is_robot_response=True)
        for category, tasks in TASKS_ROBOT_GREENHOUSE.items():
            self.say_and_print(f"\n--- {category.replace('_', ' ').upper()} ---")
            for task in tasks: self.say_and_print(f"- {task}")
        return True

    def handle_remember_fact(self, command):
        fact_to_remember = command.get("fact")
        if not fact_to_remember:
            self.say_and_print("I didn't catch what you wanted me to remember. Please try again.", is_robot_response=True)
            return False
        added = self.kg.add_key_fact(self.current_user_id, fact_to_remember)
        if added: confirmation_prompt = f"Confirm to the user that you have successfully remembered the following fact: '{fact_to_remember}'"
        else: confirmation_prompt = f"Inform the user that you already have the following fact on record: '{fact_to_remember}'"
        self.generate_and_speak_response(confirmation_prompt, {})
        return True

    def handle_query_rules_and_permissions(self, command):
        rules = self.kg.get_safety_rules()
        original_request = command.get("original_request")
        context_prompt = (
            f"The user, a '{self.current_user_role}', requested: '{original_request}'.\n"
            f"Here are the relevant safety rules: {json.dumps(rules)}\n"
            "Respond based on their role. For a 'Child', be gentle and suggest a safe alternative "
            "(e.g., 'we can count the plants'). For adults, be professional and explain the relevant rule simply."
        )
        self.generate_and_speak_response(context_prompt, {"relevant_rules": rules})
        return True

    def handle_query_user_profile(self, command):
        profile = self.current_user_profile
        preferred_task, _ = self.kg.get_preferred_task(self.current_user_id)
        profile_summary = {
            "User ID": self.current_user_id, "Role": profile.get("profile", {}).get("role"),
            "Interaction Count": profile.get("relationship", {}).get("interaction_count", 0),
            "Relationship Level": f"{profile.get('relationship', {}).get('relationship_level', 0):.2f}",
            "Frequent Task": preferred_task or "None"
        }
        self.generate_and_speak_response(command.get("original_request"), {"user_profile_summary": profile_summary})
        return True

# MERGED: Using the more robust try/finally shutdown sequence from the voice script
if __name__ == '__main__':
    controller = None
    try:
        controller = MainRobotController()
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Shutdown signal received.")
    finally:
        rospy.loginfo("Initiating final shutdown sequence...")
        if controller:
            if controller.listener:
                # Ensure the whisper manager cleans up its resources
                controller.listener.shutdown()
            if controller.kg:
                # Ensure the knowledge graph is always saved
                rospy.loginfo("Saving final knowledge graph state...")
                controller.kg.save_kg()
        rospy.loginfo("Controller has been shut down.")