#!/usr/bin/env python
# main_controller.py

import rospy
import json
import os
import time
from std_msgs.msg import String

# --- Import Application Modules ---
from knowledge_graph_service import KnowledgeGraphService
from llm_manager import LLMManager
from recommendation_engine import RecommenderEngine

class MainController:
    def __init__(self):
        rospy.init_node('main_greenhouse_controller')
        rospy.loginfo("Main Controller initializing...")

        # --- Define File Paths ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        template_path = os.path.join(script_dir, 'user_profiles_template.json')
        live_db_path = os.path.join(script_dir, 'live_user_profiles.json')

        # --- Component Initialization ---
        self.kgs = KnowledgeGraphService(template_path, live_db_path)
        self.llm = LLMManager()
        self.recommender = RecommenderEngine(self.kgs)

        # --- Ground the LLM with known entities from the KGS ---
        self.llm.set_knowledge_base_entities(
            roles=self.kgs.get_all_roles(),
            tasks=self.kgs.get_all_tasks(),
            equipment=self.kgs.get_all_equipment()
        )

        # --- ROS Subscribers to keep the KGS live ---
        rospy.Subscriber('/robot/sensors', String, self.robot_sensors_callback)
        rospy.Subscriber('/serra/state', String, self.serra_state_callback)

        self.current_user = None
        rospy.loginfo("Controller initialized. Waiting for live data...")
        time.sleep(2)

    def robot_sensors_callback(self, msg):
        data = json.loads(msg.data)
        graph = self.kgs.static_graph
        graph.nodes['tiago_robot']['battery_percent'] = data.get('battery_percent')
        graph.nodes['env_sensors']['temperature_celsius'] = data.get('temperature_celsius')
        graph.nodes['env_sensors']['humidity_percent'] = data.get('humidity_percent')

    def serra_state_callback(self, msg):
        data = json.loads(msg.data)
        graph = self.kgs.static_graph
        graph.nodes['water_tank_1']['level_percent'] = data.get('water_tank_1_percent')
        graph.nodes['water_tank_2']['level_percent'] = data.get('water_tank_2_percent')
        graph.nodes['solar_storage']['energy_percent'] = data.get('stored_solar_energy_percent')
        for plant_info in data.get('plants_info', []):
            plant_id = f"plant_{plant_info['bay_id']}_{plant_info['plant_name'].lower()}"
            if graph.has_node(plant_id):
                graph.nodes[plant_id]['soil_moisture_percent'] = plant_info.get('soil_moisture_percent')

    def say(self, message):
        print(f"\nTIAGo: {message}")

    def listen(self):
        return input(f"\n{self.current_user}> ")

    def get_confirmation(self, prompt):
        self.say(prompt + " (yes/no)")
        response = self.listen().lower().strip()
        return response in ['yes', 'y', 'yep', 'sure']

    def run(self):
        self.identify_user()
        self.say(f"Hello {self.current_user}! How can I assist you today? (You can say 'stop' to end the session).")
        
        while not rospy.is_shutdown():
            user_input = self.listen()
            
            profile_context = self.kgs.get_user_profile(self.current_user)
            parsed_query = self.llm.parse_main_query(user_input, self.current_user, profile_context)
            rospy.loginfo(f"LLM Parsed Output: {json.dumps(parsed_query, indent=2)}")

            if not parsed_query or parsed_query.get("error"):
                self.say("Sorry, I had trouble understanding that. Could you please rephrase?")
                continue
            
            intent = parsed_query.get("intent")
            if intent == "stop_interaction":
                break
            
            ### --- CHANGE #1: Logic to handle "what should I do" and learn task preference --- ###
            # If the user asks a general question, we check their habits first.
            if intent == "perform_task" and not parsed_query.get("target_task_id"):
                session_status = self.propose_task_based_on_habits()
            else:
                # Otherwise, we process their specific request.
                if parsed_query.get("updated_preferences"):
                    self.kgs.update_user_preferences(self.current_user, parsed_query.get("updated_preferences"))
                    self.say("Thank you. I've updated your preferences.")
                session_status = self.process_intent(parsed_query)

            if session_status.get("status") == "stop_session":
                break
        
        self.say("Goodbye!")

    def identify_user(self):
        self.current_user = "User"
        valid_roles = self.kgs.get_all_roles()
        self.say(f"Welcome. Please state your role. (Options: {', '.join(valid_roles)})")
        while not rospy.is_shutdown():
            role_input = self.listen().strip().title().replace(" ", "")
            if role_input in valid_roles:
                self.current_user = role_input
                self.say(f"Acknowledged. Logged in as {self.current_user}.")
                return
            else:
                self.say("Role not recognized. Please choose from the available options.")
    
    ### --- CHANGE #2: New function to handle proactive task suggestions --- ###
    def propose_task_based_on_habits(self):
        """Checks user habits and proposes a task, or asks for a new one."""
        profile = self.kgs.get_user_profile(self.current_user)
        preferred_tasks = profile.get('preferred_tasks', [])

        if preferred_tasks:
            # Propose the most common/recent task
            primary_task_id = preferred_tasks[-1]
            task_name = self.kgs.get_all_tasks()[-1].get('name')
            if self.get_confirmation(f"Based on your habits, do you want to '{task_name}'?"):
                # If yes, launch the task handler
                return self._handle_iterative_task({"target_task_id": primary_task_id})
        
        # If no habits or user says no, ask what they want to do
        self.say("How can I assist you? (e.g., 'check resources', 'get greenhouse status')")
        user_input = self.listen()
        parsed_query = self.llm.parse_main_query(user_input, self.current_user)
        if parsed_query.get("intent") == "perform_task" and parsed_query.get("target_task_id"):
            # Learn this new task preference before executing
            self.kgs.update_user_preferences(self.current_user, {"preferred_tasks": [parsed_query.get("target_task_id")]})
            return self._handle_iterative_task(parsed_query)
        else:
            self.say("I'm sorry, I didn't understand that as a specific task.")
            return {"status": "completed"}


    def process_intent(self, parsed_query):
        intent = parsed_query.get("intent")
        
        if intent == "show_preferences":
            summary = self.recommender.generate_preferences_summary(self.current_user)
            self.say(summary)
            return {"status": "completed"}
        
        if intent == "update_preferences_only":
            self.say("Preference noted. How else can I help?")
            return {"status": "completed"}

        if intent == "get_specific_status":
            query_result = self.kgs.execute_structured_query(parsed_query)
            status_data = query_result.get('equipment_status')
            report = self.recommender.format_single_item_report(status_data)
            self.say(report)
            return {"status": "completed"}
            
        if intent == "perform_task":
             ### --- CHANGE #3: Learn the task preference if it's a direct command --- ###
            task_id = parsed_query.get("target_task_id")
            if task_id:
                self.kgs.update_user_preferences(self.current_user, {"preferred_tasks": [task_id]})
            return self._handle_iterative_task(parsed_query)

        self.say("I'm not sure how to handle that request. Please try rephrasing.")
        return {"status": "error"}

    def _handle_iterative_task(self, parsed_query):
        task_id = parsed_query.get("target_task_id")
        if not task_id:
            self.say("I understood you want to perform a task, but I'm not sure which one.")
            return {"status": "completed"}
        
        task_name = next((t['name'] for t in self.kgs.get_all_tasks() if t['id'] == task_id), task_id)
        self.say(f"Understood. Starting task: '{task_name}'.")
        
        profile = self.kgs.get_user_profile(self.current_user)
        task_prefs = profile.get('task_specific_preferences', {}).get(task_id, {})
        sensors_of_interest = task_prefs.get('sensors_of_interest', [])
        
        if sensors_of_interest:
            if self.get_confirmation(f"I see you're usually interested in certain sensors for this task. Shall I report on them now?"):
                for sensor_id in sensors_of_interest:
                    result = self.kgs.execute_structured_query({"intent": "get_specific_status", "target_equipment_id": sensor_id})
                    report = self.recommender.format_single_item_report(result.get('equipment_status'))
                    self.say(report)

        while not rospy.is_shutdown():
            self.say("\nWhat specific item would you like to check now? Or say 'done with this task'.")
            user_input = self.listen()
            
            if "done" in user_input:
                break
                
            parsed_item_query = self.llm.parse_main_query(user_input, self.current_user)
            item_id = parsed_item_query.get("target_equipment_id")
            
            if item_id:
                result = self.kgs.execute_structured_query({"intent": "get_specific_status", "target_equipment_id": item_id})
                report = self.recommender.format_single_item_report(result.get('equipment_status'))
                self.say(report)
                
                self.kgs.update_user_preferences(self.current_user, {
                    "task_specific_preferences": {task_id: {"sensors_of_interest": [item_id]}}
                })
            else:
                self.say("I didn't recognize that specific item. Please be more specific.")
        
        self.say(f"Task '{task_name}' complete.")
        return {"status": "completed"}

if __name__ == '__main__':
    try:
        controller = MainController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted. Shutting down.")
    except Exception as e:
        rospy.logerr(f"An unhandled exception occurred: {e}", exc_info=True)