#!/usr/bin/env python

import rospy
import time
from datetime import datetime
from diagnostics_node import DiagnosticsNode
from kg_manager import KnowledgeGraphManager
from speech_manager import SpeechManager # ## ADD THIS IMPORT ##

# ... (TASKS definition remains the same) ...
TASKS = [
    "set the stellar route",
    "check the available resource",
    "to take care of the greenhouse",
    "to clean the floar"
]


class MainRobotController:
    def __init__(self):
        rospy.init_node('main_robot_controller', anonymous=True)
        self.diagnostics = DiagnosticsNode()
        self.kg = KnowledgeGraphManager()
        self.speaker = SpeechManager() 
        self.current_user_role = None
        
        rospy.loginfo("Main controller is waiting for initial sensor data...")
        while not self.diagnostics.get_robot_power() and not rospy.is_shutdown():
            time.sleep(0.5)
        rospy.loginfo("Initial sensor data received. Controller is ready.")

    # ## HELPER FUNCTION ##
    def say_and_print(self, message):
        """A helper function for the 'double way' output."""
        print(message)
        self.speaker.say(message)

    # handle yes/no questions
    def get_confirmation(self, prompt):
        """Asks the user a yes/no question and returns True for yes, False for no."""
        self.say_and_print(prompt)
        response = input("> ").lower().strip()
        affirmative_responses = ['yes', 'y', 'ok', 'sure', 'fine', 'confirm', 'go ahead']
        if response in affirmative_responses:
            return True
        return False
    
    # A function to let the user pick a different task
    def select_alternative_task(self):
        """Handles the case where the user rejects the suggested task."""
        self.say_and_print("Understood. What would you like to do instead?")
        dialogue = f"Your options are: ({') or ('.join(TASKS)})"
        self.say_and_print(dialogue)
        
        chosen_task = input("> ").lower().strip()
        matched_task = next((task for task in TASKS if task in chosen_task), None)

        if matched_task:
            # We still log this interaction to learn that preferences can change!
            self.kg.log_interaction(self.current_user_role, matched_task)
            self.handle_task(matched_task, optimized=True) # Still use optimized flow for the new task
        else:
            self.say_and_print("Sorry, I didn't understand that task. We can try again later.")

    def run(self):
    
        self.say_and_print("\n--- TIAGo HRI System Initialized ---")
        self.identify_user()

        if self.kg.is_experienced() and self.kg.get_preferred_task(self.current_user_role):
            self.run_experienced_flow()
        else:
            self.run_newbie_flow()

    def identify_user(self):
        self.say_and_print("HI, good morning, who i m talking with? Please say Commander, Greenhouse Employee or Crew Assistant.")
        role_input = input("> ")
        if "commander" in role_input.lower():
            self.current_user_role = "Commander"
        elif "greenhouse" in role_input.lower():
            self.current_user_role = "Greenhouse Employee"
        else:
            self.current_user_role = "Crew Assistant"

    def run_newbie_flow(self):
        """Implements the logic for Case 1 and 3."""
        dialogue = f"hi {self.current_user_role}, what we will do today? Do you prefer to ({') or ('.join(TASKS)})?"
        self.say_and_print(dialogue)
        
        chosen_task = input("> ").lower().strip()
        matched_task = next((task for task in TASKS if task in chosen_task), None)

        if matched_task:
            self.kg.log_interaction(self.current_user_role, matched_task)
            self.handle_task(matched_task, optimized=False)
        else:
            self.say_and_print("Sorry, I didn't understand that task.")

    def run_experienced_flow(self):
        """Implements the logic for Case 2 and 4 with confirmation."""
        preferred_task = self.kg.get_preferred_task(self.current_user_role)
        
        self.say_and_print(f"Hi {self.current_user_role}, welcome back.")
        
        # Ask for confirmation before proceeding with the suggested task
        prompt = f"Based on our history, your usual task is '{preferred_task}'. Shall we proceed with that? (yes or no)"
        
        if self.get_confirmation(prompt):
            self.handle_task(preferred_task, optimized=True)
        else:
            # If the user says no, let them choose a new task
            self.select_alternative_task()
    
    def handle_task(self, task, optimized):
        """Dispatches to the correct handler based on the task."""
        if "resource" in task:
            self.handle_check_resources(optimized)
        elif "greenhouse" in task:
            self.handle_greenhouse_care(optimized)
        else:
            self.say_and_print(f"Executing generic task: {task}")
            
    # ## MODIFIED ##: This function now includes the second confirmation step.
    def handle_check_resources(self, optimized):
        power_data = self.diagnostics.get_robot_power()
        ufficio_data = self.diagnostics.get_ufficio_status()
        now = datetime.now()
        
        report = (f"Of course commander, the resource level are the following: today is {now.strftime('%A %d of %B')}, "
                  f"now is the {now.strftime('%I:%M %p')} and we have {ufficio_data.get('water_tank_1_percent', 'N/A')}% of water in the first tank "
                  f"and {ufficio_data.get('water_tank_2_percent', 'N/A')}% of water in the second one, then we have "
                  f"{power_data.get('battery_percent', 'N/A')}% of energy stored in our battery.")
        self.say_and_print(report)

        if not optimized: # Case 1
            # TODO ADD THE COMMANDER INSTRUCTION AS PART OF THE KG
            #self.say_and_print("(suggestion to take care of the resource here.)(for now are unavailable because of the zero kg.)")
            self.say_and_print("Please provide instructions.")
            input("Commander instructions > ")
            self.say_and_print("Accepted. Executing task...")
        else: # Case 2
            # TODO ADD THE SUGGESTION WITH LLM
            suggestion_text = ""
            knowledge = self.kg.get_world_knowledge()
            crit_battery = knowledge.get('critical_thresholds', {}).get('battery_percent', 100)
            
            if power_data.get('battery_percent', 100) < crit_battery:
                suggestion = knowledge.get('optimal_actions', {}).get('low_battery', 'recharge')
                suggestion_text = (f"My suggestion, considering the past days, is to {suggestion}, "
                                   f"considering that its {now.strftime('%I:%M %p')} should give the best performance. "
                                   "Regarding the water, we can be safe because the greenhouse was watered yesterday and no more plants must be watered today.")
                self.say_and_print(suggestion_text)

            # Ask for confirmation on the suggestion
            if suggestion_text and self.get_confirmation("Do you approve this suggestion? (yes/no)"):
                # print("Commander answare with an happy face.")
                self.say_and_print("Excellent. Executing task...")
                # ... Add execution logic here ...
            else:
                self.say_and_print("Okay, plan rejected. Please provide manual instructions.")
                input("Commander instructions > ")
                self.say_and_print("Understood. Following your instructions.")
                
    # TODO ADD THE GARDENER

if __name__ == '__main__':
    try:
        controller = MainRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass