#!/usr/bin/env python

import rospy
import json
import os
import shutil
import threading

# Import the builder for our static world model
from greenhouse_kg_builder import create_greenhouse_graph

class KnowledgeGraphService:
    def __init__(self, profiles_template_path, live_profiles_db_path):
        rospy.loginfo("KGS: Initializing Knowledge Graph Service...")

        # --- Part 1: The STATIC World Model ---
        # This graph contains the permanent layout and entities of the greenhouse.
        # Live data will be populated into its nodes by the main controller.
        self.static_graph = create_greenhouse_graph()
        rospy.loginfo("KGS: Static greenhouse graph loaded successfully.")

        # --- Part 2: The DYNAMIC User Profiles ---
        # This dictionary holds the learnable, persistent user habits and feedback.
        self.profiles_template_path = profiles_template_path
        self.live_profiles_db_path = live_profiles_db_path
        self.user_profiles = self._load_or_initialize_profiles()
        self.profiles_lock = threading.Lock()

        # --- Extract Entities for LLM Grounding ---
        # This is equivalent to _extract_graph_entities in the mall example.
        self._extract_entities_for_llm()
        rospy.loginfo("KGS: Extracted entities for LLM grounding.")

    def _load_or_initialize_profiles(self):
        if not os.path.exists(self.live_profiles_db_path):
            rospy.logwarn(f"'{self.live_profiles_db_path}' not found. Creating new profiles from template.")
            shutil.copy(self.profiles_template_path, self.live_profiles_db_path)
        with open(self.live_profiles_db_path, 'r') as f:
            profiles = json.load(f)
        rospy.loginfo("KGS: User profiles loaded successfully.")
        return profiles

    def _save_profiles(self):
        with self.profiles_lock:
            with open(self.live_profiles_db_path, 'w') as f:
                json.dump(self.user_profiles, f, indent=2)
            rospy.loginfo(f"KGS: User profile changes saved to '{self.live_profiles_db_path}'.")

    def _extract_entities_for_llm(self):
        """Extracts all known entity names to provide context to the LLM."""
        self.roles_from_db = list(self.user_profiles.keys())
        
        self.tasks_from_graph = [] # In a more complex system, tasks could be nodes
        self.tasks_from_graph.append({"id": "check_resources", "name": "Check Resources"})
        self.tasks_from_graph.append({"id": "get_greenhouse_status", "name": "Get Greenhouse Status"})
        self.tasks_from_graph.append({"id": "run_diagnostics", "name": "Run Diagnostics"})

        self.equipment_from_graph = []
        for node_id, data in self.static_graph.nodes(data=True):
            self.equipment_from_graph.append({"id": node_id, "name": data['name']})

    # --- Methods to provide entities to other modules (like LLMManager) ---
    def get_all_roles(self):
        return self.roles_from_db
    
    def get_all_tasks(self):
        return self.tasks_from_graph
    
    def get_all_equipment(self):
        return self.equipment_from_graph

    # --- Methods for interacting with the DYNAMIC User Profiles ---
    def get_user_profile(self, user_role):
        with self.profiles_lock:
            return self.user_profiles.get(user_role)

    def update_user_preferences(self, user_role, updates):
        """Updates a user's profile with new learned information."""
        with self.profiles_lock:
            profile = self.user_profiles.get(user_role)
            if not profile:
                rospy.logwarn(f"KGS: Attempted to update profile for non-existent role: {user_role}")
                return
            
            # This is a simple merge, a real system might have more complex logic
            # For example, appending to lists instead of replacing them.
            if 'preferred_tasks' in updates:
                for task in updates['preferred_tasks']:
                    if task not in profile['preferred_tasks']:
                        profile['preferred_tasks'].append(task)
            
            # Deeper merge for task-specific preferences
            if 'task_specific_preferences' in updates:
                for task_id, prefs in updates['task_specific_preferences'].items():
                    if task_id in profile['task_specific_preferences']:
                        for key, value in prefs.items():
                             # Example: appending to lists within the preferences
                            if isinstance(profile['task_specific_preferences'][task_id].get(key), list):
                                for item in value:
                                    if item not in profile['task_specific_preferences'][task_id][key]:
                                        profile['task_specific_preferences'][task_id][key].append(item)
                            else: # Overwrite other values
                                profile['task_specific_preferences'][task_id][key] = value

        self._save_profiles()

    # --- Methods for querying the STATIC Graph (populated with live data) ---
    def get_equipment_details(self, equipment_id):
        """Returns all data for a specific piece of equipment."""
        if self.static_graph.has_node(equipment_id):
            return self.static_graph.nodes[equipment_id]
        return None

    def execute_structured_query(self, structured_query):
        """
        Equivalent to the function of the same name. It takes a parsed command from the LLM
        and retrieves the corresponding data from the knowledge graph.
        """
        intent = structured_query.get("intent")
        rospy.loginfo(f"KGS: Executing structured query with intent: {intent}")
        
        results = {}

        if intent == "get_specific_status":
            target_id = structured_query.get("target_equipment_id")
            if target_id:
                details = self.get_equipment_details(target_id)
                if details:
                    results['equipment_status'] = details
                else:
                    results['error'] = f"Equipment ID '{target_id}' not found."
        
        elif intent == "perform_task":
            task_id = structured_query.get("target_task_id")
            # This is where you'd retrieve all data relevant to a task.
            # For "check_resources", this means all resource-related nodes.
            if task_id == "check_resources":
                resource_nodes = []
                for node_id, data in self.static_graph.nodes(data=True):
                    if data.get('label_node') in ["ResourceTank", "ResourceStorage", "Robot"]:
                        resource_nodes.append(self.static_graph.nodes[node_id])
                results['task_data'] = {"resources": resource_nodes}
            else:
                results['info'] = f"Data retrieval for task '{task_id}' is not implemented."

        return results

if __name__ == "__main__":
    print("--- Testing KnowledgeGraphService for Greenhouse ---")
    
    # Create dummy template/live files for standalone testing
    script_dir = os.path.dirname(os.path.abspath(__file__))
    template_path = os.path.join(script_dir, 'user_profiles_template.json')
    live_db_path = os.path.join(script_dir, 'live_user_profiles.json')
    if not os.path.exists(template_path):
        print("Error: user_profiles_template.json not found. Cannot run test.")
        exit()
    if os.path.exists(live_db_path):
        os.remove(live_db_path)

    # Initialize the service
    kgs = KnowledgeGraphService(template_path, live_db_path)
    
    print("\n--- Testing Entity Extraction ---")
    print(f"Roles: {kgs.get_all_roles()}")
    print(f"Tasks: {kgs.get_all_tasks()}")
    # print(f"Equipment: {kgs.get_all_equipment()}") # Can be verbose

    print("\n--- Testing Structured Query (get_specific_status) ---")
    query1 = {"intent": "get_specific_status", "target_equipment_id": "water_tank_1"}
    results1 = kgs.execute_structured_query(query1)
    print(f"Query Result: {json.dumps(results1, indent=2)}")

    print("\n--- Testing Structured Query (perform_task) ---")
    query2 = {"intent": "perform_task", "target_task_id": "check_resources"}
    results2 = kgs.execute_structured_query(query2)
    print(f"Query Result: {json.dumps(results2, indent=2)}")

    print("\n--- Testing Preference Update ---")
    update_data = {
        "preferred_tasks": ["check_resources"],
        "task_specific_preferences": {
            "check_resources": {
                "sensors_of_interest": ["water_tank_1", "solar_storage"]
            }
        }
    }
    kgs.update_user_preferences("Commander", update_data)
    print("Commander profile updated. Check 'live_user_profiles.json' to see the saved changes.")
    print(f"In-memory profile now: {json.dumps(kgs.get_user_profile('Commander'), indent=2)}")