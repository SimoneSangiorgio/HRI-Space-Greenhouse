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
        self.static_graph = create_greenhouse_graph()
        rospy.loginfo("KGS: Static greenhouse graph loaded successfully.")

        # --- Part 2: The DYNAMIC User Profiles ---
        self.profiles_template_path = profiles_template_path
        self.live_profiles_db_path = live_profiles_db_path
        self.user_profiles = self._load_or_initialize_profiles()
        self.profiles_lock = threading.Lock()

        # --- Extract Entities for LLM Grounding ---
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
        self.roles_from_db = list(self.user_profiles.keys())
        
        # --- THIS IS THE FIX: Create dictionaries, not lists of dicts ---
        self.tasks_from_graph = {
            "check_resources": "Check Resources",
            "get_greenhouse_status": "Get Greenhouse Status",
            "run_diagnostics": "Run Diagnostics"
        }
        
        self.equipment_from_graph = {node_id: data['name'] for node_id, data in self.static_graph.nodes(data=True)}

    # --- Methods to provide entities to other modules ---
    def get_all_roles(self):
        return self.roles_from_db
    
    def get_all_tasks(self):
        # Return in the format the LLM needs (list of dicts)
        return [{"id": k, "name": v} for k, v in self.tasks_from_graph.items()]
    
    def get_all_equipment(self):
        # Return in the format the LLM needs (list of dicts)
        return [{"id": k, "name": v} for k, v in self.equipment_from_graph.items()]

    def get_user_profile(self, user_role):
        with self.profiles_lock:
            return self.user_profiles.get(user_role)

    def update_user_preferences(self, user_role, updates):
        with self.profiles_lock:
            profile = self.user_profiles.get(user_role)
            if not profile: return
            
            if 'preferred_tasks' in updates:
                for task in updates['preferred_tasks']:
                    if task not in profile['preferred_tasks']:
                        profile['preferred_tasks'].append(task)
            
            if 'task_specific_preferences' in updates:
                for task_id, prefs in updates['task_specific_preferences'].items():
                    if task_id in profile['task_specific_preferences']:
                        for key, value in prefs.items():
                            if isinstance(profile['task_specific_preferences'][task_id].get(key), list):
                                for item in value:
                                    if item not in profile['task_specific_preferences'][task_id][key]:
                                        profile['task_specific_preferences'][task_id][key].append(item)
                            else:
                                profile['task_specific_preferences'][task_id][key] = value

        self._save_profiles()

    def get_equipment_details(self, equipment_id):
        if self.static_graph.has_node(equipment_id):
            return self.static_graph.nodes[equipment_id]
        return None

    def execute_structured_query(self, structured_query):
        intent = structured_query.get("intent")
        rospy.loginfo(f"KGS: Executing structured query with intent: {intent}")
        results = {}
        if intent == "get_specific_status":
            target_id = structured_query.get("target_equipment_id")
            if target_id:
                details = self.get_equipment_details(target_id)
                if details: results['equipment_status'] = details
                else: results['error'] = f"Equipment ID '{target_id}' not found."
        elif intent == "perform_task":
            task_id = structured_query.get("target_task_id")
            if task_id == "check_resources":
                resource_nodes = [self.static_graph.nodes[node_id] for node_id, data in self.static_graph.nodes(data=True) if data.get('label_node') in ["ResourceTank", "ResourceStorage", "Robot"]]
                results['task_data'] = {"resources": resource_nodes}
            else:
                results['info'] = f"Data retrieval for task '{task_id}' is not implemented."
        return results

if __name__ == "__main__":
    # Test code remains the same, will work with the new structure
    pass