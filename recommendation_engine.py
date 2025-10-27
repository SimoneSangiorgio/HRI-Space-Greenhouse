#!/usr/bin/env python
# recommender_engine.py

import rospy
import json

class RecommenderEngine:
    def __init__(self, kgs):
        self.kgs = kgs
        rospy.loginfo("Recommender Engine initialized.")

    def generate_report_from_ranked_list(self, ranked_list):
        if not ranked_list:
            return "No relevant information to report."
        
        report_parts = ["Based on priority and your preferences, here is the status:"]
        for item in ranked_list:
            details = item['equipment_details']
            name = details.get('name', 'Unknown Equipment')
            status_parts = []
            for key, value in details.items():
                if value is not None and key not in ['label_node', 'name', 'bay_id', 'species', 'id']:
                    status_parts.append(f"{key.replace('_', ' ')}: {value}")
            
            if status_parts:
                 report_parts.append(f" - {name}: {', '.join(status_parts)} (Priority Score: {item['score']})")

        return "\n".join(report_parts)
    
    def format_single_item_report(self, equipment_data):
        if not equipment_data:
            return "No data found for the requested item."
        
        name = equipment_data.get('name', 'Unknown Equipment')
        parts = [f"Status for {name}:"]
        
        details_found = False
        for key, value in equipment_data.items():
            if value is not None and key not in ['label_node', 'name', 'bay_id', 'species', 'id']:
                parts.append(f" - {key.replace('_', ' ').capitalize()}: {value}")
                details_found = True
        
        if not details_found:
            return f"No live data is currently available for {name}."

        return "\n".join(parts)

    ### --- CHANGE: Replaced the ugly JSON dump with this clean function --- ###
    def generate_preferences_summary(self, user_role):
        """Generates a clean, human-readable summary of a user's learned preferences."""
        profile = self.kgs.get_user_profile(user_role)
        if not profile:
            return f"No profile found for {user_role}."

        summary_parts = [f"Here is a summary of your learned preferences, {user_role}:"]
        
        preferred_tasks = profile.get('preferred_tasks')
        if preferred_tasks:
            summary_parts.append(f" - Your most common task is: '{preferred_tasks[-1]}'")
        else:
            summary_parts.append(" - I have not learned your preferred tasks yet.")

        task_prefs = profile.get('task_specific_preferences', {})
        has_sensor_prefs = False
        for task, prefs in task_prefs.items():
            sensors = prefs.get('sensors_of_interest')
            if sensors:
                has_sensor_prefs = True
                # Get the actual names of the sensors from the KGS
                all_equipment = self.kgs.get_all_equipment()
                sensor_names = [next((e['name'] for e in all_equipment if e['id'] == s_id), s_id) for s_id in sensors]
                summary_parts.append(f" - For the '{task}' task, you are interested in: {', '.join(sensor_names)}")
        
        if not has_sensor_prefs:
            summary_parts.append(" - I have not learned your specific sensor preferences for any task yet.")

        return "\n".join(summary_parts)