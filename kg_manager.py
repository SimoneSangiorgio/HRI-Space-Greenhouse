#!/usr/bin/env python

import json
import os
import rospy
import shutil
from datetime import datetime

SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
KG_FILE_PATH = os.path.join(SCRIPT_DIRECTORY, 'knowledge_graph.json')

class KnowledgeGraphManager:
    def __init__(self, file_path=KG_FILE_PATH):
        self.file_path = file_path
        self.kg = {}
        self.load_kg()
        rospy.on_shutdown(self.save_kg)

    def load_kg(self):
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as f:
                    if os.path.getsize(self.file_path) > 0:
                        self.kg = json.load(f)
                        rospy.loginfo(f"Knowledge Graph loaded from: {self.file_path}")
                    else:
                        rospy.logwarn(f"KG file is empty. Starting a new one.")
                        self._create_default_kg()
            except json.JSONDecodeError:
                rospy.logerr(f"Failed to decode KG from {self.file_path}. File may be corrupt.")
                self._try_restore_backup()
        else:
            rospy.loginfo(f"No KG found at {self.file_path}. Starting a new one.")
            self._create_default_kg()

    def _try_restore_backup(self):
        backup_path = self.file_path + ".bak"
        if os.path.exists(backup_path):
            rospy.loginfo(f"Attempting to restore KG from backup: {backup_path}")
            try:
                shutil.copy(backup_path, self.file_path)
                self.load_kg()
            except Exception as e:
                rospy.logerr(f"Failed to restore from backup: {e}. Starting fresh.")
                self._create_default_kg()
        else:
            rospy.logwarn("No backup file found. Starting a fresh KG.")
            self._create_default_kg()
            
    def _create_default_kg(self):
        self.kg = {
            "user_profiles": {},
            "world_knowledge": {
                "critical_thresholds": { "battery_percent": 35 },
                "safety_rules": [
                    "Playing with toys is not permitted in the greenhouse to avoid accidental damage to sensitive plants and equipment.",
                    "Food and drinks are strictly forbidden to maintain a sterile environment.",
                    "Please always walk on the designated pathways and do not touch the plants unless authorized.",
                    "This is a scientific research area, so running and shouting are not allowed."
                ]
            }
        }

    def save_kg(self):
        tmp_path = self.file_path + ".tmp"
        bak_path = self.file_path + ".bak"
        try:
            if os.path.exists(self.file_path):
                shutil.copy(self.file_path, bak_path)
            with open(tmp_path, 'w') as f:
                json.dump(self.kg, f, indent=4)
            os.rename(tmp_path, self.file_path)
            rospy.loginfo(f"Knowledge Graph saved safely to: {self.file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save Knowledge Graph: {e}")
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def _get_or_create_user_profile(self, user_id):
        profiles = self.kg.setdefault("user_profiles", {})
        if user_id not in profiles:
            role = user_id.split('_')[0] if '_' in user_id else "Unknown"
            profiles[user_id] = {
                "profile": {
                    "role": role,
                    "preferred_address": "by_role"
                },
                "interaction_style": {
                    "verbosity_preference": "normal",
                    "formality": "formal" if role == "Commander" else "neutral",
                    "tone_preference": "neutral_factual" if role == "Commander" else "collaborative",
                    "proactivity_level": "reactive"
                },
                "relationship": {
                    "interaction_count": 0,
                    "relationship_level": 0.1,
                    "trust_score": 0.5
                },
                "memory": {
                    "key_facts": [],
                    "last_session_summary": None
                },
                "task_preferences": {},
                "timestamps": {
                    "first_seen": datetime.now().isoformat(),
                    "last_seen": datetime.now().isoformat()
                }
            }
        return profiles[user_id]

    def get_user_profile(self, user_id):
        profile = self._get_or_create_user_profile(user_id)
        profile["timestamps"]["last_seen"] = datetime.now().isoformat()
        return profile

    def log_interaction(self, user_id, intent, success=True):
        profile = self._get_or_create_user_profile(user_id)
        
        prefs = profile.setdefault("task_preferences", {})
        task_log = prefs.setdefault(intent, {"count": 0, "success_rate": 1.0})
        total_runs = task_log.get("count", 0)
        current_successes = total_runs * task_log.get("success_rate", 1.0)
        new_total_runs = total_runs + 1
        new_successes = current_successes + (1 if success else 0)
        task_log["count"] = new_total_runs
        task_log["success_rate"] = new_successes / new_total_runs if new_total_runs > 0 else 0
        task_log["last_used"] = datetime.now().isoformat()

        relationship = profile.setdefault("relationship", {})
        relationship["interaction_count"] = relationship.get("interaction_count", 0) + 1
        if success:
            relationship["relationship_level"] = min(1.0, relationship.get("relationship_level", 0.1) + 0.01)
        else:
            relationship["relationship_level"] = max(0.0, relationship.get("relationship_level", 0.1) - 0.05)

        profile["timestamps"]["last_seen"] = datetime.now().isoformat()
        rospy.loginfo(f"Logged interaction: {user_id} performed {intent} (Success: {success})")

    def update_interaction_style(self, user_id, command_text):
        profile = self._get_or_create_user_profile(user_id)
        style = profile.setdefault("interaction_style", {})
        
        command_length = len(command_text.split())
        if command_length <= 3:
            style["verbosity_preference"] = 'brief'
        elif command_length >= 8:
            style["verbosity_preference"] = 'verbose'
        else:
            style["verbosity_preference"] = 'normal'

        if any(word in command_text.lower() for word in ["please", "could you", "thank you"]):
            style["formality"] = "formal"
        elif command_length <= 2:
            style["formality"] = "informal"

    def add_key_fact(self, user_id, fact_text):
        profile = self._get_or_create_user_profile(user_id)
        memory = profile.setdefault("memory", {})
        key_facts = memory.setdefault("key_facts", [])

        normalized_fact = fact_text.strip().lower()
        
        for existing_fact in key_facts:
            if existing_fact.strip().lower() == normalized_fact:
                rospy.loginfo(f"Fact '{fact_text}' already exists for user {user_id}. Not adding.")
                return False

        key_facts.append(fact_text.strip())
        
        max_facts = 10
        if len(key_facts) > max_facts:
            key_facts.pop(0)

        rospy.loginfo(f"Added key fact for {user_id}: '{fact_text}'")
        return True

    def update_session_summary(self, user_id, summary_text):
        """Aggiorna il riassunto dell'ultima sessione per un utente."""
        profile = self._get_or_create_user_profile(user_id)
        memory = profile.setdefault("memory", {})
        memory["last_session_summary"] = summary_text
        rospy.loginfo(f"Updated session summary for {user_id}: '{summary_text}'")
    
    def get_preferred_task(self, user_id):
        profile = self._get_or_create_user_profile(user_id)
        prefs = profile.get("task_preferences", {})
        if not prefs:
            return None, 0
        preferred = max(prefs.keys(), key=lambda k: prefs[k].get("count", 0), default=None)
        return preferred, prefs.get(preferred, {}).get("count", 0)
        
    def get_world_knowledge(self):
        return self.kg.get("world_knowledge", {})

    def get_safety_rules(self):
        return self.kg.get("world_knowledge", {}).get("safety_rules", [])
