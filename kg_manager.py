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
        # Registra il metodo di salvataggio per essere chiamato alla chiusura di ROS.
        # Questo centralizza il salvataggio e lo rende più robusto.
        rospy.on_shutdown(self.save_kg)

    def load_kg(self):
        # Gestione robusta del caricamento per prevenire crash dovuti a file corrotti.
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
                self.load_kg() # Riprova a caricare dopo il ripristino
            except Exception as e:
                rospy.logerr(f"Failed to restore from backup: {e}. Starting fresh.")
                self._create_default_kg()
        else:
            rospy.logwarn("No backup file found. Starting a fresh KG.")
            self._create_default_kg()
            
    def _create_default_kg(self):
        # Centralizzata la creazione del KG di default
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
        # Implementato un salvataggio "atomico" per prevenire la corruzione del file.
        # 1. Crea un backup del file esistente.
        # 2. Scrive su un file temporaneo.
        # 3. Rinomina il file temporaneo a quello finale (operazione atomica).
        tmp_path = self.file_path + ".tmp"
        bak_path = self.file_path + ".bak"
        try:
            if os.path.exists(self.file_path):
                shutil.copy(self.file_path, bak_path) # Crea backup

            with open(tmp_path, 'w') as f:
                json.dump(self.kg, f, indent=4)
            
            os.rename(tmp_path, self.file_path) # Operazione atomica di sostituzione
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
                "role": role,
                "task_preferences": {},
                "interaction_style": { "verbosity_preference": "normal" },
                "interaction_count": 0,
                "first_seen": datetime.now().isoformat(),
                "last_seen": datetime.now().isoformat()
            }
        return profiles[user_id]

    def get_user_profile(self, user_id):
        # Assicura che 'last_seen' sia sempre aggiornato quando si accede al profilo
        profile = self._get_or_create_user_profile(user_id)
        profile["last_seen"] = datetime.now().isoformat()
        return profile

    def log_interaction(self, user_id, intent):
        profile = self._get_or_create_user_profile(user_id)
        prefs = profile.setdefault("task_preferences", {})
        prefs[intent] = prefs.get(intent, 0) + 1
        
        profile["interaction_count"] = profile.get("interaction_count", 0) + 1
        profile["last_seen"] = datetime.now().isoformat()
        
        # Rimosso il salvataggio esplicito. Verrà gestito dall'hook on_shutdown.
        rospy.loginfo(f"Logged interaction: {user_id} performed {intent}")

    def update_interaction_style(self, user_id, command_text):
        profile = self._get_or_create_user_profile(user_id)
        style = profile.setdefault("interaction_style", {})
        
        command_length = len(command_text.split())
        if command_length <= 3: current_verbosity = 'brief'
        elif command_length >= 8: current_verbosity = 'verbose'
        else: current_verbosity = 'normal'
        style["verbosity_preference"] = current_verbosity

    def get_preferred_task(self, user_id):
        profile = self._get_or_create_user_profile(user_id)
        prefs = profile.get("task_preferences")
        if not prefs: return None, 0
        preferred = max(prefs, key=prefs.get)
        return preferred, prefs.get(preferred, 0)
        
    def get_world_knowledge(self):
        return self.kg.get("world_knowledge", {})

    def get_safety_rules(self):
        return self.kg.get("world_knowledge", {}).get("safety_rules", [])
