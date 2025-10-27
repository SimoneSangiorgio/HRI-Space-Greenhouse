#!/usr/bin/env python

import os
import json
import rospy
import signal
from groq import Groq
from dotenv import load_dotenv

load_dotenv()

class TimeoutException(Exception): pass

def with_timeout(seconds):
    def decorator(func):
        def wrapper(*args, **kwargs):
            def handler(signum, frame):
                raise TimeoutException(f"Function {func.__name__} timed out after {seconds} seconds")
            
            old_handler = signal.signal(signal.SIGALRM, handler)
            signal.alarm(seconds)
            
            try:
                result = func(*args, **kwargs)
            finally:
                signal.alarm(0)
                signal.signal(signal.SIGALRM, old_handler)
            return result
        return wrapper
    return decorator

TASKS_ROBOT_GREENHOUSE = {
    "OPERATIONAL_MANAGEMENT": ["self_diagnostic_check", "check_battery_level", "navigate_to_charging_station", "map_and_localize", "plan_optimal_path", "stop_current_task"],
    "ENVIRONMENTAL_MONITORING": ["read_temperature_humidity_sensors", "read_CO2_level", "read_light_intensity", "read_soil_moisture_at_plant", "report_environmental_anomalies"],
    "PLANT_CARE": ["image_analysis_for_health", "detect_pests_and_diseases", "execute_precision_watering", "apply_targeted_nutrients", "execute_selective_pruning", "identify_and_harvest_ripe_fruit"],
    "MAINTENANCE_AND_LOGISTICS": ["clean_floor_and_pathways", "sanitize_robotic_arm_tools", "monitor_water_and_nutrient_tanks", "transport_harvest_to_storage"]
}
def _flatten_tasks(tasks_dict):
    return [task for category in tasks_dict.values() for task in category]
ALL_TASKS_LIST = _flatten_tasks(TASKS_ROBOT_GREENHOUSE)

def _generate_persona_instructions(user_profile):
    role = user_profile.get("role", "Unknown") if user_profile else "Unknown"
    
    if role == "Commander":
        base_persona = "You are SERRA, a service robot reporting to your Commander. Your tone is respectful, concise, and mission-focused."
    elif role == "Scientist":
        base_persona = "You are SERRA. Your tone is collaborative, precise, and data-driven."
    else:
        base_persona = "You are SERRA, a friendly and patient guide robot. Your tone is simple and engaging."

    if user_profile:
        verbosity = user_profile.get("interaction_style", {}).get("verbosity_preference", "normal")
        if verbosity == "brief": base_persona += " This user prefers brief responses."
        elif verbosity == "verbose": base_persona += " This user appreciates detailed responses."
    
    return base_persona

def _build_context_string(context=None, history=None):
    parts = []
    if context and context.get('last_intent'):
        parts.append(f"LAST_INTENT: {context.get('last_intent')}")
    if history:
        history_str = "\n".join([f"{msg['role']}: {msg['content']}" for msg in history])
        parts.append(f"RECENT CONVERSATION HISTORY:\n---\n{history_str}\n---")
    return "\n\n".join(parts)

def _generate_intent_parsing_prompt(user_profile=None, context=None):
    persona = _generate_persona_instructions(user_profile)
    
    contextual_hint = ""
    if context and context.get('last_intent') == 'list_capabilities':
        contextual_hint = "CRITICAL CONTEXT: You have JUST listed your capabilities. The user's next input is highly likely to be a command selecting one of those tasks. Prioritize identifying a specific, actionable task."

    return f"""
{persona}
Your task is to interpret the user's LATEST request and classify it.
{contextual_hint}
Available "meta" intents: 'ask_for_recommendation', 'general_query', 'query_user_profile', 'query_rules_and_permissions', 'list_capabilities', 'acknowledgement'.
Respond ONLY with a JSON object: {{ "intent": "...", "original_request": "..." }}
"""

def _generate_recommendation_prompt(user_profile=None):
    persona = _generate_persona_instructions(user_profile)
    return f"""
{persona}
Suggest ONE actionable task from this list: {', '.join(ALL_TASKS_LIST)}.
CRITICAL: Respond in this exact format:
TASK: <task_name_from_list>
REASON: <Your concise explanation.>
If no action is applicable, respond with:
TASK: none
REASON: <Your explanation.>
"""

def _generate_conversational_prompt(user_profile=None):
    persona = _generate_persona_instructions(user_profile)
    return f"""
{persona}
Answer the user's LATEST question. Use the RECENT CONVERSATION HISTORY to avoid repeating information.
CRITICAL: Do not start your response with your name or any prefix like 'SERRA:'. Just provide the response directly.
"""

class LLMManager:
    def __init__(self):
        try:
            self.client = Groq()
            rospy.loginfo("LLM Manager (Groq) initialized successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Groq client. Error: {e}")
            self.client = None

    @with_timeout(10)
    def parse_intent(self, user_input, context=None, user_profile=None, history=None):
        try:
            # Controllo hardcoded per meta-intenti semplici e affidabili
            user_input_lower = user_input.lower().strip()
            if user_input_lower in ["ok", "okay", "got it", "thanks", "thank you", "perfect", "fine", "all right, go ahead", "good robot", "yes", "yep", "y"]:
                return {"intent": "acknowledgement", "is_question": False, "confidence": 1.0, "original_request": user_input}
            if any(phrase in user_input_lower for phrase in ["what can you do", "your capabilities", "your functions", "what are your tasks"]):
                return {"intent": "list_capabilities", "is_question": True, "confidence": 1.0, "original_request": user_input}
            
            if not self.client: return {"intent": "error", "reason": "LLM client not initialized."}
            
            # Se nessun match, usa l'LLM con il contesto completo
            prompt = _generate_intent_parsing_prompt(user_profile, context)
            messages = [{"role": "system", "content": prompt}]
            full_context_str = _build_context_string(context, history)
            messages.append({"role": "user", "content": f"{full_context_str}\n\nUser Request: '{user_input}'"})
            
            chat_completion = self.client.chat.completions.create(
                messages=messages, model="llama-3.1-8b-instant", temperature=0.1, max_tokens=200,
            )
            return json.loads(chat_completion.choices[0].message.content.strip())
        except TimeoutException as e:
            rospy.logerr(str(e))
            return {"intent": "error", "reason": "Intent parsing timed out."}
        except Exception as e:
            rospy.logerr(f"Error calling Groq API for intent parsing: {e}")
            return {"intent": "error", "reason": str(e)}

    @with_timeout(15)
    def get_recommendation(self, system_context, user_goal="", user_profile=None, history=None):
        try:
            if not self.client: return "TASK: none\nREASON: Recommendation system offline."
            prompt = _generate_recommendation_prompt(user_profile)
            full_context_str = _build_context_string(history=history)
            prompt_with_context = (f"{full_context_str}\n\nUser Goal: \"{user_goal}\"\n\n" f"Current System Context: {system_context}")
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "system", "content": prompt}, {"role": "user", "content": prompt_with_context}],
                model="llama-3.1-8b-instant", temperature=0.2, max_tokens=250,
            )
            return chat_completion.choices[0].message.content
        except TimeoutException as e:
            rospy.logerr(str(e))
            return "TASK: none\nREASON: Recommendation generation timed out."
        except Exception as e:
            rospy.logerr(f"Error calling Groq API for recommendation: {e}")
            return "TASK: none\nREASON: Error formulating recommendation."

    @with_timeout(15)
    def get_conversational_response(self, user_question, context, user_profile=None, history=None):
        try:
            if not self.client: return "Conversational system offline."
            prompt = _generate_conversational_prompt(user_profile)
            full_context_str = _build_context_string(history=history)
            full_prompt = f"{full_context_str}\n\nData Context:\n{json.dumps(context, indent=2)}\n\nBased on all the above, answer the user's question: '{user_question}'"
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "system", "content": prompt}, {"role": "user", "content": full_prompt}],
                model="llama-3.1-8b-instant", temperature=0.7, max_tokens=512,
            )
            return chat_completion.choices[0].message.content
        except TimeoutException as e:
            rospy.logerr(str(e))
            return "Conversational response generation timed out."
        except Exception as e:
            rospy.logerr(f"Error calling Groq API for conversation: {e}")
            return "Error formulating response."