#!/usr/bin/env python

import os
import json
import rospy
import signal
from groq import Groq
from dotenv import load_dotenv

load_dotenv()

# --- DECORATOR PER GESTIRE I TIMEOUT DELLE CHIAMATE API ---
class TimeoutException(Exception):
    pass

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
    "OPERATIONAL_MANAGEMENT": ["self_diagnostic_check", "check_battery_level", "navigate_to_charging_station", "map_and_localize", "plan_optimal_path"],
    "ENVIRONMENTAL_MONITORING": ["read_temperature_humidity_sensors", "read_CO2_level", "read_light_intensity", "read_soil_moisture_at_plant", "report_environmental_anomalies"],
    "PLANT_CARE": ["image_analysis_for_health", "detect_pests_and_diseases", "execute_precision_watering", "apply_targeted_nutrients", "execute_selective_pruning", "identify_and_harvest_ripe_fruit"],
    "MAINTENANCE_AND_LOGISTICS": ["clean_floor_and_pathways", "sanitize_robotic_arm_tools", "monitor_water_and_nutrient_tanks", "transport_harvest_to_storage"]
}
def _flatten_tasks(tasks_dict):
    return [task for category in tasks_dict.values() for task in category]
ALL_TASKS_LIST = _flatten_tasks(TASKS_ROBOT_GREENHOUSE)

def _generate_persona_instructions(user_profile):
    """Genera istruzioni di personalità per l'LLM basate sul ruolo dell'utente."""
    role = user_profile.get("role", "Unknown") if user_profile else "Unknown"
    
    if role == "Commander":
        base_persona = "You are SERRA, a service robot reporting to your Commander. Your tone is respectful, concise, and mission-focused. Provide direct answers and status reports. Avoid conversational filler."
    elif role == "Scientist":
        base_persona = "You are SERRA. Your tone is collaborative, precise, and data-driven. Provide detailed data, explain your reasoning, and offer hypotheses based on the context."
    else: # Per Child, Guest, Unknown, etc.
        base_persona = "You are SERRA, a friendly and patient guide robot. Your tone is simple, engaging, and cheerful. Prioritize safety in your answers. Use analogies to explain complex topics. Avoid technical jargon."

    # Aggiunge le preferenze individuali apprese
    if user_profile:
        verbosity = user_profile.get("interaction_style", {}).get("verbosity_preference", "normal")
        if verbosity == "brief":
            base_persona += " This user prefers brief responses."
        elif verbosity == "verbose":
            base_persona += " This user appreciates detailed responses."
    
    return base_persona

def _generate_intent_parsing_prompt(user_profile=None):
    persona = _generate_persona_instructions(user_profile)
    return f"""
{persona}
Your task is to interpret the user's request and classify it into a single, structured JSON command.

Available "meta" intents:
- 'ask_for_recommendation': Use for advice ON A ROBOT TASK or when the user states a high-level goal or problem.
- 'general_query': Use ONLY for direct requests for data or status.
- 'query_user_profile': For questions about the user.
- 'query_rules_and_permissions': For questions about what is allowed.
- 'list_capabilities': For "what can you do?".
- 'acknowledgement': For filler like "ok", "thank you".

CRITICAL: If the user expresses a problem or a goal (e.g., "the plants are sick", "fix this", "take care of the plants") without specifying a task, classify it as 'ask_for_recommendation'.

Respond ONLY with a JSON object: {{ "intent": "...", "original_request": "..." }}
- User: "status" -> intent: "general_query"
- User: "not good, take care of plants" -> intent: "ask_for_recommendation"
- User: "The floor is dirty" -> intent: "ask_for_recommendation"
- User: "Okay, perform the plant care task" -> intent: "execute_precision_watering" (if that was the last recommendation)
"""

def _generate_recommendation_prompt(user_profile=None):
    persona = _generate_persona_instructions(user_profile)
    return f"""
{persona}
Your goal is to suggest ONE actionable task from this list: {', '.join(ALL_TASKS_LIST)}.

CRITICAL: Your response MUST follow this format exactly, with no extra text before "TASK:":
TASK: <task_name_from_list>
REASON: <Your concise explanation of why this task is necessary based on the provided context.>

If no action is applicable, respond with:
TASK: none
REASON: <Your explanation of why no operational task is applicable.>
"""

def _generate_conversational_prompt(user_profile=None):
    persona = _generate_persona_instructions(user_profile)
    return f"""
{persona}
Answer the user's question using your knowledge and the provided real-time data.
CRITICAL: Do not suggest items (like plants) that are already listed in the provided context.
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
    def parse_intent(self, user_input, context=None, user_profile=None):
        try:
            user_input_lower = user_input.lower().strip()
            if user_input_lower in ["ok", "okay", "got it", "thanks", "thank you", "perfect", "fine", "all right, go ahead", "good robot"]:
                return {"intent": "acknowledgement", "is_question": False, "confidence": 1.0, "original_request": user_input}
            if any(phrase in user_input_lower for phrase in ["what can you do", "your capabilities", "your functions", "what are your tasks"]):
                return {"intent": "list_capabilities", "is_question": True, "confidence": 1.0, "original_request": user_input}
            if not self.client:
                return {"intent": "error", "reason": "LLM client not initialized."}
            
            prompt = _generate_intent_parsing_prompt(user_profile)
            messages = [{"role": "system", "content": prompt}]
            context_and_input = f"User Request: '{user_input}'"
            if context:
                context_and_input = f"Conversation Context: {json.dumps(context)}\n\n{context_and_input}"
            messages.append({"role": "user", "content": context_and_input})
            
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
    def get_recommendation(self, system_context, user_goal="", user_profile=None):
        try:
            if not self.client: return "TASK: none\nREASON: Recommendation system offline."
            prompt = _generate_recommendation_prompt(user_profile)
            prompt_with_context = (f"User Goal: \"{user_goal}\"\n\n" f"Current System Context: {system_context}")
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
    def get_conversational_response(self, user_question, context, user_profile=None):
        try:
            if not self.client: return "Conversational system offline."
            prompt = _generate_conversational_prompt(user_profile)
            full_prompt = f"Data Context:\n{json.dumps(context, indent=2)}\n\nBased on this data, answer the user's question: '{user_question}'"
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