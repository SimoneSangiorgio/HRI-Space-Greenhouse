#!/usr/bin/env python

import os
import json
import rospy
import signal
from groq import Groq
from dotenv import load_dotenv
from collections import deque

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
    if not user_profile:
        return "You are SERRA, a friendly and helpful service robot in a greenhouse."

    profile = user_profile.get("profile", {})
    style = user_profile.get("interaction_style", {})
    relationship = user_profile.get("relationship", {})
    memory = user_profile.get("memory", {})
    role = profile.get("role", "User")
    address_method = profile.get("preferred_address", "by_role")
    name_to_use = role if address_method == "by_role" else user_profile.get("name_from_id", role)
    persona = f"You are SERRA, a service robot. You are interacting with {name_to_use}, whose role is '{role}'."
    formality = style.get("formality", "neutral")
    tone = style.get("tone_preference", "helpful")
    verbosity = style.get("verbosity_preference", "normal")
    persona += f"\nYour communication style with this user must be: {formality} in formality, with a {tone} tone."
    if verbosity == "brief": persona += " The user prefers brief, to-the-point responses. Avoid unnecessary details."
    elif verbosity == "verbose": persona += " The user appreciates detailed and thorough explanations."
    rel_level = relationship.get("relationship_level", 0.1)
    if rel_level > 0.7: persona += " You have a strong, established relationship with this user; you can be more collaborative and proactive."
    elif rel_level > 0.3: persona += " You are familiar with this user and their general preferences."
    else: persona += " You are still getting to know this user. Be helpful but maintain a professional distance."
    key_facts = memory.get("key_facts", [])
    if key_facts:
        facts_str = "; ".join(key_facts)
        persona += f"\nCRITICAL USER-SPECIFIC MEMORY: Remember these facts about the user: '{facts_str}'."
    return persona

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
        contextual_hint = "CRITICAL CONTEXT: You have JUST listed your capabilities. The user's next input is highly likely to be a command selecting one of those tasks."
    if context and context.get('last_intent_status') == 'failed':
        contextual_hint += f"\nCRITICAL CONTEXT: The last task '{context.get('last_intent')}' failed. The user might be asking about the failure or trying a different approach."
    intent_examples = """
CRITICAL EXAMPLES:
- User says: "Status" -> {"intent": "general_query", "original_request": "Status"}
- User says: "Ok, thanks" -> {"intent": "acknowledgement", "original_request": "Ok, thanks"}
- User says: "Recommend a task" -> {"intent": "ask_for_recommendation", "original_request": "Recommend a task"}
- User says: "Remember that the nutrient mix for tomatoes is formula B" -> {"intent": "remember_fact", "fact": "The nutrient mix for tomatoes is formula B", "original_request": "Remember that the nutrient mix for tomatoes is formula B"}
- User says: "I want to play" -> {"intent": "query_rules_and_permissions", "original_request": "I want to play"}
- User says: "Can we play a game?" -> {"intent": "query_rules_and_permissions", "original_request": "Can we play a game?"}
"""
    return f"""
{persona}
Your primary task is to interpret the user's LATEST request, classify its intent, and extract relevant information.

{contextual_hint}
{intent_examples}

Available "meta" intents:
- 'ask_for_recommendation': User asks for a suggestion on what task to perform.
- 'general_query': User asks for information (e.g., status reports, data). THIS IS FOR CONCISE COMMANDS LIKE "Status".
- 'remember_fact': User explicitly asks you to remember a piece of information. You MUST extract the information into a "fact" field.
- 'list_capabilities': User asks what you can do.
- 'acknowledgement': User gives a simple confirmation or thanks.
- 'query_user_profile': User is asking about themselves.
- 'query_rules_and_permissions': User is asking about rules, permissions, or wants to do something recreational like 'play'.

Respond ONLY with a valid JSON object.
- For 'remember_fact', the format MUST be: {{"intent": "remember_fact", "fact": "the complete fact to remember", "original_request": "..."}}
- For all other intents, the format is: {{"intent": "...", "original_request": "..."}}
"""

def _generate_recommendation_prompt(user_profile=None):
    persona = _generate_persona_instructions(user_profile)
    user_context_hint = ""
    if user_profile:
        prefs = user_profile.get("task_preferences", {})
        if prefs:
            preferred_task = max(prefs.keys(), key=lambda k: prefs[k].get("count", 0), default=None)
            if preferred_task:
                user_context_hint = f"USER PROFILE HINT: This user frequently performs '{preferred_task}'. If relevant, prioritize suggestions related to this task or category."
    return f"""
{persona}
{user_context_hint}
Suggest ONE actionable robotic task from this list: {', '.join(ALL_TASKS_LIST)}.
Base your suggestion on the current system context and the user's likely goals.
If no action is applicable or the user's goal cannot be met by a task, respond with:
TASK: none
REASON: <Your explanation.>

CRITICAL: Respond in this exact format:
TASK: <task_name_from_list>
REASON: <Your concise explanation.>
"""

def _generate_conversational_prompt(user_profile=None):
    persona = _generate_persona_instructions(user_profile)
    return f"""
{persona}
Answer the user's LATEST question. Use the RECENT CONVERSATION HISTORY to avoid repeating information.
Use your general knowledge combined with the provided context to provide a helpful and relevant response.
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
            user_input_lower = user_input.lower().strip()
            if user_input_lower in ["ok", "okay", "got it", "thanks", "thank you", "perfect", "fine", "all right, go ahead", "good robot", "yes", "yep", "y"]:
                return {"intent": "acknowledgement", "original_request": user_input}
            if any(phrase in user_input_lower for phrase in ["what can you do", "your capabilities", "your functions", "what are your tasks"]):
                return {"intent": "list_capabilities", "original_request": user_input}
            
            if not self.client: return {"intent": "error", "reason": "LLM client not initialized."}
            
            prompt = _generate_intent_parsing_prompt(user_profile, context)
            messages = [{"role": "system", "content": prompt}]
            full_context_str = _build_context_string(context, history)
            messages.append({"role": "user", "content": f"{full_context_str}\n\nUser Request: '{user_input}'"})
            
            chat_completion = self.client.chat.completions.create(
                messages=messages, model="llama-3.1-8b-instant", temperature=0.0, max_tokens=300,
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
        
    @with_timeout(15)
    def get_session_summary(self, history: deque):
        """Genera un breve riassunto di una conversazione."""
        if not self.client or len(history) < 3: # Non riassumere conversazioni troppo brevi
            return None

        history_str = "\n".join([f"{msg['role']}: {msg['content']}" for msg in history])
        
        prompt = (
            "You are a summarization assistant. Your task is to summarize the following conversation "
            "from the user's perspective in one short, past-tense sentence. "
            "Focus only on the main tasks performed or key information discussed. "
            "For example, if the user checked the status and watered a plant, a good summary is: "
            "'checked the plant status and then watered the lemon tree'. "
            "If the user asked to be called by a new name, a good summary is: "
            "'updated my preferred name'. "
            "Be concise and start the sentence with a verb."
        )

        try:
            chat_completion = self.client.chat.completions.create(
                messages=[
                    {"role": "system", "content": prompt},
                    {"role": "user", "content": f"Please summarize this conversation:\n\n{history_str}"}
                ],
                model="llama-3.1-8b-instant",
                temperature=0.1,
                max_tokens=100,
            )
            summary = chat_completion.choices[0].message.content.strip().strip('"')
            return summary
        except TimeoutException as e:
            rospy.logerr(str(e))
            return "had a conversation that timed out during summarization."
        except Exception as e:
            rospy.logerr(f"Error calling Groq API for summarization: {e}")
            return "had a conversation that ended with an error."
