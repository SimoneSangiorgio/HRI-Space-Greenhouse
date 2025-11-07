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
    "OPERATIONAL_MANAGEMENT": ["self_diagnostic_check", "check_battery_level", "navigate_to_charging_station", "navigate_to_location", "map_and_localize", "plan_optimal_path", "stop_current_task", "execute_task_sequence"],
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

    known_locations = ["start point", "plant 1", "plant 2", "harvesting zone", "maintenance area", "charging station", "water tank"]
    location_hint = f"HINT: Known locations are: {', '.join(known_locations)}. 'plant 1' maps to 'plant 1', 'plant 2' to 'plant 2', etc."
    
    predefined_sequences = ["general greenhouse check"]
    sequence_hint = f"HINT: Pre-defined sequences which can be triggered by name: {', '.join(predefined_sequences)}."

    permission_rule = "CRITICAL RULE: If the user asks for permission (e.g., 'Can I...', 'Am I allowed to...'), the intent is ALWAYS 'query_rules_and_permissions', even if the sentence contains action words like 'go' or 'touch'."
    
    intent_examples = f"""
CRITICAL EXAMPLES:
- User says: "watering plant 1" -> {{"intent": "execute_precision_watering", "target": "plant 1", "original_request": "watering plant 1"}}
- User says: "go to plant 1 and plant 2" -> {{"intent": "execute_task_sequence", "tasks": [{{"intent": "navigate_to_location", "location": "plant 1"}}, {{"intent": "navigate_to_location", "location": "plant 2"}}], "original_request": "go to plant 1 and plant 2"}}
- User says: "can i go into the greenhouse?" -> {{"intent": "query_rules_and_permissions", "original_request": "can i go into the greenhouse?"}}
- User says: "Perform a general greenhouse check" -> {{"intent": "execute_task_sequence", "sequence_name": "general greenhouse check", "original_request": "Perform a general greenhouse check"}}
"""

    return f"""
{persona}
Your primary task is to interpret the user's LATEST request and convert it into a JSON object.
Your goal is to identify the user's high-level intent.

{permission_rule}
CRITICAL RULE: If the user asks to go to MULTIPLE places (e.g., 'go to A and B'), you MUST use the 'execute_task_sequence' intent, creating a list of 'navigate_to_location' tasks.
CRITICAL RULE: A complex action with a SINGLE target (e.g., 'water plant 1') should be parsed as a SINGLE high-level intent, NOT as a sequence.

{location_hint}
{sequence_hint}
{contextual_hint}
{intent_examples}

Available Intent Categories:
- Task Intents: 'navigate_to_location', 'execute_precision_watering', etc. Use for DIRECT COMMANDS to the robot for a SINGLE target.
- Sequence Intents: 'execute_task_sequence'. Use for multi-step commands, commands with multiple targets, or named routines.
- Permission & Rules Intents: 'query_rules_and_permissions'. Use when the user asks what THEY can do.
- Conversational Intents: 'greeting', 'farewell', 'small_talk', 'contextual_question'.
- Meta Intents: 'remember_fact', 'acknowledgement'.

Respond ONLY with a valid JSON object.
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
Your task is to propose ONE single, actionable robotic task based on the system context.
The task MUST be chosen from this list: {', '.join(ALL_TASKS_LIST)}.
You MUST respond with a valid JSON object in the following format.

CRITICAL FORMAT:
{{
  "task": {{ "intent": "task_name_from_list", "target": "relevant_location_if_needed" }},
  "reason": "Your concise explanation for why this task is necessary."
}}

EXAMPLE: If the data shows low moisture for a plant at 'plant 2', your response should be:
{{
  "task": {{ "intent": "execute_precision_watering", "target": "plant 2" }},
  "reason": "The soil moisture for the plant at plant 2 is critically low."
}}

If no action is necessary, respond with 'task' as null:
{{
  "task": null,
  "reason": "All systems are operating within nominal parameters. No immediate action is required."
}}

Do NOT add any text outside the JSON object.
"""

def _generate_conversational_prompt(user_profile=None):
    persona = _generate_persona_instructions(user_profile)
    style_guideline = ""

    if user_profile:
        style = user_profile.get("interaction_style", {})
        memory = user_profile.get("memory", {})
        
        if style.get("verbosity_preference") == 'brief':
            style_guideline = "\nCRITICAL INSTRUCTION: The user prefers brief, to-the-point, schematic responses. Provide only the most essential information in a list or bullet points if possible."
        
        key_facts = memory.get("key_facts", [])
        for fact in key_facts:
            fact_lower = fact.lower()
            if "short" in fact_lower or "schematic" in fact_lower or "brief" in fact_lower:
                style_guideline = "\nCRITICAL INSTRUCTION: The user has explicitly requested short and schematic responses. Be as concise as possible. Use bullet points for status reports."
                break

    return f"""
{persona}
{style_guideline}

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
            
            sanitized_input = user_input.replace("'", "\\'").replace('"', '\\"')
            messages.append({"role": "user", "content": f"{full_context_str}\n\nUser Request: '{sanitized_input}'"})
            
            chat_completion = self.client.chat.completions.create(
                messages=messages, model="llama-3.1-8b-instant", temperature=0.0, max_tokens=512,
            )
            result = json.loads(chat_completion.choices[0].message.content.strip())
            result['original_request'] = user_input
            return result
        except TimeoutException as e:
            rospy.logerr(str(e))
            return {"intent": "error", "reason": "Intent parsing timed out."}
        except Exception as e:
            rospy.logerr(f"Error calling Groq API for intent parsing: {e}")
            return {"intent": "error", "reason": str(e)}

    @with_timeout(15)
    def get_recommendation(self, system_context, user_goal="", user_profile=None, history=None):
        try:
            if not self.client: return '{"task": null, "reason": "Recommendation system offline."}'
            prompt = _generate_recommendation_prompt(user_profile)
            full_context_str = _build_context_string(history=history)
            
            sanitized_goal = user_goal.replace("'", "\\'").replace('"', '\\"')
            prompt_with_context = (f"{full_context_str}\n\nUser Goal: \"{sanitized_goal}\"\n\n" f"Current System Context: {system_context}")
            
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "system", "content": prompt}, {"role": "user", "content": prompt_with_context}],
                model="llama-3.1-8b-instant", temperature=0.2, max_tokens=300,
            )
            return chat_completion.choices[0].message.content
        except TimeoutException as e:
            rospy.logerr(str(e))
            return '{"task": null, "reason": "Recommendation generation timed out."}'
        except Exception as e:
            rospy.logerr(f"Error calling Groq API for recommendation: {e}")
            return '{"task": null, "reason": "Error formulating recommendation."}'

    @with_timeout(15)
    def get_conversational_response(self, user_question, context, user_profile=None, history=None):
        try:
            if not self.client: return "Conversational system offline."
            prompt = _generate_conversational_prompt(user_profile)
            full_context_str = _build_context_string(history=history)

            sanitized_question = user_question.replace("'", "\\'").replace('"', '\\"')
            full_prompt = f"{full_context_str}\n\nData Context:\n{json.dumps(context, indent=2)}\n\nBased on all the above, answer the user's question: '{sanitized_question}'"
            
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
        if not self.client or len(history) < 3:
            return None
        history_str = "\n".join([f"{msg['role']}: {msg['content']}" for msg in history])
        
        prompt = (
            "You are a summarization assistant. Your task is to summarize the following conversation "
            "from the USER's perspective in one short, past-tense sentence. "
            "The summary MUST start with 'I' followed by a past-tense verb. "
            "Focus only on the main tasks the user performed or key information they discussed. "
            "For example, if the user checked the status and watered a plant, a good summary is: "
            "'I checked the plant status and then watered the lemon tree'. "
            "If the user asked to be called by a new name, a good summary is: "
            "'I updated my preferred name'. "
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
            if not summary.lower().startswith("i "):
                summary = "I " + summary[0].lower() + summary[1:]

            return summary
        except TimeoutException as e:
            rospy.logerr(str(e))
            return "had a conversation that timed out during summarization."
        except Exception as e:
            rospy.logerr(f"Error calling Groq API for summarization: {e}")
            return "had a conversation that ended with an error."