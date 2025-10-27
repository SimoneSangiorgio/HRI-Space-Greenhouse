#!/usr/bin/env python
# llm_manager.py
# This is a direct, feature-complete adaptation of the provided mall parser example,
# tailored specifically for the greenhouse project.

import rospy
import json
import os
from groq import Groq
from dotenv import load_dotenv

load_dotenv()

class LLMManager:
    def __init__(self):
        api_key = os.environ.get("GROQ_API_KEY")
        self.client = Groq(api_key=api_key) if api_key else None
        if not self.client:
            rospy.logerr("LLM_ERROR: GROQ_API_KEY not found. LLM calls will fail.")
        else:
            rospy.loginfo("LLM Manager (Groq) initialized.")

        # --- Entity lists populated by set_knowledge_base_entities ---
        self.valid_roles = []
        self.valid_tasks = {}            # Maps task ID to task name
        self.valid_equipment = {}        # Maps equipment ID to its name

    def set_knowledge_base_entities(self, roles, tasks, equipment):
        """
        Sets the lists of known entities from the knowledge graph to ground the LLM.
        - roles: List of user roles (e.g., ["Commander", "GreenhouseEmployee"])
        - tasks: List of dicts, e.g., [{"id": "check_resources", "name": "Check Resources"}]
        - equipment: List of dicts, e.g., [{"id": "water_tank_1", "name": "Water Tank 1"}]
        """
        self.valid_roles = roles
        self.valid_tasks = {t['id']: t['name'] for t in tasks}
        self.valid_equipment = {e['id']: e['name'] for e in equipment}
        rospy.loginfo("LLM knowledge base entities have been set.")

    def _call_groq_api(self, prompt, max_tokens=400):
        if not self.client:
            rospy.logerr("LLM Error: Groq client not initialized.")
            # Return a mock error that matches the expected structure of different parsers
            if "parse_fulfillment_status" in prompt:
                return {"fulfilled": None, "error": "Groq client not available"}
            elif "parse_next_action" in prompt:
                 return {"intent": "stop_interaction", "details": None, "error": "Groq client not available"}
            return {"error": "Groq client not available"}

        try:
            chat_completion = self.client.chat.completions.create(
                messages=[{"role": "user", "content": prompt}],
                model="llama-3.1-8b-instant",
                temperature=0.1,
                max_tokens=max_tokens,
                response_format={"type": "json_object"},
                timeout=20.0
            )
            raw_output = chat_completion.choices[0].message.content
            # Basic cleanup, though response_format helps
            cleaned_output = raw_output.strip().replace("```json", "").replace("```", "").strip()
            return json.loads(cleaned_output)
        except json.JSONDecodeError as e:
            rospy.logerr(f"LLM JSON Decode Error: {e} in output: '{cleaned_output if 'cleaned_output' in locals() else 'Unavailable'}'")
            return {"error": "JSON Decode Error", "details": str(e)}
        except Exception as e:
            rospy.logerr(f"Groq API Error or Timeout: {e}")
            return {"error": "Groq API Error", "details": str(e)}

    # --- THIS IS THE FIX: Added 'self' as the first argument ---
    def parse_main_query(self, user_query, current_user_role, user_profile_context=None):
        """
        Equivalent to `generate_structured_query`. It's the main entry point for parsing user intent.
        It understands the user's goal, targets, and learns new preferences from the query itself.
        """
        profile_str = json.dumps(user_profile_context, indent=2) if user_profile_context else "{}"

        prompt = f"""
        You are the command interpretation brain for a space assistant robot in a greenhouse.
        The person speaking is the '{current_user_role}'.
        Analyze their query.

        Return STRICTLY VALID JSON with these fields:
        - "intent": (string) "perform_task", "get_specific_status", "show_preferences", "update_preferences_only", or "stop_interaction".
        - "target_user": (string) The user profile being referenced or acted upon. Must be one of {self.valid_roles}. Default to the current speaker if not specified.
        - "target_task_id": (string, optional) The ID of the task to perform.
        - "target_equipment_id": (string, optional) The ID of the equipment to query.
        - "updated_preferences": (object, optional) If the query reveals a new or changed preference for the 'target_user', include this. Structure: {{"preferred_tasks": ["task_id"], "task_specific_preferences": {{"task_id": {{"notes": ["new note"]}}}}}}.

        CONTEXT - USER PROFILE:
        This is the known profile for the '{current_user_role}':
        {profile_str}

        AVAILABLE ENTITIES:
        - TASKS: {json.dumps(self.valid_tasks)}
        - EQUIPMENT/SENSORS: {json.dumps(self.valid_equipment)}
        - USER ROLES: {self.valid_roles}

        RULES:
        1. Always output a single, valid JSON object. No explanations.
        2. Match informal language to the correct ID (e.g., "water level" -> "water_tank_1").
        3. If the user states a new habit (e.g., "I always check resources first"), capture it in "updated_preferences".

        Examples:
        Query: "Let's check the resources." -> {{"intent": "perform_task", "target_user": "{current_user_role}", "target_task_id": "check_resources"}}
        Query: "How much power is left in the solar storage?" -> {{"intent": "get_specific_status", "target_user": "{current_user_role}", "target_equipment_id": "solar_storage"}}
        Query: "From now on, when I ask for status, always include the basil plant." -> {{"intent": "update_preferences_only", "target_user": "{current_user_role}", "updated_preferences": {{"task_specific_preferences": {{"get_greenhouse_status": {{"plants_of_interest": ["plant_1_basil"]}}}}}}}}
        Query: "What are the Commander's typical tasks?" -> {{"intent": "show_preferences", "target_user": "Commander"}}
        Query: "That's it for now." -> {{"intent": "stop_interaction"}}

        Current Query: "{user_query}"
        Respond ONLY with valid JSON:
        """
        parsed_json = self._call_groq_api(prompt)
        if "error" in parsed_json: return parsed_json
        return self._validate_main_query(parsed_json, current_user_role)

    def _validate_main_query(self, output_json, current_user_role):
        """
        A direct equivalent to `_validate_output_v3`. It cleans and validates the LLM's output
        to ensure all IDs and values are legitimate before the main controller uses them.
        """
        validated = {
            "intent": "get_specific_status", # A safe default
            "target_user": current_user_role,
            "target_task_id": None,
            "target_equipment_id": None,
            "updated_preferences": {}
        }
        if not isinstance(output_json, dict):
            rospy.logerr(f"LLM output was not a dict: {output_json}")
            validated["error"] = "Invalid LLM output format"
            return validated

        # Validate intent
        intent = output_json.get("intent")
        if intent in ["perform_task", "get_specific_status", "show_preferences", "update_preferences_only", "stop_interaction"]:
            validated["intent"] = intent
        
        # Validate target_user
        user = output_json.get("target_user")
        if user in self.valid_roles:
            validated["target_user"] = user

        # Validate target_task_id
        task_id = output_json.get("target_task_id")
        if task_id in self.valid_tasks:
            validated["target_task_id"] = task_id
        
        # Validate target_equipment_id
        equip_id = output_json.get("target_equipment_id")
        if equip_id in self.valid_equipment:
            validated["target_equipment_id"] = equip_id

        # Validate and extract updated_preferences (a simplified version of _extract_profile_updates)
        updates = output_json.get("updated_preferences")
        if isinstance(updates, dict):
            validated["updated_preferences"] = updates # In a real system, you'd validate this deeper
        
        return validated

    def parse_feedback(self, feedback_text, last_target_id=None):
        """
        A direct equivalent to `parse_feedback_to_profile_update`. It extracts structured
        feedback from a user's general comments.
        """
        prompt = f"""
        You are a profile update assistant analyzing user feedback about greenhouse equipment.
        The user was recently discussing: '{self.valid_equipment.get(last_target_id, 'an unspecified item')}'.
        User Feedback: "{feedback_text}"

        Analyze the feedback and return STRICTLY VALID JSON with the structure:
        {{
          "feedback_updates": [
            {{
              "target_equipment_id": "string, ID from AVAILABLE EQUIPMENT",
              "status_implied": "string, 'nominal', 'anomaly', or 'unclear'",
              "notes_positive": ["list of positive strings"],
              "notes_negative": ["list of negative strings"]
            }}
          ]
        }}
        
        AVAILABLE EQUIPMENT: {json.dumps(self.valid_equipment)}

        Examples:
        Feedback: "The readings from tank 1 seem a bit jumpy today, but the basil plant is looking much healthier."
        Output: {{
          "feedback_updates": [
            {{"target_equipment_id": "water_tank_1", "status_implied": "anomaly", "notes_positive": [], "notes_negative": ["Sensor readings seem unstable/jumpy."]}},
            {{"target_equipment_id": "plant_1_basil", "status_implied": "nominal", "notes_positive": ["Appears healthier than before."], "notes_negative": []}}
          ]
        }}

        User Feedback: "{feedback_text}"
        """
        return self._call_groq_api(prompt)

    def parse_fulfillment_status(self, user_response, context_question):
        """
        A direct equivalent to the function of the same name. Determines if a request was satisfied.
        """
        prompt = f"""
        Analyze the user's response to determine if their request was fulfilled.
        The robot asked: "{context_question}"
        User's response: "{user_response}"
        Respond with JSON: {{"fulfilled": true | false | null}}
        - true: User is satisfied.
        - false: User is not satisfied or wants more.
        - null: User is unsure or ambiguous.
        Examples:
        Response: "Yes, that's perfect." -> {{"fulfilled": true}}
        Response: "It's okay, but can you also tell me..." -> {{"fulfilled": false}}
        Response: "I guess so." -> {{"fulfilled": null}}
        """
        return self._call_groq_api(prompt, max_tokens=100)

    def parse_next_action(self, user_response, last_task_summary):
        """
        A direct equivalent to `parse_next_action_decision`. Figures out the user's next goal.
        """
        prompt = f"""
        The user just finished a task related to: "{last_task_summary}".
        They were asked what to do next. Their response was: "{user_response}"
        Determine their intent. Respond with JSON:
        {{
          "intent": "string, 'new_task', 'continue_task', or 'stop_interaction'",
          "details": "string or null, describing the new goal if applicable"
        }}
        Examples:
        Response: "Okay, now let's water the plants." -> {{"intent": "new_task", "details": "water the plants"}}
        Response: "Give me the temperature now." -> {{"intent": "continue_task", "details": "get temperature"}}
        Response: "No, that's everything." -> {{"intent": "stop_interaction", "details": null}}
        """
        return self._call_groq_api(prompt, max_tokens=150)

if __name__ == '__main__':
    print("--- Testing FULLY FEATURED Greenhouse LLM Manager ---")
    parser = LLMManager()
    if not parser.client: exit()

    # Define mock knowledge base entities for standalone testing
    mock_roles = ["Commander", "GreenhouseEmployee"]
    mock_tasks = [{"id": "check_resources", "name": "Check Resources"}]
    mock_equipment = [
        {"id": "water_tank_1", "name": "Water Tank 1"},
        {"id": "plant_1_basil", "name": "Basil Plant"},
        {"id": "solar_storage", "name": "Solar Storage"}
    ]
    parser.set_knowledge_base_entities(roles=mock_roles, tasks=mock_tasks, equipment=mock_equipment)

    print("\n--- Test 1: Main Query Parsing & Validation ---")
    queries = [
        ("How much power is left?", "Commander"),
        ("I need to check the dilithium crystals.", "Commander") # Should be validated out
    ]
    for q, role in queries:
        print(f"Query: '{q}' (as {role})")
        result = parser.parse_main_query(q, role)
        print(f"Parsed -> {json.dumps(result, indent=2)}")

    print("\n--- Test 2: Detailed Feedback Parsing ---")
    feedback = "The basil is looking a little yellow, but the solar panels are working great."
    print(f"Feedback: '{feedback}'")
    result = parser.parse_feedback(feedback, "plant_1_basil")
    print(f"Parsed -> {json.dumps(result, indent=2)}")

    print("\n--- Test 3: Fulfillment Status Parsing ---")
    response = "That report was fine, but not exactly what I needed."
    print(f"Response: '{response}'")
    result = parser.parse_fulfillment_status(response, "Was the resource report satisfactory?")
    print(f"Parsed -> {json.dumps(result, indent=2)}")

    print("\n--- Test 4: Next Action Parsing ---")
    response = "Great. Now, can you run a diagnostic on the water systems?"
    print(f"Response: '{response}'")
    result = parser.parse_next_action(response, "providing a resource report")
    print(f"Parsed -> {json.dumps(result, indent=2)}")