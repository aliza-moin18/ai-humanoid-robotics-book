#!/usr/bin/env python3
"""
Simple test script for the RetrievalAgent
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.agent import RetrievalAgent
from backend.agent_types import ConversationContext


def test_agent():
    print("Testing RetrievalAgent...")
    
    try:
        # Create the agent
        agent = RetrievalAgent()
        print("[PASS] Agent created successfully")

        # Test a simple query
        query = "What is artificial intelligence?"
        print(f"Testing query: '{query}'")

        response = agent.process_query(query)
        print(f"[PASS] Response received: {len(response.answer)} characters")
        print(f"[PASS] Citations found: {len(response.citations)}")

        # Test with conversation context (follow-up)
        context = ConversationContext(
            previous_query="What is artificial intelligence?",
            previous_response="Artificial intelligence is a branch of computer science...",
            session_id="test-session-123"
        )

        follow_up_query = "Can you elaborate on machine learning?"
        print(f"Testing follow-up query: '{follow_up_query}'")

        follow_up_response = agent.process_query(follow_up_query, context)
        print(f"[PASS] Follow-up response received: {len(follow_up_response.answer)} characters")
        print(f"[PASS] Follow-up citations found: {len(follow_up_response.citations)}")

        print("\n[SUCCESS] All tests passed!")

    except Exception as e:
        print(f"[ERROR] Error during testing: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_agent()