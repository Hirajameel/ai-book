#!/usr/bin/env python3
"""
Test script to verify the RAG agent works with both OpenAI and Gemini configurations
"""

import os
import sys
import logging

# Add backend directory to path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from agent import RAGAgent
from config import get_agent_config

def test_agent_creation():
    """Test that the RAG agent can be created without errors"""
    print("Testing RAG agent creation...")
    try:
        agent = RAGAgent()
        print("[SUCCESS] RAG agent created successfully")
        print(f"  - OpenAI client: {'Available' if agent.openai_client else 'Not configured'}")
        print(f"  - Gemini client: {'Available' if agent.gemini_client else 'Not configured'}")
        return agent
    except Exception as e:
        print(f"[ERROR] Error creating RAG agent: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def test_query_with_fallback(agent, query="What is artificial intelligence?"):
    """Test a query that should work with fallback mechanisms"""
    print(f"\nTesting query: '{query}'...")
    try:
        response = agent.query_simple(query)
        print(f"[SUCCESS] Query successful. Response: {response[:200]}...")
        return True
    except Exception as e:
        print(f"[ERROR] Error running query: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("RAG Agent Test Script with Gemini Support")
    print("=" * 50)

    # Check if required environment variables are set
    print("Checking environment variables...")
    required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']  # Only required ones
    optional_vars = ['OPENAI_API_KEY', 'GEMINI_API_KEY']

    missing_required = [var for var in required_vars if not os.getenv(var)]
    available_optional = [var for var in optional_vars if os.getenv(var)]

    if missing_required:
        print(f"[ERROR] Missing required environment variables: {missing_required}")
        print("Please set these variables before running the agent.")
        return False
    else:
        print("[SUCCESS] All required environment variables are set")

    if available_optional:
        print(f"[INFO] Available optional APIs: {available_optional}")
    else:
        print("[INFO] No optional APIs configured (will only use retrieved content)")

    # Test agent creation
    agent = test_agent_creation()
    if not agent:
        return False

    # Test query with fallback
    success = test_query_with_fallback(agent)

    if success:
        print("\n[RAG AGENT] is working properly with fallback mechanisms!")
        return True
    else:
        print("\n[RAG AGENT] has issues.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)