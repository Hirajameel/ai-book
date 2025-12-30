#!/usr/bin/env python3
"""
Test script to verify the RAG agent is working properly
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
        return agent
    except Exception as e:
        print(f"[ERROR] Error creating RAG agent: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def test_simple_query(agent, query="What is this book about?"):
    """Test a simple query to the agent"""
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
    print("RAG Agent Test Script")
    print("=" * 50)

    # Check if required environment variables are set
    print("Checking environment variables...")
    required_vars = ['OPENAI_API_KEY', 'COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"[WARNING] Missing required environment variables: {missing_vars}")
        print("Please set these variables before running the agent.")
        print("Example: export OPENAI_API_KEY='your-key-here'")
        return False
    else:
        print("[SUCCESS] All required environment variables are set")

    # Test agent creation
    agent = test_agent_creation()
    if not agent:
        return False

    # Test simple query
    success = test_simple_query(agent)

    if success:
        print("\n[RAG AGENT] is working properly!")
        return True
    else:
        print("\n[RAG AGENT] has issues.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)