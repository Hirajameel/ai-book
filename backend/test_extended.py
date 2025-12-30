#!/usr/bin/env python3
"""
Test script to verify the RAG agent is working properly with different queries
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

def test_queries(agent):
    """Test various queries to the agent"""
    queries = [
        "What is AI?",
        "Explain machine learning",
        "Tell me about this book",
        "What are the main topics?"
    ]

    for query in queries:
        print(f"\nTesting query: '{query}'...")
        try:
            response = agent.query_simple(query)
            print(f"[SUCCESS] Query successful. Response: {response[:200]}...")
        except Exception as e:
            print(f"[ERROR] Error running query: {str(e)}")
            import traceback
            traceback.print_exc()

def main():
    print("RAG Agent Extended Test Script")
    print("=" * 50)

    # Check if required environment variables are set
    print("Checking environment variables...")
    required_vars = ['OPENAI_API_KEY', 'COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"[WARNING] Missing required environment variables: {missing_vars}")
        print("Please set these variables before running the agent.")
        return False
    else:
        print("[SUCCESS] All required environment variables are set")

    # Test agent creation
    agent = test_agent_creation()
    if not agent:
        return False

    # Test multiple queries
    test_queries(agent)

    print("\n[RAG AGENT] Extended testing completed!")
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)