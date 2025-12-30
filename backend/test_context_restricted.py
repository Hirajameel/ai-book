"""
Test script for context-restricted query functionality
"""

import json
from api import app
from models import APIQueryRequest
from fastapi.testclient import TestClient


def test_context_restricted_queries():
    """
    Test the context-restricted query functionality
    """
    print("Testing context-restricted query functionality...")

    # Create a test client
    client = TestClient(app)

    # Test 1: Full-book query (should work)
    print("\n1. Testing full-book context query...")
    full_book_query = {
        "query": "What is AI?",
        "context_type": "full_book"
    }

    response = client.post("/api/query", json=full_book_query)
    print(f"Full-book query status: {response.status_code}")
    if response.status_code == 200:
        data = response.json()
        print(f"Success: {data['success']}")
        if data['success'] and data.get('data'):
            print(f"Response preview: {data['data']['answer'][:50]}...")
    else:
        print(f"Full-book query failed: {response.text}")

    # Test 2: Selected-text context query (should work with selected text)
    print("\n2. Testing selected-text context query...")
    selected_text_query = {
        "query": "What does this text say?",
        "context_type": "selected_text",
        "selected_text": "Artificial Intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence."
    }

    response = client.post("/api/query", json=selected_text_query)
    print(f"Selected-text query status: {response.status_code}")
    if response.status_code == 200:
        data = response.json()
        print(f"Success: {data['success']}")
        if data['success'] and data.get('data'):
            print(f"Response preview: {data['data']['answer'][:50]}...")
    else:
        print(f"Selected-text query failed: {response.text}")

    # Test 3: Selected-text context query without selected text (should fail)
    print("\n3. Testing selected-text query without selected text (should fail)...")
    invalid_query = {
        "query": "What does this text say?",
        "context_type": "selected_text"
        # Missing selected_text field
    }

    response = client.post("/api/query", json=invalid_query)
    print(f"Invalid selected-text query status: {response.status_code}")
    if response.status_code in [400, 422]:
        print("SUCCESS: Validation correctly rejected query without selected text")

    # Test 4: Invalid context type
    print("\n4. Testing invalid context type...")
    invalid_context_query = {
        "query": "What is AI?",
        "context_type": "invalid_type"
    }

    response = client.post("/api/query", json=invalid_context_query)
    print(f"Invalid context query status: {response.status_code}")
    if response.status_code in [400, 422]:
        print("SUCCESS: Validation correctly rejected invalid context type")

    print("\nContext-restricted query tests completed!")


if __name__ == "__main__":
    test_context_restricted_queries()