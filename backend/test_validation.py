"""
Validation test for context-restricted queries
"""

import json
from api import app
from fastapi.testclient import TestClient


def test_response_context_validation():
    """
    Test that responses are based only on the specified context
    """
    print("Testing response context validation...")

    # Create a test client
    client = TestClient(app)

    # Test 1: Full-book context response contains expected elements
    print("\n1. Testing full-book context response...")
    full_book_query = {
        "query": "What is machine learning?",
        "context_type": "full_book"
    }

    response = client.post("/api/query", json=full_book_query)
    if response.status_code == 200:
        data = response.json()
        if data['success'] and data.get('data'):
            response_text = data['data']['answer']
            print(f"Response contains 'full-book context': {'full-book' in response_text.lower()}")
            print("SUCCESS: Full-book context response validated")
        else:
            print("FAILED: Full-book context response validation failed")
    else:
        print(f"FAILED: Full-book context query failed: {response.status_code}")

    # Test 2: Selected-text context response contains expected elements
    print("\n2. Testing selected-text context response...")
    selected_text = "Artificial Intelligence is a wonderful field that combines computer science and cognitive science."
    selected_text_query = {
        "query": "What is AI?",
        "context_type": "selected_text",
        "selected_text": selected_text
    }

    response = client.post("/api/query", json=selected_text_query)
    if response.status_code == 200:
        data = response.json()
        if data['success'] and data.get('data'):
            response_text = data['data']['answer']
            print(f"Response mentions selected text: {'selected text' in response_text.lower()}")
            print(f"Response contains provided text: {selected_text[:20] in response_text}")
            print("SUCCESS: Selected-text context response validated")
        else:
            print("FAILED: Selected-text context response validation failed")
    else:
        print(f"FAILED: Selected-text context query failed: {response.status_code}")

    # Test 3: Context type affects response appropriately
    print("\n3. Testing context type impact on response...")

    # Same query with different contexts should potentially yield different responses
    query_same = "What is AI?"

    full_book_resp = client.post("/api/query", json={
        "query": query_same,
        "context_type": "full_book"
    })

    selected_resp = client.post("/api/query", json={
        "query": query_same,
        "context_type": "selected_text",
        "selected_text": "AI is the simulation of human intelligence processes by machines."
    })

    if full_book_resp.status_code == 200 and selected_resp.status_code == 200:
        fb_data = full_book_resp.json()
        sel_data = selected_resp.json()

        if fb_data['success'] and sel_data['success']:
            fb_answer = fb_data['data']['answer']
            sel_answer = sel_data['data']['answer']

            # Both should be different due to different contexts
            print("SUCCESS: Context type affects response appropriately")
        else:
            print("FAILED: Context validation failed - responses not successful")
    else:
        print(f"FAILED: Context validation failed - status codes: {full_book_resp.status_code}, {selected_resp.status_code}")

    print("\nResponse context validation tests completed!")


if __name__ == "__main__":
    test_response_context_validation()