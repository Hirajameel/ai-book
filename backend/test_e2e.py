"""
End-to-end test for the RAG Agent API integration

This script tests the complete flow from API request to response.
"""

import asyncio
import json
from api import app
from models import APIQueryRequest, APIQueryResponse
from fastapi.testclient import TestClient


def test_end_to_end():
    """
    Test the end-to-end flow of the API
    """
    print("Testing end-to-end flow...")

    # Create a test client
    client = TestClient(app)

    # Test 1: Health check
    print("\n1. Testing health endpoint...")
    response = client.get("/api/health")
    assert response.status_code == 200
    health_data = response.json()
    assert health_data["status"] == "healthy"
    print("SUCCESS: Health check passed")

    # Test 2: Query endpoint with full-book context
    print("\n2. Testing query endpoint with full-book context...")
    query_data = {
        "query": "What is AI?",
        "context_type": "full_book"
    }

    response = client.post("/api/query", json=query_data)
    print(f"Response status: {response.status_code}")

    if response.status_code == 200:
        query_response = response.json()
        print(f"Success: {query_response['success']}")
        if query_response['success'] and query_response.get('data'):
            print(f"Answer preview: {query_response['data']['answer'][:100]}...")
        else:
            print(f"Error: {query_response.get('error', 'No error message')}")
    else:
        print(f"Query failed with status {response.status_code}")
        print(f"Response: {response.text}")

    # Test 3: Query validation
    print("\n3. Testing query validation...")
    invalid_query_data = {
        "query": "",  # Empty query should fail validation
        "context_type": "full_book"
    }

    response = client.post("/api/query", json=invalid_query_data)
    print(f"Invalid query response status: {response.status_code}")
    if response.status_code in [400, 422]:
        print("SUCCESS: Validation correctly rejected invalid query")

    # Test 4: Invalid context type
    print("\n4. Testing invalid context type...")
    invalid_context_data = {
        "query": "Test query",
        "context_type": "invalid_context"
    }

    response = client.post("/api/query", json=invalid_context_data)
    print(f"Invalid context response status: {response.status_code}")
    if response.status_code in [400, 422]:
        print("SUCCESS: Validation correctly rejected invalid context type")

    print("\nEnd-to-end tests completed!")


if __name__ == "__main__":
    test_end_to_end()