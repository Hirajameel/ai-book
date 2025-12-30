# FastAPI RAG Integration - API Contract

## Overview
This document defines the API contract for the RAG integration feature, specifying the endpoints and data structures for communication between the frontend and backend.

## Base URL
`http://localhost:8000` (for local development)

## Endpoints

### POST /api/query
Process user queries against the RAG system

#### Request
**Content-Type**: `application/json`

**Body**:
```json
{
  "query": "What is the main concept discussed in chapter 3?",
  "context_type": "full_book",
  "selected_text": "Optional text selected by the user for context-restricted queries"
}
```

**Fields**:
- `query` (string, required): The user's query text (1-2000 characters)
- `context_type` (string, required): Either "full_book" or "selected_text"
- `selected_text` (string, optional): Text content for context-restricted queries

#### Response
**Success Response (200 OK)**:
```json
{
  "success": true,
  "data": {
    "answer": "The main concept in chapter 3 is...",
    "sources": [
      {
        "source_id": "ch3-section1",
        "source_title": "Chapter 3, Section 1",
        "content_snippet": "The main concept discussed is...",
        "relevance_score": 0.95
      }
    ],
    "query_id": "query-12345"
  }
}
```

**Error Response (400 Bad Request)**:
```json
{
  "success": false,
  "error": "Invalid query parameters"
}
```

**Server Error Response (500 Internal Server Error)**:
```json
{
  "success": false,
  "error": "Internal server error occurred while processing query"
}
```

### GET /api/health
Check the health status of the API

#### Response
**Success Response (200 OK)**:
```json
{
  "status": "healthy",
  "timestamp": "2025-01-05T10:00:00Z"
}
```

## Data Models

### QueryRequest
- `query`: string (required) - The user's query text
- `context_type`: string (required) - Either "full_book" or "selected_text"
- `selected_text`: string (optional) - Text content for context-restricted queries

### QueryResponse
- `success`: boolean (required) - Whether the query was processed successfully
- `data`: object (optional) - The response data if successful
  - `answer`: string (required) - The agent's answer to the query
  - `sources`: array (required) - List of source citations
  - `query_id`: string (required) - Unique identifier for the query
- `error`: string (optional) - Error message if the query failed

### SourceCitation
- `source_id`: string (required) - Unique identifier for the source
- `source_title`: string (required) - Title of the source material
- `content_snippet`: string (required) - Excerpt from the source material
- `relevance_score`: number (required) - How relevant the source was to the response

### HealthResponse
- `status`: string (required) - Health status of the API
- `timestamp`: string (required) - ISO 8601 timestamp of the check

## Error Codes
- `400`: Bad Request - Invalid request parameters
- `422`: Unprocessable Entity - Request validation failed
- `500`: Internal Server Error - Server error occurred