# RAG Agent API Documentation

## Overview
This API provides endpoints for querying book content using a RAG (Retrieval-Augmented Generation) agent. It allows users to ask questions about book content and receive contextually relevant answers with source citations.

## API Endpoints

### Health Check
- **GET** `/api/health`
- **Description**: Check the health status of the API
- **Response**:
  ```json
  {
    "status": "healthy",
    "timestamp": "2025-01-05T10:00:00Z"
  }
  ```

### Query Endpoint
- **POST** `/api/query`
- **Description**: Process user queries against book content
- **Request Body**:
  ```json
  {
    "query": "string (required, 1-2000 characters)",
    "context_type": "string (required, either 'full_book' or 'selected_text')",
    "selected_text": "string (optional, required when context_type is 'selected_text')"
  }
  ```
- **Headers**:
  - `Content-Type: application/json`
- **Response**:
  ```json
  {
    "success": true,
    "data": {
      "answer": "string",
      "sources": [
        {
          "url": "string",
          "section": "string",
          "content_preview": "string",
          "relevance_score": "number (0-1)"
        }
      ],
      "query_id": "string"
    }
  }
  ```
- **Error Response**:
  ```json
  {
    "success": false,
    "error": "string"
  }
  ```

## Context Types

### Full Book Context
When `context_type` is `"full_book"`, the query is processed against the entire book content.

### Selected Text Context
When `context_type` is `"selected_text"`, the query is processed only against the provided `selected_text` content.

## Error Codes

- `400 Bad Request`: Invalid request parameters
- `422 Unprocessable Entity`: Request validation failed
- `500 Internal Server Error`: Server error occurred

## Examples

### Query with Full Book Context
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is artificial intelligence?",
    "context_type": "full_book"
  }'
```

### Query with Selected Text Context
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What does this text explain?",
    "context_type": "selected_text",
    "selected_text": "Artificial Intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence."
  }'
```

## Running the API

### Development
```bash
cd backend
python api.py
```

The API will be available at `http://localhost:8000`

## Dependencies

- Python 3.11+
- FastAPI
- uvicorn
- Required packages listed in `requirements.txt`