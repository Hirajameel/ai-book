# Quickstart Guide: FastAPI RAG Integration

## Overview
This guide provides instructions for setting up and running the FastAPI RAG integration feature for local development.

## Prerequisites
- Python 3.11+
- Node.js (for Docusaurus frontend)
- Access to Qdrant vector database
- Access to OpenAI API or compatible service

## Setup Instructions

### 1. Backend Setup
1. Navigate to the backend directory:
   ```bash
   cd backend/
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   # Or if using uv:
   uv pip install -r requirements.txt
   ```

3. Set up environment variables by copying `.env.example` to `.env`:
   ```bash
   cp .env.example .env
   ```

4. Update the `.env` file with your configuration:
   - Qdrant connection details
   - API keys for OpenAI or compatible service
   - Any other required environment variables

### 2. Start the FastAPI Server
1. Run the API server:
   ```bash
   cd backend/
   python api.py
   ```

   Or if using uv:
   ```bash
   uv run python api.py
   ```

2. The server will start on `http://localhost:8000`

### 3. Frontend Setup
1. Navigate to the frontend directory:
   ```bash
   cd frontend-docu/
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the Docusaurus development server:
   ```bash
   npm start
   ```

4. The frontend will be available at `http://localhost:3000`

## API Usage

### Making a Query Request
Send a POST request to `http://localhost:8000/api/query` with the following JSON payload:

```json
{
  "query": "What is the main concept discussed in chapter 3?",
  "context_type": "full_book"
}
```

For a context-restricted query:
```json
{
  "query": "Explain this concept further",
  "context_type": "selected_text",
  "selected_text": "The main concept discussed in this chapter is..."
}
```

### Example Response
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

## Frontend Integration
The chatbot UI will be integrated into the Docusaurus pages, allowing users to:
1. Select text in the book content
2. Ask questions about the full book or selected text
3. View responses with source citations

## Troubleshooting

### Common Issues
- **API Connection Errors**: Ensure the backend server is running on the correct port
- **Environment Variables**: Verify all required environment variables are set
- **Database Connection**: Check Qdrant connection settings in your environment

### Checking Service Health
Visit `http://localhost:8000/api/health` to check if the backend is running properly.