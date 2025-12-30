# Research: FastAPI RAG Integration

## Overview
This research document addresses the technical requirements for implementing the FastAPI RAG integration feature as specified in the feature specification.

## Decision: FastAPI Backend Implementation
**Rationale**: FastAPI was chosen as the backend framework based on the project constraints and requirements. It provides excellent performance, built-in API documentation, and async support which is ideal for RAG applications.

**Alternatives considered**:
- Flask: Simpler but less performant for async operations
- Django: More complex than needed for this API-only use case
- Express.js: Would require switching to Node.js ecosystem

## Decision: Agent Integration Strategy
**Rationale**: The existing agent from `backend/agent.py` (from Spec-3) will be integrated directly into the FastAPI application. This maintains code reuse and leverages the existing RAG implementation.

**Integration approach**: The agent will be called synchronously from the FastAPI endpoint, with proper error handling and response formatting.

## Decision: Frontend Integration
**Rationale**: The existing Docusaurus frontend in `frontend-docu/` will be used as-is with added chatbot UI components. This maintains consistency with the existing book structure.

**Approach**: A chatbot component will be added to the Docusaurus pages that can make API calls to the FastAPI backend.

## Decision: API Endpoint Design
**Rationale**: A single query endpoint will be created to handle both full-book queries and context-restricted queries based on selected text.

**Endpoint**: `POST /api/query` with JSON payload containing the query text and optional context restrictions.

## Decision: Response Format
**Rationale**: JSON responses will be used to maintain compatibility with web frontend requirements.

**Format**: Responses will include the agent's answer, source citations, and metadata in a structured JSON format.

## Technical Unknowns Resolved
- **Frontend location**: Confirmed as `frontend-docu/` directory
- **Agent location**: Confirmed as `backend/agent.py`
- **Integration method**: Direct function calls from FastAPI to agent
- **Query types**: Both full-book and selected-text restricted queries supported