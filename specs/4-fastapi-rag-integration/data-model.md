# Data Model: FastAPI RAG Integration

## Overview
This document defines the data models for the FastAPI RAG integration feature, based on the entities identified in the feature specification.

## Core Entities

### Query
**Description**: A text-based request from the user seeking information about the book content
**Fields**:
- `query_text` (string): The user's query text
- `context_type` (string): Type of query context ("full_book" or "selected_text")
- `selected_text` (string, optional): Specific text selected by the user for context-restricted queries
- `user_id` (string, optional): Identifier for the user making the query

### Response
**Description**: The system-generated answer based on the RAG agent's analysis of the book content
**Fields**:
- `response_text` (string): The agent's response to the query
- `source_citations` (array of objects): List of source citations for the response
- `query_id` (string): Unique identifier for the query
- `timestamp` (datetime): When the response was generated
- `metadata` (object): Additional metadata about the response

### Source Citation
**Description**: Reference to the source material used in generating the response
**Fields**:
- `source_id` (string): Unique identifier for the source
- `source_title` (string): Title of the source material
- `content_snippet` (string): Excerpt from the source material
- `relevance_score` (number): How relevant the source was to the response

### Selected Text Context
**Description**: A subset of book content that the user has specifically selected for focused queries
**Fields**:
- `text_content` (string): The selected text content
- `start_position` (number): Starting position of the selection in the book
- `end_position` (number): Ending position of the selection in the book
- `context_id` (string): Unique identifier for this context

## API Request/Response Models

### Query Request Model
**Purpose**: Structure for incoming query requests to the API
**Fields**:
- `query` (string): The user's query text
- `context_type` (string): Either "full_book" or "selected_text"
- `selected_text` (string, optional): Text content for context-restricted queries

### Query Response Model
**Purpose**: Structure for outgoing response from the API
**Fields**:
- `success` (boolean): Whether the query was processed successfully
- `data` (object): The response data
  - `answer` (string): The agent's answer to the query
  - `sources` (array): List of source citations
  - `query_id` (string): Unique identifier for the query
- `error` (string, optional): Error message if the query failed

## Validation Rules

### Query Validation
- `query_text` must be between 1 and 2000 characters
- `context_type` must be either "full_book" or "selected_text"
- If `context_type` is "selected_text", `selected_text` must be provided and not empty

### Response Validation
- `response_text` must not be empty
- `source_citations` must be an array of valid citation objects
- `query_id` must be a unique identifier

## State Transitions

### Query States
- `pending`: Query received, waiting for processing
- `processing`: RAG agent is processing the query
- `completed`: Response generated successfully
- `failed`: Error occurred during processing