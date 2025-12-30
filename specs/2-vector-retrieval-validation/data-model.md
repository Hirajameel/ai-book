# Data Model: Vector Retrieval and Validation

**Feature**: Vector Retrieval and Pipeline Validation
**Created**: 2025-12-27
**Status**: Draft

## Overview

This document describes the data structures used in the vector retrieval and validation system. The system retrieves content chunks from a Qdrant vector database and validates their structure against the schema established in Spec-1.

## Entities

### Query Input
**Description**: Represents a text query input from the user for similarity search

**Attributes**:
- `query_text` (str): The text content of the user's query
- `top_k` (int): Number of results to retrieve (default: 5)

**Validation Rules**:
- `query_text` must not be empty or whitespace only
- `top_k` must be a positive integer (1 or greater)

### Embedding Vector
**Description**: Represents a numerical vector representation of text for similarity comparison

**Attributes**:
- `vector` (List[float]): The embedding vector representation
- `dimensions` (int): Size of the embedding vector (1024 for Cohere embed-english-v3.0)

**Validation Rules**:
- `vector` must be a list of floats
- `vector` length must match the expected embedding dimension (1024 for Cohere)
- `dimensions` must match the configured vector size from Spec-1 (1024)

### Retrieved Chunk
**Description**: Represents a content chunk retrieved from the vector database with metadata

**Attributes**:
- `id` (str): Unique identifier from Qdrant
- `content` (str): Text content of the retrieved chunk
- `metadata` (dict): Metadata including URL, section, and other information as stored in Qdrant payload
  - `source_document_id` (str): ID of the source document
  - `associated_metadata` (dict): Additional metadata like URL, section info
    - `url` (str): Source URL of the content
    - `section` (str): Section or heading where content was found
  - `created_at` (str): Timestamp of when the chunk was created
- `similarity_score` (float): Relevance score from similarity search (0.0 to 1.0)

**Validation Rules**:
- `id` must be a valid identifier from Qdrant
- `content` must not be empty
- `metadata` must contain required fields (source_document_id, associated_metadata)
- `similarity_score` must be between 0.0 and 1.0
- `associated_metadata` must contain URL and section information

### Search Results
**Description**: Collection of retrieved chunks with metadata about the search

**Attributes**:
- `query` (QueryInput): The original query that generated these results
- `retrieved_chunks` (List[RetrievedChunk]): Ordered list of retrieved content chunks
- `total_results` (int): Total number of results returned
- `search_time_ms` (float): Time taken to perform the search in milliseconds

**Validation Rules**:
- `retrieved_chunks` must be ordered by similarity_score (descending)
- `total_results` must match the length of `retrieved_chunks`
- `search_time_ms` must be non-negative

### Configuration
**Description**: Configuration parameters for the retrieval system

**Attributes**:
- `qdrant_url` (str): URL of the Qdrant Cloud instance
- `qdrant_api_key` (str): API key for Qdrant authentication
- `qdrant_collection_name` (str): Name of the collection to query
- `qdrant_vector_size` (int): Size of vectors in the collection (1024)
- `cohere_api_key` (str): API key for Cohere embedding service
- `cohere_model_name` (str): Name of the Cohere model to use ("embed-english-v3.0")
- `default_top_k` (int): Default number of results to return (5)

**Validation Rules**:
- `qdrant_url` must be a valid URL
- `qdrant_api_key` must not be empty
- `qdrant_collection_name` must follow Qdrant naming conventions
- `qdrant_vector_size` must match the embedding model's output (1024 for Cohere)
- `cohere_api_key` must not be empty
- `default_top_k` must be a positive integer

## Relationships

- **Query Input** → **Embedding Vector**: One query generates one embedding vector
- **Embedding Vector** → **Retrieved Chunks**: One embedding vector is used to retrieve multiple content chunks
- **Search Results** → **Retrieved Chunks**: One search result contains multiple retrieved chunks
- **Configuration** → **All entities**: Configuration parameters affect how all other entities are processed

## Schema Compliance

The retrieved chunks must match the schema established in Spec-1:
- Each chunk corresponds to a vector record in Qdrant
- Metadata structure must match the payload format from Spec-1
- Vector dimensions must be 1024 to match Cohere's embed-english-v3.0 model
- All required metadata fields from Spec-1 must be preserved