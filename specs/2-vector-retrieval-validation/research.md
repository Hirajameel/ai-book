# Research: Vector Retrieval and Validation

## Overview
This research document resolves unknowns for the vector retrieval and validation implementation based on existing Spec-1 implementation.

## Resolved Unknowns

### 1. Qdrant Collection Name
- **Decision**: Use "rag_embeddings" as the default collection name
- **Rationale**: Based on `QdrantConfig` class in `config.py`, the default collection name is "rag_embeddings". This was also confirmed in the contracts from Spec-1 which shows "rag_collection" as an example.
- **Implementation**: The collection name can be overridden via the `QDRANT_COLLECTION_NAME` environment variable, but defaults to "rag_embeddings"

### 2. Vector Dimension Size
- **Decision**: Use 1024 as the vector dimension size
- **Rationale**: Based on `QdrantConfig` class in `config.py`, the default vector_size is 1024. This matches the Cohere embedding model output for the "embed-english-v3.0" model when using "search_document" input type.
- **Implementation**: The vector size can be overridden via the `QDRANT_VECTOR_SIZE` environment variable, but defaults to 1024

### 3. Cohere API Configuration
- **Decision**: Use environment variable configuration for API key
- **Rationale**: Based on `EmbeddingConfig` class in `config.py`, the Cohere API key is loaded from the `COHERE_API_KEY` environment variable.
- **Implementation**: The system uses the "embed-english-v3.0" model with "search_document" input type by default

### 4. Qdrant Connection Details
- **Decision**: Use environment variable configuration for Qdrant connection
- **Rationale**: Based on `QdrantConfig` class in `config.py`, Qdrant connection details are loaded from environment variables:
  - `QDRANT_URL`: The Qdrant Cloud URL
  - `QDRANT_API_KEY`: The Qdrant API key
  - `QDRANT_COLLECTION_NAME`: The collection name (default: "rag_embeddings")
- **Implementation**: The storage.py file shows the exact connection pattern used in Spec-1

## Technical Implementation Patterns

### Qdrant Client Creation
- Pattern: `QdrantClient(url=qdrant_config.url, api_key=qdrant_config.api_key, timeout=10.0)`
- Includes connection testing with `client.get_collections()`

### Similarity Search Implementation
- The existing storage.py has methods to interact with Qdrant but doesn't show retrieval methods
- Need to implement search functionality using Qdrant's `search` method
- Should follow the same error handling patterns as the existing code

### Embedding Generation
- Use Cohere's embedding API with the same configuration as in embedder.py
- Follow the same model ("embed-english-v3.0") and input type ("search_document")
- Handle rate limits and errors appropriately

## Data Structure
- Content chunks are stored with metadata including URL, section information
- Each vector has a payload with "source_document_id", "associated_metadata", etc.
- The metadata structure follows the models.py definitions from Spec-1

## Best Practices Identified
- Use environment variables for configuration
- Implement comprehensive error handling with logging
- Follow Pydantic for configuration validation
- Use proper typing and documentation