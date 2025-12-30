# Quickstart: Vector Retrieval and Validation

**Feature**: Vector Retrieval and Pipeline Validation
**Created**: 2025-12-27
**Status**: Draft

## Overview

This guide helps you quickly set up and run the vector retrieval validation tool. This tool allows developers to validate their RAG retrieval pipeline by querying a vector database with text queries, generating embeddings, and retrieving relevant content chunks with metadata.

## Prerequisites

- Python 3.8+
- Access to Cohere API (API key)
- Access to Qdrant Cloud (URL and API key)
- Existing vector collection from Spec-1 with embeddings

## Installation

1. Navigate to the backend directory:
   ```bash
   cd backend/
   ```

2. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   # Or install specific packages:
   pip install qdrant-client cohere python-dotenv pydantic
   ```

## Configuration

1. Copy the example environment file:
   ```bash
   cp .env.example .env
   ```

2. Edit the `.env` file with your credentials:
   ```bash
   # Cohere API configuration
   COHERE_API_KEY=your_cohere_api_key_here

   # Qdrant configuration
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_COLLECTION_NAME=rag_embeddings
   QDRANT_VECTOR_SIZE=1024
   ```

## Basic Usage

Run the retrieval validation tool with a simple query:

```bash
python retrieve.py --query "your search query here"
```

### Command Line Options

- `--query`: Text query to search for (required)
- `--top-k`: Number of results to retrieve (default: 5)
- `--collection`: Name of the Qdrant collection to query (default: "rag_embeddings")

### Examples

1. Basic retrieval with default settings:
   ```bash
   python retrieve.py --query "machine learning concepts in the book"
   ```

2. Retrieve top 10 results:
   ```bash
   python retrieve.py --query "neural networks explained" --top-k 10
   ```

3. Query specific collection:
   ```bash
   python retrieve.py --query "RAG systems" --collection "my_custom_collection"
   ```

## Expected Output

The tool will display:
- Retrieved content chunks with their text content
- Metadata including source URL and section information
- Similarity scores for each result (higher scores = more relevant)
- Validation information about embedding dimensions

Example output:
```
Query: "machine learning concepts"
Query embedding generated (1024 dimensions)

Retrieved 5 results:
1. [0.892] Section: Introduction to ML - URL: https://example.com/ml-intro
   Content: "Machine learning is a subset of artificial intelligence that enables systems..."

2. [0.845] Section: Neural Networks - URL: https://example.com/neural-networks
   Content: "Neural networks are computing systems inspired by the human brain..."

3. [0.789] Section: Supervised Learning - URL: https://example.com/supervised
   Content: "Supervised learning uses labeled training data to learn a mapping..."
```

## Troubleshooting

1. **Connection errors**: Verify QDRANT_URL and QDRANT_API_KEY in your environment
2. **API errors**: Check COHERE_API_KEY in your environment
3. **No results**: Ensure the collection name is correct and contains data
4. **Dimension mismatch**: Verify QDRANT_VECTOR_SIZE matches the embedding dimensions in your collection

## Next Steps

- Review the implementation plan for advanced configuration options
- Explore the data model to understand the structure of retrieved content
- Check the specification for detailed functional requirements