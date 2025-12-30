"""
Retrieval tool for the RAG Agent system.

This module provides functions to retrieve relevant content chunks from Qdrant
vector database based on query embeddings generated using Cohere.
"""

import logging
from typing import List
from config import get_agent_config
from models import RetrievedContentChunk


def retrieve_content_chunks(query: str, top_k: int = 5) -> List[RetrievedContentChunk]:
    """
    Retrieve relevant content chunks from Qdrant based on a query string.

    Args:
        query (str): The natural language query string
        top_k (int): Number of top results to retrieve (default: 5)

    Returns:
        List[RetrievedContentChunk]: List of retrieved content chunks with metadata
    """
    try:
        # Get agent configuration
        config = get_agent_config()

        # Import the required functions from retrieve.py
        from retrieve import create_cohere_client, create_qdrant_client, generate_query_embedding

        # Create Cohere client for embedding generation
        cohere_client = create_cohere_client(type('obj', (object,), {
            'api_key': config.cohere_api_key,
            'model_name': config.cohere_model_name,
            'input_type': config.cohere_input_type
        })())

        # Create Qdrant client for vector search
        qdrant_client = create_qdrant_client(type('obj', (object,), {
            'url': config.qdrant_url,
            'api_key': config.qdrant_api_key,
            'collection_name': config.qdrant_collection_name,
            'vector_size': config.qdrant_vector_size,
            'distance_metric': config.qdrant_distance_metric
        })())

        # Generate embedding for the query
        query_embedding = generate_query_embedding(cohere_client, query, type('obj', (object,), {
            'api_key': config.cohere_api_key,
            'model_name': config.cohere_model_name,
            'input_type': config.cohere_input_type
        })())

        # Perform similarity search in Qdrant
        search_results = qdrant_client.query_points(
            collection_name=config.qdrant_collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        # Format results as RetrievedContentChunk objects
        retrieved_chunks = []
        for result in search_results.points:
            # Get the payload from the result
            payload = result.payload
            associated_metadata = payload.get("associated_metadata", {})

            # Extract content from associated_metadata
            content = associated_metadata.get("content_text", "N/A")
            if content == "N/A":
                content = associated_metadata.get("content", "N/A")
            if content == "N/A":
                content = associated_metadata.get("text", "N/A")
            if content == "N/A":
                content = payload.get("content", "N/A")

            # Extract URL and section from associated_metadata
            url = associated_metadata.get("source_url", "N/A")
            if url == "N/A":
                url = associated_metadata.get("url", "N/A")
            if url == "N/A":
                url = payload.get("source_url", "N/A")

            section = associated_metadata.get("section_heading", "N/A")
            if section == "N/A":
                section = associated_metadata.get("section", "N/A")
            if section == "N/A":
                section = associated_metadata.get("heading", "N/A")
            if section == "N/A":
                section = payload.get("section", "N/A")

            # Create RetrievedContentChunk object
            chunk = RetrievedContentChunk(
                id=str(result.id),
                content=content,
                similarity_score=result.score,
                metadata=associated_metadata,
                source_url=url,
                section_heading=section,
                chunk_index=associated_metadata.get("chunk_index", 0)
            )

            retrieved_chunks.append(chunk)

        logging.info(f"Retrieved {len(retrieved_chunks)} content chunks for query: {query[:50]}...")
        return retrieved_chunks

    except Exception as e:
        logging.error(f"Error during content retrieval: {str(e)}")
        raise


def validate_retrieved_chunks(chunks: List[RetrievedContentChunk]) -> bool:
    """
    Validate that retrieved chunks meet requirements (non-empty, proper metadata).

    Args:
        chunks (List[RetrievedContentChunk]): List of retrieved content chunks to validate

    Returns:
        bool: True if all chunks are valid, False otherwise
    """
    for chunk in chunks:
        # Check that content is not empty
        if not chunk.content or chunk.content.strip() == "" or chunk.content == "N/A":
            logging.warning(f"Chunk with ID {chunk.id} has empty content")
            return False

        # Check that similarity score is valid
        if chunk.similarity_score < 0 or chunk.similarity_score > 1:
            logging.warning(f"Chunk with ID {chunk.id} has invalid similarity score: {chunk.similarity_score}")
            return False

        # Check that required metadata fields exist
        if not chunk.source_url or chunk.source_url == "N/A":
            logging.warning(f"Chunk with ID {chunk.id} has missing or invalid source URL")
            return False

        if not chunk.section_heading or chunk.section_heading == "N/A":
            logging.warning(f"Chunk with ID {chunk.id} has missing or invalid section heading")
            return False

    return True


def retrieve_content_with_validation(query: str, top_k: int = 5, threshold: float = 0.3) -> List[RetrievedContentChunk]:
    """
    Retrieve content chunks with validation and filtering based on similarity threshold.

    Args:
        query (str): The natural language query string
        top_k (int): Number of top results to retrieve (default: 5)
        threshold (float): Minimum similarity score threshold (default: 0.3)

    Returns:
        List[RetrievedContentChunk]: List of validated and filtered content chunks
    """
    # Retrieve content chunks
    chunks = retrieve_content_chunks(query, top_k)

    # Filter chunks based on similarity threshold
    filtered_chunks = [chunk for chunk in chunks if chunk.similarity_score >= threshold]

    # Validate the filtered chunks
    if not validate_retrieved_chunks(filtered_chunks):
        logging.warning("Some retrieved chunks failed validation")
        # Return only the valid chunks if any exist
        valid_chunks = []
        for chunk in filtered_chunks:
            if chunk.content and chunk.content.strip() != "" and chunk.content != "N/A":
                if chunk.source_url and chunk.source_url != "N/A":
                    if chunk.section_heading and chunk.section_heading != "N/A":
                        if 0 <= chunk.similarity_score <= 1:
                            valid_chunks.append(chunk)
        return valid_chunks

    return filtered_chunks


if __name__ == "__main__":
    # Example usage
    query = "What are Digital Twins in Robotics?"
    try:
        chunks = retrieve_content_with_validation(query)
        print(f"Retrieved {len(chunks)} chunks for query: '{query}'")
        for i, chunk in enumerate(chunks[:3]):  # Show first 3 chunks
            print(f"\nChunk {i+1}:")
            print(f"  Section: {chunk.section_heading}")
            print(f"  URL: {chunk.source_url}")
            print(f"  Similarity: {chunk.similarity_score:.3f}")
            print(f"  Content preview: {chunk.content[:100]}...")
    except Exception as e:
        print(f"Error retrieving content: {str(e)}")