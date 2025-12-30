"""
Vector Retrieval and Validation Tool

This script allows developers to validate their RAG retrieval pipeline by querying
a vector database with text queries, generating embeddings, and retrieving relevant
content chunks with metadata.
"""
import argparse
import logging
import os
import sys
import time
from typing import List, Dict, Any, Optional

import cohere
from dotenv import load_dotenv
from pydantic import BaseModel, Field
from qdrant_client import QdrantClient
from qdrant_client.http import models


# Load environment variables
load_dotenv()


class QdrantConfig(BaseModel):
    """Configuration for Qdrant vector database"""
    url: str = Field(default="", description="Qdrant Cloud URL")
    api_key: str = Field(default="", description="Qdrant API key")
    collection_name: str = Field(default="rag_embeddings", description="Name of the collection to use")
    vector_size: int = Field(default=1024, description="Size of embedding vectors (depends on model)")
    distance_metric: str = Field(default="Cosine", description="Distance metric for similarity search")

    def model_post_init(self, __context: Any) -> None:
        """Validate configuration after initialization"""
        if not self.url:
            raise ValueError('QDRANT_URL must be provided in environment variables')
        if not self.api_key:
            raise ValueError('QDRANT_API_KEY must be provided in environment variables')


class CohereConfig(BaseModel):
    """Configuration for Cohere embedding service"""
    api_key: str = Field(default="", description="Cohere API key")
    model_name: str = Field(default="embed-english-v3.0", description="Name of the Cohere embedding model")
    input_type: str = Field(default="search_query", description="Type of input for embedding")

    def model_post_init(self, __context: Any) -> None:
        """Validate configuration after initialization"""
        if not self.api_key:
            raise ValueError('COHERE_API_KEY must be provided in environment variables')


def setup_logging():
    """Configure logging for the retrieval script"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def load_config() -> tuple[QdrantConfig, CohereConfig]:
    """Load configuration from environment variables"""
    qdrant_config = QdrantConfig(
        url=os.getenv("QDRANT_URL", ""),
        api_key=os.getenv("QDRANT_API_KEY", ""),
        collection_name=os.getenv("QDRANT_COLLECTION_NAME", "rag_embeddings"),
        vector_size=int(os.getenv("QDRANT_VECTOR_SIZE", "1024")),
        distance_metric=os.getenv("QDRANT_DISTANCE_METRIC", "Cosine"),
    )

    cohere_config = CohereConfig(
        api_key=os.getenv("COHERE_API_KEY", ""),
        model_name=os.getenv("COHERE_MODEL_NAME", "embed-english-v3.0"),
        input_type=os.getenv("COHERE_INPUT_TYPE", "search_query"),
    )

    return qdrant_config, cohere_config


def create_qdrant_client(qdrant_config: QdrantConfig) -> QdrantClient:
    """Create Qdrant client with error handling"""
    try:
        client = QdrantClient(
            url=qdrant_config.url,
            api_key=qdrant_config.api_key,
            timeout=10.0
        )

        # Test connection
        client.get_collections()
        logging.info("Qdrant client initialized successfully")
        return client
    except Exception as e:
        logging.error(f"Failed to create Qdrant client: {str(e)}")
        raise


def create_cohere_client(cohere_config: CohereConfig) -> cohere.Client:
    """Create Cohere client with error handling"""
    try:
        client = cohere.Client(api_key=cohere_config.api_key)
        logging.info("Cohere client initialized successfully")
        return client
    except Exception as e:
        logging.error(f"Failed to create Cohere client: {str(e)}")
        raise


def generate_query_embedding(cohere_client: cohere.Client, query: str, cohere_config: CohereConfig) -> List[float]:
    """Generate embedding for the query using Cohere API"""
    try:
        response = cohere_client.embed(
            texts=[query],
            model=cohere_config.model_name,
            input_type=cohere_config.input_type
        )

        embedding = response.embeddings[0]

        # Validate embedding dimensions
        if len(embedding) != 1024:  # Cohere's embed-english-v3.0 should produce 1024-dim vectors
            raise ValueError(f"Expected embedding dimension 1024, got {len(embedding)}")

        logging.info(f"Query embedding generated with {len(embedding)} dimensions")
        return embedding
    except Exception as e:
        logging.error(f"Failed to generate query embedding: {str(e)}")
        raise


def perform_similarity_search(
    qdrant_client: QdrantClient,
    query_embedding: List[float],
    collection_name: str,
    top_k: int = 5
) -> List[Dict[str, Any]]:
    """Perform similarity search in Qdrant and return top-k results"""
    try:
        search_results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        # Format results
        formatted_results = []
        for result in search_results.points:
            # Get the payload from the result
            payload = result.payload

            # According to the storage.py and models.py, the content should be in associated_metadata
            associated_metadata = payload.get("associated_metadata", {})

            # Extract content from associated_metadata
            content = associated_metadata.get("content_text", "N/A")  # This is the field used in the Document model
            if content == "N/A":
                content = associated_metadata.get("content", "N/A")  # Fallback
            if content == "N/A":
                content = associated_metadata.get("text", "N/A")  # Another fallback
            if content == "N/A":
                content = payload.get("content", "N/A")  # Direct content in payload as last fallback

            # Extract URL and section from associated_metadata
            url = associated_metadata.get("source_url", "N/A")  # This is from Document model
            if url == "N/A":
                url = associated_metadata.get("url", "N/A")  # Fallback
            if url == "N/A":
                url = payload.get("source_url", "N/A")  # Direct in payload as fallback

            section = associated_metadata.get("section_heading", "N/A")  # This is from Document model
            if section == "N/A":
                section = associated_metadata.get("section", "N/A")  # Fallback
            if section == "N/A":
                section = associated_metadata.get("heading", "N/A")  # Another fallback
            if section == "N/A":
                section = payload.get("section", "N/A")  # Direct in payload as fallback

            formatted_result = {
                "id": result.id,
                "content": content,
                "metadata": {
                    "source_document_id": payload.get("source_document_id", "N/A"),
                    "associated_metadata": associated_metadata,
                    "created_at": payload.get("created_at", "N/A")
                },
                "similarity_score": result.score
            }

            # Add URL and section to associated_metadata if not already present
            if url != "N/A" and "url" not in formatted_result["metadata"]["associated_metadata"]:
                formatted_result["metadata"]["associated_metadata"]["url"] = url
            if section != "N/A" and "section" not in formatted_result["metadata"]["associated_metadata"]:
                formatted_result["metadata"]["associated_metadata"]["section"] = section

            formatted_results.append(formatted_result)

        logging.info(f"Retrieved {len(formatted_results)} results from Qdrant")
        return formatted_results
    except Exception as e:
        logging.error(f"Failed to perform similarity search: {str(e)}")
        raise


def validate_chunk_structure(chunk: Dict[str, Any]) -> bool:
    """Validate that retrieved chunk matches Spec-1 schema"""
    required_fields = ["id", "content", "metadata", "similarity_score"]
    metadata_required_fields = ["source_document_id", "associated_metadata", "created_at"]

    # Check top-level fields
    for field in required_fields:
        if field not in chunk:
            logging.warning(f"Missing field in chunk: {field}")
            return False

    # Check metadata fields
    metadata = chunk["metadata"]
    for field in metadata_required_fields:
        if field not in metadata:
            logging.warning(f"Missing metadata field in chunk: {field}")
            return False

    # Check if similarity score is valid
    if not isinstance(chunk["similarity_score"], (int, float)) or chunk["similarity_score"] < 0:
        logging.warning("Invalid similarity score")
        return False

    # For validation, we require either content or that associated_metadata has URL/section info
    has_content = chunk["content"] and chunk["content"] != "N/A"
    has_metadata_info = (
        metadata["associated_metadata"].get("url") != "N/A" or
        metadata["associated_metadata"].get("section") != "N/A"
    )

    if not has_content and not has_metadata_info:
        logging.warning("Chunk has neither content nor essential metadata (URL/section)")
        return False

    return True


def validate_retrieved_chunks(chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Validate retrieved chunks against Spec-1 schema and return validation report"""
    total_chunks = len(chunks)
    valid_chunks = 0
    invalid_chunks = []

    for i, chunk in enumerate(chunks):
        if validate_chunk_structure(chunk):
            valid_chunks += 1
        else:
            invalid_chunks.append(i)

    validation_report = {
        "total_chunks": total_chunks,
        "valid_chunks": valid_chunks,
        "invalid_chunks": invalid_chunks,
        "compliance_percentage": (valid_chunks / total_chunks * 100) if total_chunks > 0 else 0
    }

    return validation_report


def display_results(results: List[Dict[str, Any]], query: str):
    """Format and display retrieved chunks with metadata"""
    print(f"\nQuery: {query}")
    print(f"Found {len(results)} results:")
    print("-" * 80)

    for i, result in enumerate(results, 1):
        metadata = result["metadata"]["associated_metadata"]
        url = metadata.get("url", "N/A")
        section = metadata.get("section", "N/A")

        print(f"{i}. [{result['similarity_score']:.3f}] Section: {section} - URL: {url}")
        print(f"   Content: {result['content'][:200]}{'...' if len(result['content']) > 200 else ''}")
        print()


def main():
    """Main function to run the retrieval validation tool"""
    setup_logging()

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Vector Retrieval and Validation Tool")
    parser.add_argument("--query", type=str, required=True, help="Text query to search for")
    parser.add_argument("--top-k", type=int, default=5, help="Number of results to retrieve (default: 5)")
    parser.add_argument("--collection", type=str, default="rag_embeddings", help="Name of the Qdrant collection to query")

    args = parser.parse_args()

    try:
        # Load configuration
        qdrant_config, cohere_config = load_config()

        # Validate configuration
        try:
            qdrant_config.model_validate(qdrant_config.model_dump())
            cohere_config.model_validate(cohere_config.model_dump())
        except ValueError as e:
            logging.error(f"Configuration validation failed: {str(e)}")
            sys.exit(1)

        # Create clients
        qdrant_client = create_qdrant_client(qdrant_config)
        cohere_client = create_cohere_client(cohere_config)

        # Generate query embedding
        start_time = time.time()
        query_embedding = generate_query_embedding(cohere_client, args.query, cohere_config)
        embedding_time = time.time() - start_time

        # Perform similarity search
        start_time = time.time()
        results = perform_similarity_search(
            qdrant_client,
            query_embedding,
            args.collection,
            args.top_k
        )
        search_time = time.time() - start_time

        # Validate retrieved chunks
        validation_report = validate_retrieved_chunks(results)

        # Display results
        display_results(results, args.query)

        # Display validation report
        print("Validation Report:")
        print(f"  Total chunks: {validation_report['total_chunks']}")
        print(f"  Valid chunks: {validation_report['valid_chunks']}")
        print(f"  Compliance: {validation_report['compliance_percentage']:.1f}%")
        print(f"  Search time: {search_time:.3f}s")
        print(f"  Embedding time: {embedding_time:.3f}s")

        # Exit with appropriate code based on validation
        if validation_report['compliance_percentage'] < 100 and validation_report['total_chunks'] > 0:
            logging.warning("Some chunks did not meet validation criteria")
            sys.exit(2)  # Partial success
        elif validation_report['total_chunks'] == 0:
            logging.error("No results returned from search")
            sys.exit(1)  # Failure
        else:
            logging.info("All chunks validated successfully")
            sys.exit(0)  # Success

    except Exception as e:
        logging.error(f"Error during execution: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()