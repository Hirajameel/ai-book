"""
Storage module for RAG Ingestion Pipeline
Handles vector storage using Qdrant
"""
import logging
from typing import List
from models import Embedding


def create_qdrant_client(qdrant_config):
    """Create Qdrant client with error handling"""
    try:
        from qdrant_client import QdrantClient

        client = QdrantClient(
            url=qdrant_config.url,
            api_key=qdrant_config.api_key,
            timeout=30.0  # Increased timeout to 30 seconds
        )

        # Test connection
        client.get_collections()
        logging.info("Qdrant client initialized successfully")
        return client
    except Exception as e:
        logging.error(f"Failed to create Qdrant client: {str(e)}")
        raise


def create_qdrant_collection(qdrant_client, collection_name: str, vector_size: int = 1024, distance_metric: str = "Cosine"):
    """Create Qdrant collection with proper schema and vector size for embeddings"""
    try:
        from qdrant_client.http import models

        # Check if collection already exists
        collections = qdrant_client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)

        if not collection_exists:
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance[distance_metric.upper()]
                )
            )
            logging.info(f"Created Qdrant collection: {collection_name}")
        else:
            logging.info(f"Qdrant collection already exists: {collection_name}")
    except Exception as e:
        logging.error(f"Error creating Qdrant collection: {str(e)}")
        raise


def store_embeddings_in_qdrant(qdrant_client, embeddings: List[Embedding], collection_name: str):
    """Implement function to store embeddings in Qdrant with metadata payload"""
    try:
        from qdrant_client.http import models
        import hashlib

        # Prepare points for upload
        points = []
        for embedding in embeddings:
            # Extract content from associated_metadata if it exists
            associated_metadata = embedding.associated_metadata or {}
            content_text = associated_metadata.get('content_text', '')
            if not content_text:
                content_text = associated_metadata.get('content', '')
            if not content_text:
                content_text = associated_metadata.get('text', '')

            point = models.PointStruct(
                id=abs(hash(embedding.id)) % (10**9),  # Qdrant ID should be int, using abs to ensure positive
                vector=embedding.vector,
                payload={
                    "id": embedding.id,
                    "source_document_id": embedding.source_document_id,
                    "content": content_text,  # Store content directly in payload for easy retrieval
                    "associated_metadata": associated_metadata,
                    "created_at": embedding.created_at.isoformat() if embedding.created_at else None
                }
            )
            points.append(point)

        # Upload points to Qdrant
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )

        logging.info(f"Successfully stored {len(points)} embeddings in Qdrant collection: {collection_name}")
        return True
    except Exception as e:
        logging.error(f"Error storing embeddings in Qdrant: {str(e)}")
        return False


def handle_qdrant_storage_errors(qdrant_client, embeddings: List[Embedding], collection_name: str):
    """Add error handling for Qdrant storage operations"""
    try:
        return store_embeddings_in_qdrant(qdrant_client, embeddings, collection_name)
    except Exception as e:
        logging.error(f"Qdrant storage error: {str(e)}")
        # Implement retry logic or other error handling here
        return False


def verify_storage_success(qdrant_client, collection_name: str):
    """Implement function to verify successful storage and accessibility of embeddings"""
    try:
        # Get the count of points in the collection
        count_result = qdrant_client.count(
            collection_name=collection_name
        )

        logging.info(f"Verification: Collection {collection_name} contains {count_result.count} vectors")
        return count_result.count > 0
    except Exception as e:
        logging.error(f"Error verifying storage: {str(e)}")
        return False