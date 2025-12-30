"""
Embedder module for RAG Ingestion Pipeline
Handles embedding generation using Cohere API
"""
import logging
import time
import random
from typing import List
from models import Embedding


def create_cohere_client(embedding_config):
    """Create Cohere client with error handling"""
    try:
        import cohere

        client = cohere.Client(
            embedding_config.api_key
        )

        # Test connection by making a simple request
        # client.embed(texts=["test"], model=embedding_config.model_name)
        logging.info("Cohere client initialized successfully")
        return client
    except Exception as e:
        logging.error(f"Failed to create Cohere client: {str(e)}")
        raise


def generate_embeddings_with_cohere(texts: List[str], cohere_client, model_name: str = "embed-english-v3.0", input_type: str = "search_document") -> List[List[float]]:
    """Implement Cohere embedding generation function with batch processing"""
    try:
        response = cohere_client.embed(
            texts=texts,
            model=model_name,
            input_type=input_type
        )
        return response.embeddings
    except Exception as e:
        logging.error(f"Error generating embeddings: {str(e)}")
        raise


def handle_rate_limiting_with_backoff(func, *args, max_retries: int = 5, base_delay: float = 1.0, **kwargs):
    """Add rate limit handling with exponential backoff for Cohere API"""
    for attempt in range(max_retries):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            if "rate limit" in str(e).lower() or "429" in str(e):
                delay = base_delay * (2 ** attempt) + random.uniform(0, 1)  # Add jitter
                logging.warning(f"Rate limit hit, retrying in {delay:.2f}s (attempt {attempt + 1}/{max_retries})")
                time.sleep(delay)
            else:
                raise e
    raise Exception(f"Failed after {max_retries} attempts due to rate limiting")


def validate_embedding_dimensions(embedding: List[float], expected_size: int = 1024) -> bool:
    """Implement embedding validation to ensure proper vector dimensions"""
    if not isinstance(embedding, list):
        return False
    if len(embedding) != expected_size:
        return False
    # Check that all elements are numeric
    for item in embedding:
        if not isinstance(item, (int, float)):
            return False
    return True


def handle_embedding_api_errors(texts: List[str], cohere_client, model_name: str = "embed-english-v3.0", input_type: str = "search_document") -> List[List[float]]:
    """Create function to handle embedding API errors gracefully"""
    try:
        # Process in batches if there are too many texts
        batch_size = 96  # Cohere's recommended batch size
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            # Use rate limiting with backoff
            embeddings = handle_rate_limiting_with_backoff(
                generate_embeddings_with_cohere,
                batch,
                cohere_client,
                model_name,
                input_type
            )
            all_embeddings.extend(embeddings)

        return all_embeddings
    except Exception as e:
        logging.error(f"Embedding API error: {str(e)}")
        # Return empty embeddings in case of error, but log the issue
        return [[] for _ in texts]


def create_embeddings_from_documents(documents, cohere_client, embedding_config, qdrant_config=None):
    """Create embeddings from a list of documents"""
    embeddings = []

    # Extract texts for embedding generation
    texts_to_embed = [doc.content_text for doc in documents if doc.content_text.strip()]
    logging.info(f"Number of documents to embed: {len(documents)}")
    logging.info(f"Number of texts to embed: {len(texts_to_embed)}")

    if texts_to_embed:
        try:
            # Generate embeddings using Cohere
            logging.info("Calling Cohere API for embedding generation...")
            embedding_vectors = handle_embedding_api_errors(
                texts_to_embed,
                cohere_client,
                embedding_config.model_name,
                embedding_config.input_type
            )
            logging.info(f"Received {len(embedding_vectors)} embedding vectors from Cohere API")

            # Use qdrant_config.vector_size if available, otherwise default
            expected_vector_size = qdrant_config.vector_size if qdrant_config else 1024

            # Create embedding objects with metadata
            for idx, (doc, vector) in enumerate(zip(documents, embedding_vectors)):
                # Validate embedding dimensions
                if validate_embedding_dimensions(vector, expected_vector_size):
                    embedding_obj = Embedding(
                        id=f"emb_{hash(doc.id)}",
                        vector=vector,
                        associated_metadata={
                            'source_url': doc.source_url,
                            'section_heading': doc.section_heading,
                            'content_text': doc.content_text,  # Include content in metadata
                            'chunk_index': doc.chunk_index,
                            'original_doc_id': doc.id,
                            'created_at': doc.created_at.isoformat() if hasattr(doc, 'created_at') else None
                        },
                        source_document_id=doc.id
                    )
                    embeddings.append(embedding_obj)
                else:
                    logging.warning(f"Invalid embedding dimensions for document {doc.id}")

        except Exception as e:
            logging.error(f"Failed to generate embeddings: {str(e)}", exc_info=True)  # Include full traceback
            raise

    return embeddings