"""
RAG Agent Implementation using OpenRouter API with Mistral Model

This module implements a Retrieval-Augmented Generation (RAG) agent that can answer
questions based on retrieved content from the book materials, using OpenRouter API
with Mistral model, Cohere embeddings, and Qdrant vector database.
"""

import os
import logging
import asyncio
from typing import List, Dict, Any, Optional
from datetime import datetime
from openai import OpenAI
import time
import random

from config import get_agent_config
from models import UserQuery, RetrievedContentChunk, SourceCitation, AgentResponse
from retrieval_tool import retrieve_content_with_validation

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def run_with_retry(func, max_retries=3, base_delay=1.0, backoff_factor=2.0):
    """
    Run a function with retry logic for API quota and rate limit errors.

    Args:
        func: The function to run
        max_retries: Maximum number of retry attempts
        base_delay: Initial delay between retries in seconds
        backoff_factor: Multiplier for delay after each retry

    Returns:
        The result of the function call
    """
    for attempt in range(max_retries + 1):
        try:
            return func()
        except Exception as e:
            error_msg = str(e)
            is_quota_error = "insufficient_quota" in error_msg or "429" in error_msg
            is_rate_limit = "rate_limit" in error_msg.lower() or "too many requests" in error_msg.lower()

            if not (is_quota_error or is_rate_limit) and attempt < max_retries:
                logger.warning(f"Attempt {attempt + 1} failed with non-retryable error: {error_msg}")
                raise e
            elif attempt < max_retries:
                # Calculate delay with exponential backoff and jitter
                delay = base_delay * (backoff_factor ** attempt)
                jitter = random.uniform(0, delay * 0.1)  # Add up to 10% jitter
                total_delay = delay + jitter

                logger.warning(f"Attempt {attempt + 1} failed with {'quota' if is_quota_error else 'rate limit'} error: {error_msg}. "
                              f"Retrying in {total_delay:.2f} seconds...")
                time.sleep(total_delay)
            else:
                # All retries exhausted
                logger.error(f"All {max_retries + 1} attempts failed with {'quota' if is_quota_error else 'rate limit'} error: {error_msg}")
                raise e


def retrieve_content_tool(query: str, config) -> List[Dict[str, Any]]:
    """
    Standalone function to retrieve content from the vector database.

    Args:
        query (str): The natural language query
        config: The agent configuration

    Returns:
        List[Dict[str, Any]]: List of retrieved content chunks with metadata
    """
    try:
        chunks = retrieve_content_with_validation(
            query=query,
            top_k=config.retrieval_top_k,
            threshold=config.retrieval_threshold
        )

        # Convert to the format expected by the agent
        result = []
        for chunk in chunks:
            result.append({
                "id": chunk.id,
                "content": chunk.content,
                "source_url": chunk.source_url,
                "section_heading": chunk.section_heading,
                "similarity_score": chunk.similarity_score,
                "chunk_index": chunk.chunk_index
            })

        logger.info(f"Retrieved {len(result)} content chunks for query: {query[:50]}...")
        return result

    except Exception as e:
        logger.error(f"Error in retrieve_content_tool: {str(e)}")
        return []


class MistralRAGAgent:
    """
    RAG (Retrieval-Augmented Generation) Agent that uses OpenRouter API
    with Mistral model to answer questions based on book content.
    """

    def __init__(self):
        """Initialize the Mistral RAG Agent with configuration and clients"""
        self.config = get_agent_config()

        # Initialize OpenAI client with OpenRouter API key
        self.openrouter_client = OpenAI(
            api_key=self.config.openrouter_api_key,
            base_url="https://openrouter.ai/api/v1"
        )

        # Initialize Cohere client for embeddings
        import cohere
        self.cohere_client = cohere.Client(api_key=self.config.cohere_api_key)

        # Initialize Qdrant client for vector search
        from qdrant_client import QdrantClient
        self.qdrant_client = QdrantClient(
            url=self.config.qdrant_url,
            api_key=self.config.qdrant_api_key,
            timeout=10.0
        )

        logger.info("Mistral RAG Agent initialized successfully")

    def query(self, user_query: str, session_id: Optional[str] = None) -> AgentResponse:
        """
        Process a user query and return a response with citations using OpenRouter API.

        Args:
            user_query (str): The natural language question from the user
            session_id (Optional[str]): Session identifier for conversation context

        Returns:
            AgentResponse: The agent's response with citations and metadata
        """
        start_time = time.time()

        try:
            # Validate input query
            if not user_query or not user_query.strip():
                raise ValueError("Query cannot be empty or None")

            # Create a user query object
            user_query_obj = UserQuery(
                query_text=user_query,
                session_id=session_id
            )

            # Always retrieve chunks first to ensure we have context for the response
            retrieved_chunks = []
            try:
                # Use the function that's available in the module
                retrieved_chunks_from_tool = retrieve_content_with_validation(
                    query=user_query,
                    top_k=self.config.retrieval_top_k,
                    threshold=self.config.retrieval_threshold
                )

                # Convert RetrievedContentChunk objects to the format expected by the agent
                retrieved_chunks = [{
                    "id": chunk.id,
                    "content": chunk.content,
                    "source_url": chunk.source_url,
                    "section_heading": chunk.section_heading,
                    "similarity_score": chunk.similarity_score,
                    "chunk_index": chunk.chunk_index
                } for chunk in retrieved_chunks_from_tool]
            except Exception as e:
                logger.error(f"Error retrieving content chunks: {str(e)}")
                retrieved_chunks = []

            # Create context from retrieved chunks
            context_text = ""
            if retrieved_chunks:
                context_text = "\n\n".join([
                    f"Section: {chunk['section_heading']}\nContent: {chunk['content'][:1000]}"
                    for chunk in retrieved_chunks  # Limit content length to avoid token limits
                ])

            # Create the prompt for the Mistral model
            if context_text:
                prompt = f"""Based on the following context, please answer the question. If the context doesn't contain the answer, please say so.

Context:
{context_text}

Question: {user_query}

Please provide a helpful, accurate response based on the context, and cite the relevant sections."""
            else:
                prompt = f"Please answer the following question: {user_query}"

            # Try to generate content using OpenRouter API
            def generate_response():
                response = self.openrouter_client.chat.completions.create(
                    model="mistralai/devstral-2512:free",
                    messages=[
                        {"role": "system", "content": "You are a helpful AI assistant that answers questions based on retrieved content from book materials. Always ground your responses in the provided context and cite the sources."},
                        {"role": "user", "content": prompt}
                    ],
                    max_tokens=1000,
                    temperature=0.7
                )
                return response.choices[0].message.content

            try:
                response_text = run_with_retry(generate_response)
            except Exception as e:
                error_msg = str(e)
                logger.error(f"Error calling OpenRouter API: {error_msg}")

                if "insufficient_quota" in error_msg or "429" in error_msg:
                    logger.error(f"OpenRouter API quota exceeded after retries: {error_msg}")
                    # Generate a response based on retrieved content when API is unavailable
                    if retrieved_chunks:
                        content_preview = " ".join([chunk['content'][:100] for chunk in retrieved_chunks[:2]])
                        response_text = (
                            f"Based on the available content: {content_preview}... "
                            f"I found {len(retrieved_chunks)} relevant sections, but I couldn't generate a detailed response due to API quota limits. "
                            "Please check your OpenRouter account usage and billing details."
                        )
                    else:
                        response_text = (
                            "I'm currently unable to process your request due to API quota limits. "
                            "Please check your OpenRouter account usage and billing details. "
                            f"Original query: '{user_query}'"
                        )
                elif "rate_limit" in error_msg.lower() or "too many requests" in error_msg.lower():
                    logger.error(f"Rate limit exceeded after retries: {error_msg}")
                    # Generate a response based on retrieved content when API is unavailable
                    if retrieved_chunks:
                        content_preview = " ".join([chunk['content'][:100] for chunk in retrieved_chunks[:2]])
                        response_text = (
                            f"Based on the available content: {content_preview}... "
                            f"I found {len(retrieved_chunks)} relevant sections, but I'm experiencing high demand. "
                            "Please try again later."
                        )
                    else:
                        response_text = (
                            "I'm currently experiencing high demand and unable to process your request. "
                            "Please try again later. Original query: '{user_query}'"
                        )
                else:
                    logger.error(f"Error calling OpenRouter API after retries: {error_msg}")
                    # Even if the API fails, try to provide information based on retrieved content
                    if retrieved_chunks:
                        content_preview = " ".join([chunk['content'][:100] for chunk in retrieved_chunks[:2]])
                        response_text = (
                            f"Based on the available content: {content_preview}... "
                            "An error occurred while generating the response, but relevant content was found."
                        )
                    else:
                        raise

            # If no content is found, return appropriate response
            if not retrieved_chunks:
                response_text = (
                    "I couldn't find any relevant content in the book materials for your query: "
                    f"'{user_query}'. The information you're looking for may not be covered in the book."
                )

                agent_response = AgentResponse(
                    response_text=response_text,
                    retrieved_chunks=[],
                    citations=[],
                    query_id=user_query_obj.id if hasattr(user_query_obj, 'id') else None
                )

                logger.info(f"No content found for query: {user_query[:50]}...")
                return agent_response

            # Create citations
            citations = self._extract_citations(retrieved_chunks)

            # Validate citations
            if not self._validate_citations(citations):
                logger.warning("Some citations failed validation")

            # Rank citations by relevance
            ranked_citations = self._rank_citations_by_relevance(citations)

            # Create RetrievedContentChunk objects from the retrieved data
            retrieved_content_objects = []
            for chunk in retrieved_chunks:
                retrieved_chunk = RetrievedContentChunk(
                    id=chunk['id'],
                    content=chunk['content'],
                    similarity_score=chunk['similarity_score'],
                    metadata={},
                    source_url=chunk['source_url'],
                    section_heading=chunk['section_heading'],
                    chunk_index=chunk.get('chunk_index', 0)
                )
                retrieved_content_objects.append(retrieved_chunk)

            # Create the agent response
            agent_response = AgentResponse(
                response_text=response_text,
                retrieved_chunks=retrieved_content_objects,
                citations=ranked_citations,
                query_id=user_query_obj.id if hasattr(user_query_obj, 'id') else None
            )

            processing_time = time.time() - start_time
            logger.info(f"Generated response for query: {user_query[:50]}... (Processing time: {processing_time:.2f}s)")
            return agent_response

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            # Return a user-friendly error response instead of raising
            error_response = AgentResponse(
                response_text=f"An error occurred while processing your query: {str(e)}. Please try again later.",
                retrieved_chunks=[],
                citations=[],
                query_id=None
            )
            return error_response

    def _extract_citations(self, retrieved_chunks: List[Dict[str, Any]]) -> List[SourceCitation]:
        """
        Extract citations from retrieved content chunks.

        Args:
            retrieved_chunks (List[Dict[str, Any]]): Retrieved content chunks

        Returns:
            List[SourceCitation]: List of source citations
        """
        citations = []
        for chunk in retrieved_chunks:
            citation = SourceCitation(
                url=chunk['source_url'],
                section=chunk['section_heading'],
                content_preview=chunk['content'][:200] + "..." if len(chunk['content']) > 200 else chunk['content'],
                relevance_score=chunk['similarity_score']
            )
            citations.append(citation)

        return citations

    def _rank_citations_by_relevance(self, citations: List[SourceCitation]) -> List[SourceCitation]:
        """
        Rank citations by relevance score in descending order.

        Args:
            citations (List[SourceCitation]): List of citations to rank

        Returns:
            List[SourceCitation]: Ranked list of citations
        """
        return sorted(citations, key=lambda x: x.relevance_score, reverse=True)

    def _validate_citations(self, citations: List[SourceCitation]) -> bool:
        """
        Validate that all required citation fields are present.

        Args:
            citations (List[SourceCitation]): List of citations to validate

        Returns:
            bool: True if all citations are valid, False otherwise
        """
        for citation in citations:
            if not citation.url or citation.url == "N/A":
                return False
            if not citation.section or citation.section == "N/A":
                return False
            if not citation.content_preview or citation.content_preview == "N/A":
                return False
            if citation.relevance_score < 0 or citation.relevance_score > 1:
                return False
        return True

    def query_simple(self, user_query: str) -> str:
        """
        Simple query method that returns just the response text.

        Args:
            user_query (str): The natural language question from the user

        Returns:
            str: The agent's response text
        """
        response = self.query(user_query)
        return response.response_text


def create_mistral_rag_agent() -> MistralRAGAgent:
    """
    Factory function to create and return a Mistral RAG Agent instance.

    Returns:
        MistralRAGAgent: Initialized Mistral RAG Agent instance
    """
    return MistralRAGAgent()


# Example usage and testing
if __name__ == "__main__":
    # Create the Mistral RAG agent
    try:
        agent = create_mistral_rag_agent()
    except Exception as e:
        print(f"Error creating Mistral RAG agent: {str(e)}")
        exit(1)

    # Test query
    test_query = "What is AI?"

    print(f"Query: {test_query}")
    print("-" * 50)

    try:
        response = agent.query(test_query)
        print(f"Response: {response.response_text}")

        if response.citations:
            print(f"\nCitations ({len(response.citations)}):")
            for i, citation in enumerate(response.citations, 1):
                print(f"  {i}. {citation.section} - {citation.url}")
        else:
            print("\nNo citations available for this response.")
    except Exception as e:
        print(f"Error: {str(e)}")
        logger.error(f"Unexpected error during query processing: {str(e)}")