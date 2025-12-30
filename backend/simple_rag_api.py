"""
Simple RAG API that bypasses the OpenAI Agents SDK and uses Google Gemini API
This should resolve the timeout and quota issues while still providing RAG functionality
"""
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any
import logging
from datetime import datetime
import os
from config import get_agent_config
from models import APIQueryRequest, APIQueryResponse, APIHealthResponse, APIQueryResponseData, SourceCitation
from retrieval_tool import retrieve_content_with_validation
import cohere
from qdrant_client import QdrantClient
import time

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app instance
app = FastAPI(
    title="Simple RAG API",
    description="Simplified RAG API using Google Gemini to avoid quota issues",
    version="1.0.0"
)

# Add CORS middleware for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class SimpleRAGAgent:
    """
    Simplified RAG Agent that uses Google Gemini API instead of OpenAI
    This avoids quota issues and timeout problems
    """
    def __init__(self):
        self.config = get_agent_config()

        # Initialize Cohere client for embeddings
        self.cohere_client = cohere.Client(api_key=self.config.cohere_api_key)

        # Initialize Qdrant client for vector search
        self.qdrant_client = QdrantClient(
            url=self.config.qdrant_url,
            api_key=self.config.qdrant_api_key,
            timeout=10.0
        )

        # Initialize Google Gemini client
        try:
            import google.generativeai as genai
            gemini_api_key = os.getenv("GEMINI_API_KEY", "")
            if not gemini_api_key:
                raise ValueError("GEMINI_API_KEY environment variable is required")
            genai.configure(api_key=gemini_api_key)
            self.gemini_model = genai.GenerativeModel('gemini-pro')  # or gemini-1.5-pro for newer model
            logger.info("Gemini API configured successfully")
        except ImportError:
            raise ImportError("Please install google-generativeai: pip install google-generativeai")

        logger.info("Simple RAG Agent with Gemini initialized successfully")

    def query(self, user_query: str, context_type: str = "full_book", selected_text: str = None) -> Dict[str, Any]:
        """
        Process a user query using retrieved content and Google Gemini API call
        """
        start_time = time.time()

        try:
            # Validate input query
            if not user_query or not user_query.strip():
                raise ValueError("Query cannot be empty or None")

            # Retrieve relevant content from vector database
            retrieved_chunks = retrieve_content_with_validation(
                query=user_query,
                top_k=self.config.retrieval_top_k,
                threshold=self.config.retrieval_threshold
            )

            logger.info(f"Retrieved {len(retrieved_chunks)} content chunks for query: {user_query[:50]}...")

            # If no content is found, return appropriate response
            if not retrieved_chunks:
                response_text = (
                    "I couldn't find any relevant content in the book materials for your query: "
                    f"'{user_query}'. The information you're looking for may not be covered in the book."
                )

                return {
                    "response_text": response_text,
                    "sources": [],
                    "query_id": f"query_{int(time.time())}"
                }

            # Build context from retrieved content
            context_parts = []
            sources = []

            for chunk in retrieved_chunks:
                context_parts.append(f"Source: {chunk.source_url}\nSection: {chunk.section_heading}\nContent: {chunk.content}\n")

                # Create source citation
                source = SourceCitation(
                    url=chunk.source_url,
                    section=chunk.section_heading,
                    content_preview=chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,
                    relevance_score=chunk.similarity_score
                )
                sources.append(source)

            context = "\n".join(context_parts)

            # Prepare the prompt for Gemini
            if context_type == "selected_text" and selected_text:
                # Combine the selected text with retrieved content
                full_context = f"Selected Text: {selected_text}\n\nRetrieved Content:\n{context}"
                prompt = f"Based on the following context, please answer the question:\n\nContext: {full_context}\n\nQuestion: {user_query}\n\nAnswer and provide source citations if possible:"
            else:
                # Use only retrieved content
                prompt = f"Based on the following context from the book, please answer the question:\n\nContext: {context}\n\nQuestion: {user_query}\n\nAnswer and provide source citations if possible:"

            # Make a call to Google Gemini
            response = self.gemini_model.generate_content(
                prompt,
                generation_config={
                    "temperature": 0.3,
                    "max_output_tokens": 500,
                }
            )

            response_text = response.text

            processing_time = time.time() - start_time
            logger.info(f"Generated response for query: {user_query[:50]}... (Processing time: {processing_time:.2f}s)")

            return {
                "response_text": response_text,
                "sources": sources,
                "query_id": f"query_{int(time.time())}"
            }

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            # Return a user-friendly error response
            return {
                "response_text": f"An error occurred while processing your query: {str(e)}. Please try again later.",
                "sources": [],
                "query_id": f"query_{int(time.time())}"
            }

# Global instance of the simple RAG agent
simple_rag_agent = None

def get_simple_rag_agent():
    global simple_rag_agent
    if simple_rag_agent is None:
        simple_rag_agent = SimpleRAGAgent()
    return simple_rag_agent

@app.get("/")
async def root():
    """
    Root endpoint for basic health check
    """
    return {"message": "Simple RAG API is running"}

@app.get("/api/health", response_model=APIHealthResponse)
async def health_check():
    """
    Health check endpoint
    """
    return APIHealthResponse(
        status="healthy",
        timestamp=datetime.now().isoformat()
    )

@app.post("/api/query", response_model=APIQueryResponse)
async def query_endpoint(request: APIQueryRequest):
    """
    Query endpoint that processes user queries against book content using simplified RAG
    """
    import time
    start_time = time.time()

    try:
        # Validate the request data
        if not request.query or len(request.query.strip()) == 0:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if request.context_type not in ["full_book", "selected_text"]:
            raise HTTPException(status_code=400, detail="context_type must be either 'full_book' or 'selected_text'")

        if request.context_type == "selected_text" and (not request.selected_text or len(request.selected_text.strip()) == 0):
            raise HTTPException(status_code=400, detail="selected_text is required when context_type is 'selected_text'")

        # Additional validation: Check query length
        if len(request.query) > 2000:  # As specified in the data model
            raise HTTPException(status_code=400, detail="Query exceeds maximum length of 2000 characters")

        if request.selected_text and len(request.selected_text) > 5000:  # Reasonable limit for selected text
            raise HTTPException(status_code=400, detail="Selected text exceeds maximum length of 5000 characters")

        # Validate query contains meaningful content (not just special characters or whitespace)
        import re
        query_clean = re.sub(r'[^\w\s]', '', request.query.strip())
        if not query_clean or len(query_clean) < 2:
            raise HTTPException(status_code=400, detail="Query must contain meaningful content")

        # Log the incoming request with more detail
        logger.info(f"Processing query: {request.query[:50]}... with context_type: {request.context_type}, query_length: {len(request.query)}")

        # Get the simple RAG agent instance
        rag_agent = get_simple_rag_agent()

        # Process the query using the simplified RAG approach
        result = rag_agent.query(
            user_query=request.query,
            context_type=request.context_type,
            selected_text=request.selected_text
        )

        # Create the API response
        api_response_data = APIQueryResponseData(
            answer=result["response_text"],
            sources=result["sources"],
            query_id=result["query_id"]
        )

        processing_time = time.time() - start_time
        logger.info(f"Query processed successfully in {processing_time:.2f}s")

        return APIQueryResponse(
            success=True,
            data=api_response_data
        )

    except HTTPException:
        raise
    except Exception as e:
        # Calculate processing time for error case
        processing_time = time.time() - start_time
        logger.error(f"Error processing query: {str(e)} (took {processing_time:.2f}s)")
        return APIQueryResponse(
            success=False,
            error=f"Error processing query: {str(e)}"
        )

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)