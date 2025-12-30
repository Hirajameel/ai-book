"""
FastAPI backend for RAG agent integration

This module implements a FastAPI server that exposes endpoints for the RAG agent
to process user queries against book content.
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any
import logging
import asyncio
import concurrent.futures
import threading

# Import the models we created
from models import APIQueryRequest, APIQueryResponse, APIHealthResponse, APIQueryResponseData

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app instance
app = FastAPI(
    title="RAG Agent API",
    description="API for querying book content using RAG agent",
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

@app.get("/")
async def root():
    """
    Root endpoint for basic health check
    """
    return {"message": "RAG Agent API is running"}

@app.get("/api/health", response_model=APIHealthResponse)
async def health_check():
    """
    Health check endpoint
    """
    from datetime import datetime
    return APIHealthResponse(
        status="healthy",
        timestamp=datetime.now().isoformat()
    )

@app.post("/api/query", response_model=APIQueryResponse)
async def query_endpoint(request: APIQueryRequest):
    """
    Query endpoint that processes user queries against book content
    """
    import time
    start_time = time.time()

    try:
        # Validate the request data using the Pydantic model
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

        # Import the RAG agent, models, and utility functions
        from mistral_rag_agent import create_mistral_rag_agent, AgentResponse
        from models import SourceCitation
        from utils import generate_query_id, validate_query_text, validate_context_type, validate_selected_text

        # Create the RAG agent instance
        try:
            rag_agent = create_mistral_rag_agent()
        except Exception as e:
            logger.warning(f"RAG agent initialization failed (this is expected if ingestion hasn't been run): {str(e)}")
            # If the agent fails to initialize, return a more helpful message
            # For now, return a mock response but with a success flag to indicate the API is working
            mock_sources = [
                SourceCitation(
                    url="https://example.com/setup-guide",
                    section="Setup Guide",
                    content_preview="To use the RAG agent, please run the ingestion pipeline first to populate the vector database.",
                    relevance_score=0.9
                )
            ]

            api_response_data = APIQueryResponseData(
                answer="The RAG agent is not yet configured. To use this feature, please run the ingestion pipeline first to populate the vector database with book content. The ingestion pipeline can be run with: python main.py",
                sources=mock_sources,
                query_id=generate_query_id()
            )

            processing_time = time.time() - start_time
            logger.info(f"Query processed with agent not configured in {processing_time:.2f}s")
            return APIQueryResponse(
                success=True,  # Return success=True but with setup instructions
                data=api_response_data
            )

        # Handle different context types
        # Since we're getting an event loop issue, we'll use a workaround
        # to handle the agent properly in the async context

        def run_agent_in_thread():
            """Run the agent query in a separate thread to avoid event loop conflicts"""
            try:
                if request.context_type == "full_book":
                    return rag_agent.query(request.query)
                elif request.context_type == "selected_text" and request.selected_text:
                    combined_query = f"Based on this text: '{request.selected_text}', answer this: {request.query}"
                    return rag_agent.query(combined_query)
                else:
                    return None
            except Exception as e:
                logger.error(f"Error in agent thread: {str(e)}")
                raise e

        # Run the agent in a separate thread to avoid event loop conflicts
        with concurrent.futures.ThreadPoolExecutor() as executor:
            try:
                future = executor.submit(run_agent_in_thread)
                agent_response = future.result(timeout=30)  # 30 second timeout

                if agent_response is None:
                    # Handle the else case
                    processing_time = time.time() - start_time
                    logger.info(f"Query processed with error in {processing_time:.2f}s")
                    return APIQueryResponse(
                        success=False,
                        error="Invalid context type or missing selected text for selected_text context."
                    )

                # Convert agent response to API response format
                api_sources = []
                if hasattr(agent_response, 'citations') and agent_response.citations:
                    api_sources = [
                        SourceCitation(
                            url=citation.url if hasattr(citation, 'url') else "N/A",
                            section=citation.section if hasattr(citation, 'section') else "N/A",
                            content_preview=citation.content_preview if hasattr(citation, 'content_preview') else citation.section if hasattr(citation, 'section') else "N/A",
                            relevance_score=citation.relevance_score if hasattr(citation, 'relevance_score') else 0.0
                        )
                        for citation in agent_response.citations
                    ]

                # Create the API response data
                api_response_data = APIQueryResponseData(
                    answer=agent_response.response_text if hasattr(agent_response, 'response_text') else str(agent_response),
                    sources=api_sources,
                    query_id=generate_query_id()
                )

                # Calculate processing time for success case before returning
                processing_time = time.time() - start_time
                logger.info(f"Query processed successfully in {processing_time:.2f}s")
                return APIQueryResponse(
                    success=True,
                    data=api_response_data
                )
            except concurrent.futures.TimeoutError:
                logger.error("Agent query timed out")
                error_response_data = APIQueryResponseData(
                    answer="Query timed out. The agent is taking too long to respond.",
                    sources=[],
                    query_id=generate_query_id()
                )
                return APIQueryResponse(
                    success=False,
                    data=error_response_data,
                    error="Query timed out"
                )
            except Exception as e:
                logger.error(f"Error calling RAG agent: {str(e)}")
                # Check if it's the event loop issue and provide more helpful message
                error_msg = str(e)
                if "cannot be called when an event loop is already running" in error_msg:
                    # If the threading approach also fails, return a more specific message
                    error_response_data = APIQueryResponseData(
                        answer="RAG agent is not properly configured or API keys are invalid. Please check your configuration and ensure the ingestion pipeline has been run.",
                        sources=[],
                        query_id=generate_query_id()
                    )
                    return APIQueryResponse(
                        success=False,
                        data=error_response_data,
                        error="Agent configuration issue"
                    )
                else:
                    error_response_data = APIQueryResponseData(
                        answer=f"Error processing query: {error_msg}",
                        sources=[],
                        query_id=generate_query_id()
                    )
                    return APIQueryResponse(
                        success=False,
                        data=error_response_data,
                        error=str(e)
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