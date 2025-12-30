"""
Data models for RAG Ingestion Pipeline and RAG Agent System
Defines Document, Embedding, and Agent-specific data structures
"""
from pydantic import BaseModel, Field, field_validator
from datetime import datetime
from typing import List, Dict, Any, Optional
import hashlib


class DocumentStatus:
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


class Document(BaseModel):
    """Represents a single crawled document/page from a Docusaurus site"""
    id: str = Field(default="", description="Unique identifier for the document")
    source_url: str = Field(default="", description="Original URL of the document")
    content_text: str = Field(default="", description="Clean text extracted from the HTML")
    section_heading: str = Field(default="", description="Main section/heading context of the content")
    chunk_index: int = Field(default=0, description="Sequential index for document chunks")
    processing_status: str = Field(default=DocumentStatus.PENDING, description="Status of processing")
    created_at: datetime = Field(default_factory=datetime.now, description="Timestamp of document creation")
    updated_at: datetime = Field(default_factory=datetime.now, description="Timestamp of last update")

    @field_validator('source_url')
    @classmethod
    def validate_source_url(cls, v):
        if not v:
            raise ValueError('source_url must not be empty')
        return v

    @field_validator('chunk_index')
    @classmethod
    def validate_chunk_index(cls, v):
        if v < 0:
            raise ValueError('chunk_index must be non-negative')
        return v

    @field_validator('processing_status')
    @classmethod
    def validate_processing_status(cls, v):
        valid_statuses = [DocumentStatus.PENDING, DocumentStatus.IN_PROGRESS, DocumentStatus.COMPLETED, DocumentStatus.FAILED]
        if v not in valid_statuses:
            raise ValueError(f'processing_status must be one of {valid_statuses}')
        return v


class Embedding(BaseModel):
    """Represents an embedding vector generated from document content"""
    id: str = Field(default="", description="Unique identifier for the embedding")
    vector: List[float] = Field(default_factory=list, description="The embedding vector representation")
    associated_metadata: Dict[str, Any] = Field(default_factory=dict, description="Metadata associated with the embedding")
    source_document_id: str = Field(default="", description="Reference to the source document")
    created_at: datetime = Field(default_factory=datetime.now, description="Timestamp of embedding creation")

    @field_validator('vector')
    @classmethod
    def validate_vector(cls, v):
        if not isinstance(v, list):
            raise ValueError('vector must be a list of floats')
        if len(v) == 0:
            raise ValueError('vector must not be empty')
        # Check that all elements are floats/numbers
        for item in v:
            if not isinstance(item, (int, float)):
                raise ValueError('vector must contain only numeric values')
        return v

    @field_validator('source_document_id')
    @classmethod
    def validate_source_document_id(cls, v):
        if not v:
            raise ValueError('source_document_id must be provided')
        return v


class VectorRecord(BaseModel):
    """Represents a record stored in the vector database"""
    record_id: str = Field(default="", description="Unique identifier for the record")
    embedding_vector: List[float] = Field(default_factory=list, description="The embedding vector to store")
    payload: Dict[str, Any] = Field(default_factory=dict, description="Metadata payload including source URL, content, section info")
    collection_name: str = Field(default="rag_embeddings", description="Name of the Qdrant collection")
    created_at: datetime = Field(default_factory=datetime.now, description="Timestamp of record creation")

    @field_validator('embedding_vector')
    @classmethod
    def validate_embedding_vector(cls, v):
        if not isinstance(v, list):
            raise ValueError('embedding_vector must be a list of floats')
        if len(v) == 0:
            raise ValueError('embedding_vector must not be empty')
        # Check that all elements are floats/numbers
        for item in v:
            if not isinstance(item, (int, float)):
                raise ValueError('embedding_vector must contain only numeric values')
        return v

    @field_validator('collection_name')
    @classmethod
    def validate_collection_name(cls, v):
        if not v:
            raise ValueError('collection_name must be provided')
        return v


class UserQuery(BaseModel):
    """
    Natural language question submitted by the user to the agent
    """
    query_text: str = Field(..., description="The actual question from the user")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the query was submitted")
    session_id: Optional[str] = Field(None, description="To track conversation context")
    user_id: Optional[str] = Field(None, description="Optional, to track user-specific interactions")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class RetrievedContentChunk(BaseModel):
    """
    Relevant text segment retrieved from Qdrant vector database based on semantic similarity
    """
    id: str = Field(..., description="Unique identifier from vector database")
    content: str = Field(..., description="The actual text content retrieved")
    similarity_score: float = Field(..., description="Relevance score from vector search", ge=0.0, le=1.0)
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Source information like URL, section heading")
    source_url: str = Field(..., description="URL where the content originated")
    section_heading: str = Field(..., description="Heading of the section containing the content")
    chunk_index: int = Field(..., description="Position of this chunk within the original document")


class SourceCitation(BaseModel):
    """
    Reference to specific section headings and URLs from the original content
    """
    url: str = Field(..., description="URL of the source content")
    section: str = Field(..., description="Section heading of the source")
    content_preview: str = Field(..., description="Brief preview of the cited content")
    relevance_score: float = Field(..., description="Similarity score from vector search", ge=0.0, le=1.0)


class AgentResponse(BaseModel):
    """
    Generated answer that is grounded in retrieved content with citations
    """
    response_text: str = Field(..., description="The agent's answer to the user")
    retrieved_chunks: List[RetrievedContentChunk] = Field(..., description="Content used to generate the response")
    citations: List[SourceCitation] = Field(default_factory=list, description="Structured citation information")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the response was generated")
    query_id: Optional[str] = Field(None, description="Link back to the original user query")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


def generate_document_id(url: str) -> str:
    """Generate a unique document ID based on the URL"""
    return f"doc_{hashlib.md5(url.encode()).hexdigest()}"


def generate_embedding_id(document_id: str) -> str:
    """Generate a unique embedding ID based on the document ID"""
    return f"emb_{hashlib.md5(document_id.encode()).hexdigest()}"


# API-specific models for FastAPI integration
class APIQueryRequest(BaseModel):
    """
    Model for incoming query requests to the API
    """
    query: str = Field(..., description="The user's query text", min_length=1, max_length=2000)
    context_type: str = Field("full_book", description="Either 'full_book' or 'selected_text'")
    selected_text: Optional[str] = Field(None, description="Text content for context-restricted queries")

    class Config:
        schema_extra = {
            "example": {
                "query": "What is the main concept discussed in chapter 3?",
                "context_type": "full_book",
                "selected_text": None
            }
        }


class APIQueryResponseData(BaseModel):
    """
    Model for the data portion of query responses
    """
    answer: str = Field(..., description="The agent's answer to the query")
    sources: List[SourceCitation] = Field(..., description="List of source citations")
    query_id: str = Field(..., description="Unique identifier for the query")


class APIQueryResponse(BaseModel):
    """
    Model for outgoing responses from the API
    """
    success: bool = Field(..., description="Whether the query was processed successfully")
    data: Optional[APIQueryResponseData] = Field(None, description="The response data if successful")
    error: Optional[str] = Field(None, description="Error message if the query failed")


class APIHealthResponse(BaseModel):
    """
    Model for health check responses
    """
    status: str = Field(..., description="Health status of the API")
    timestamp: str = Field(..., description="ISO 8601 timestamp of the check")

    class Config:
        schema_extra = {
            "example": {
                "status": "healthy",
                "timestamp": "2025-01-05T10:00:00Z"
            }
        }