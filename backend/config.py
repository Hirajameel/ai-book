"""
Configuration module for RAG Ingestion Pipeline and RAG Agent System
Handles environment variables and application constants for both ingestion and agent functionality
"""
import os
from dotenv import load_dotenv
from typing import List, Optional
from pydantic import BaseModel, Field, field_validator
from datetime import datetime


# Load environment variables
import os
from pathlib import Path

# Load .env file from the project root (parent directory)
env_path = Path(__file__).parent.parent / ".env"
if env_path.exists():
    load_dotenv(dotenv_path=env_path)
else:
    load_dotenv()  # Fallback to default behavior


class CrawlConfig(BaseModel):
    """Configuration for crawling operations"""
    target_urls: List[str] = Field(default_factory=list, description="List of URLs to crawl")
    max_depth: int = Field(default=1, description="Maximum depth for crawling")
    respect_robots_txt: bool = Field(default=True, description="Whether to respect robots.txt")
    crawl_delay: float = Field(default=1.0, description="Delay between requests in seconds")
    include_patterns: List[str] = Field(default_factory=list, description="URL patterns to include (regex)")
    exclude_patterns: List[str] = Field(default_factory=list, description="URL patterns to exclude (regex)")

    @field_validator('max_depth')
    @classmethod
    def validate_max_depth(cls, v):
        if v < 0:
            raise ValueError('max_depth must be non-negative')
        return v

    @field_validator('crawl_delay')
    @classmethod
    def validate_crawl_delay(cls, v):
        if v < 0:
            raise ValueError('crawl_delay must be non-negative')
        return v


class EmbeddingConfig(BaseModel):
    """Configuration for embedding generation"""
    model_name: str = Field(default="embed-english-v3.0", description="Name of the Cohere embedding model")
    input_type: str = Field(default="search_document", description="Type of input for embedding")
    batch_size: int = Field(default=96, description="Number of texts to process in each batch")
    api_key: str = Field(default="", description="Cohere API key")

    @field_validator('batch_size')
    @classmethod
    def validate_batch_size(cls, v):
        if v <= 0 or v > 96:  # Cohere API limit
            raise ValueError('batch_size must be between 1 and 96')
        return v

    @field_validator('api_key')
    @classmethod
    def validate_api_key(cls, v):
        if not v:
            raise ValueError('api_key must be provided')
        return v


class QdrantConfig(BaseModel):
    """Configuration for Qdrant vector database"""
    url: str = Field(default="", description="Qdrant Cloud URL")
    api_key: str = Field(default="", description="Qdrant API key")
    collection_name: str = Field(default="rag_embeddings", description="Name of the collection to use")
    vector_size: int = Field(default=1024, description="Size of embedding vectors (depends on model)")
    distance_metric: str = Field(default="Cosine", description="Distance metric for similarity search")

    @field_validator('url')
    @classmethod
    def validate_url(cls, v):
        if not v:
            raise ValueError('url must be provided')
        return v

    @field_validator('api_key')
    @classmethod
    def validate_qdrant_api_key(cls, v):
        if not v:
            raise ValueError('api_key must be provided')
        return v


class AgentConfig(BaseModel):
    """
    Configuration for the RAG Agent system
    """
    # OpenAI Configuration
    openai_api_key: Optional[str] = Field(default=None, description="OpenAI API key for agent functionality")

    # OpenRouter Configuration
    openrouter_api_key: Optional[str] = Field(default=None, description="OpenRouter API key for agent functionality")

    # Google Gemini Configuration (optional fallback)
    gemini_api_key: Optional[str] = Field(default=None, description="Google Gemini API key for fallback agent functionality")
    gemini_model: str = Field(default="gemini-2.0-flash", description="Model to use for Google Gemini")

    # Cohere Configuration (for retrieval)
    cohere_api_key: str = Field(..., description="Cohere API key for embedding generation")
    cohere_model_name: str = Field(default="embed-english-v3.0", description="Name of the Cohere embedding model")
    cohere_input_type: str = Field(default="search_query", description="Type of input for embedding")

    # Qdrant Configuration (for retrieval)
    qdrant_url: str = Field(..., description="Qdrant Cloud URL")
    qdrant_api_key: str = Field(..., description="Qdrant API key")
    qdrant_collection_name: str = Field(default="rag_embeddings", description="Name of the collection to use")
    qdrant_vector_size: int = Field(default=1024, description="Size of embedding vectors (depends on model)")
    qdrant_distance_metric: str = Field(default="Cosine", description="Distance metric for similarity search")

    # Agent Configuration
    agent_instructions: str = Field(
        default="You are a helpful AI assistant that answers questions based on retrieved content from book materials. "
                "Always ground your responses in the provided context and cite the sources.",
        description="Instructions for the agent"
    )
    agent_model: str = Field(default="gpt-4-turbo", description="Model to use for the agent")

    # Retrieval Configuration
    retrieval_top_k: int = Field(default=5, description="Number of results to retrieve from vector database")
    retrieval_threshold: float = Field(default=0.3, description="Minimum similarity score threshold for retrieval")

    @field_validator('cohere_api_key', 'qdrant_url', 'qdrant_api_key')
    @classmethod
    def validate_required_fields(cls, v):
        """Validate that required API keys and URLs are provided"""
        if not v:
            raise ValueError(f"Required configuration field is missing")
        return v

    @field_validator('qdrant_vector_size')
    @classmethod
    def validate_vector_size(cls, v):
        """Validate vector size is positive"""
        if v <= 0:
            raise ValueError('qdrant_vector_size must be positive')
        return v

    @field_validator('retrieval_top_k')
    @classmethod
    def validate_top_k(cls, v):
        """Validate top_k is positive"""
        if v <= 0:
            raise ValueError('retrieval_top_k must be positive')
        return v

    @field_validator('retrieval_threshold')
    @classmethod
    def validate_threshold(cls, v):
        """Validate threshold is between 0 and 1"""
        if v < 0 or v > 1:
            raise ValueError('retrieval_threshold must be between 0 and 1')
        return v


def load_config_from_env() -> tuple[CrawlConfig, EmbeddingConfig, QdrantConfig]:
    """Load configuration from environment variables with validation (for ingestion pipeline)"""
    # Load target URLs from environment
    target_urls_str = os.getenv("TARGET_URLS", "")
    target_urls = [url.strip() for url in target_urls_str.split(",") if url.strip()] if target_urls_str else []

    crawl_config = CrawlConfig(
        target_urls=target_urls,
        max_depth=int(os.getenv("MAX_DEPTH", "1")),
        respect_robots_txt=os.getenv("RESPECT_ROBOTS_TXT", "true").lower() == "true",
        crawl_delay=float(os.getenv("CRAWL_DELAY", "1.0")),
    )

    embedding_config = EmbeddingConfig(
        model_name=os.getenv("COHERE_MODEL_NAME", "embed-english-v3.0"),
        input_type=os.getenv("COHERE_INPUT_TYPE", "search_document"),
        batch_size=int(os.getenv("COHERE_BATCH_SIZE", "96")),
        api_key=os.getenv("COHERE_API_KEY", ""),
    )

    qdrant_config = QdrantConfig(
        url=os.getenv("QDRANT_URL", ""),
        api_key=os.getenv("QDRANT_API_KEY", ""),
        collection_name=os.getenv("QDRANT_COLLECTION_NAME", "rag_embeddings"),
        vector_size=int(os.getenv("QDRANT_VECTOR_SIZE", "1024")),
        distance_metric=os.getenv("QDRANT_DISTANCE_METRIC", "Cosine"),
    )

    return crawl_config, embedding_config, qdrant_config


def load_agent_config() -> AgentConfig:
    """
    Load agent-specific configuration from environment variables

    Returns:
        AgentConfig: Validated configuration object for the agent
    """
    # Get environment variables with defaults
    config_data = {
        "openai_api_key": os.getenv("OPENAI_API_KEY"),
        "openrouter_api_key": os.getenv("OPENROUTER_API_KEY"),
        "gemini_api_key": os.getenv("GEMINI_API_KEY"),
        "gemini_model": os.getenv("GEMINI_MODEL", "gemini-2.0-flash"),
        "cohere_api_key": os.getenv("COHERE_API_KEY"),
        "qdrant_url": os.getenv("QDRANT_URL"),
        "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
        "qdrant_collection_name": os.getenv("QDRANT_COLLECTION_NAME", "rag_embeddings"),
        "qdrant_vector_size": int(os.getenv("QDRANT_VECTOR_SIZE", "1024")),
        "qdrant_distance_metric": os.getenv("QDRANT_DISTANCE_METRIC", "Cosine"),
        "cohere_model_name": os.getenv("COHERE_MODEL_NAME", "embed-english-v3.0"),
        "cohere_input_type": os.getenv("COHERE_INPUT_TYPE", "search_query"),
        "agent_model": os.getenv("AGENT_MODEL", "gpt-4-turbo"),
        "retrieval_top_k": int(os.getenv("RETRIEVAL_TOP_K", "5")),
        "retrieval_threshold": float(os.getenv("RETRIEVAL_THRESHOLD", "0.3")),
        "agent_instructions": os.getenv(
            "AGENT_INSTRUCTIONS",
            "You are a helpful AI assistant that answers questions based on retrieved content from book materials. "
            "Always ground your responses in the provided context and cite the sources."
        )
    }

    # Validate and return the configuration
    config = AgentConfig(**config_data)
    return config


def validate_config(crawl_config: CrawlConfig, embedding_config: EmbeddingConfig, qdrant_config: QdrantConfig):
    """Validate all configurations"""
    import logging
    try:
        # In Pydantic v2, validation happens automatically during model creation
        # We just need to ensure the models were created successfully
        crawl_config.model_dump()
        embedding_config.model_dump()
        qdrant_config.model_dump()
        return True
    except Exception as e:
        logging.error(f"Configuration validation failed: {str(e)}")
        return False


# Global configuration instances
ingestion_crawl_config: Optional[CrawlConfig] = None
ingestion_embedding_config: Optional[EmbeddingConfig] = None
ingestion_qdrant_config: Optional[QdrantConfig] = None
agent_config: Optional[AgentConfig] = None


def get_ingestion_configs() -> tuple[CrawlConfig, EmbeddingConfig, QdrantConfig]:
    """
    Get the global ingestion configuration instances, loading them if necessary

    Returns:
        tuple: The global configuration instances for crawling, embedding, and Qdrant
    """
    global ingestion_crawl_config, ingestion_embedding_config, ingestion_qdrant_config
    if ingestion_crawl_config is None or ingestion_embedding_config is None or ingestion_qdrant_config is None:
        ingestion_crawl_config, ingestion_embedding_config, ingestion_qdrant_config = load_config_from_env()
    return ingestion_crawl_config, ingestion_embedding_config, ingestion_qdrant_config


def get_agent_config() -> AgentConfig:
    """
    Get the global agent configuration instance, loading it if necessary

    Returns:
        AgentConfig: The global agent configuration instance
    """
    global agent_config
    if agent_config is None:
        agent_config = load_agent_config()
    return agent_config