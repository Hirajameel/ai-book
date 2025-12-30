# Data Model: URL ingestion, embedding generation, and vector storage for RAG

## Entity: Document

**Description**: Represents a single crawled document/page from a Docusaurus site

**Fields**:
- `id` (str): Unique identifier for the document
- `source_url` (str): Original URL of the document
- `content_text` (str): Clean text extracted from the HTML
- `section_heading` (str): Main section/heading context of the content
- `chunk_index` (int): Sequential index for document chunks
- `processing_status` (str): Status of processing (pending, in_progress, completed, failed)
- `created_at` (datetime): Timestamp of document creation
- `updated_at` (datetime): Timestamp of last update

**Validation Rules**:
- `source_url` must be a valid URL format
- `content_text` must not be empty
- `chunk_index` must be non-negative
- `processing_status` must be one of the allowed values

## Entity: Embedding

**Description**: Represents an embedding vector generated from document content

**Fields**:
- `id` (str): Unique identifier for the embedding
- `vector` (List[float]): The embedding vector representation
- `associated_metadata` (Dict): Metadata associated with the embedding
- `source_document_id` (str): Reference to the source document
- `created_at` (datetime): Timestamp of embedding creation

**Validation Rules**:
- `vector` must be a list of floats
- `vector` length must match the expected embedding dimension (typically 1024 for Cohere)
- `source_document_id` must reference an existing document

## Entity: VectorRecord

**Description**: Represents a record stored in the vector database

**Fields**:
- `record_id` (str): Unique identifier for the record
- `embedding_vector` (List[float]): The embedding vector to store
- `payload` (Dict): Metadata payload including source URL, content, section info
- `collection_name` (str): Name of the Qdrant collection
- `created_at` (datetime): Timestamp of record creation

**Validation Rules**:
- `embedding_vector` must match the collection's expected vector size
- `payload` must contain required metadata fields
- `collection_name` must exist in Qdrant

## Entity: CrawlConfig

**Description**: Configuration for crawling operations

**Fields**:
- `target_urls` (List[str]): List of URLs to crawl
- `max_depth` (int): Maximum depth for crawling (default: 1)
- `respect_robots_txt` (bool): Whether to respect robots.txt (default: True)
- `crawl_delay` (float): Delay between requests in seconds (default: 1.0)
- `include_patterns` (List[str]): URL patterns to include (regex)
- `exclude_patterns` (List[str]): URL patterns to exclude (regex)

**Validation Rules**:
- `target_urls` must contain at least one valid URL
- `max_depth` must be positive
- `crawl_delay` must be non-negative

## Entity: EmbeddingConfig

**Description**: Configuration for embedding generation

**Fields**:
- `model_name` (str): Name of the Cohere embedding model (default: "embed-english-v3.0")
- `input_type` (str): Type of input for embedding (default: "search_document")
- `batch_size` (int): Number of texts to process in each batch (default: 96)
- `api_key` (str): Cohere API key (from environment)

**Validation Rules**:
- `model_name` must be a valid Cohere model
- `batch_size` must be within API limits
- `api_key` must be provided

## Entity: QdrantConfig

**Description**: Configuration for Qdrant vector database

**Fields**:
- `url` (str): Qdrant Cloud URL (from environment)
- `api_key` (str): Qdrant API key (from environment)
- `collection_name` (str): Name of the collection to use
- `vector_size` (int): Size of embedding vectors (depends on model)
- `distance_metric` (str): Distance metric for similarity search (default: "Cosine")

**Validation Rules**:
- `url` must be a valid URL
- `collection_name` must follow Qdrant naming conventions
- `vector_size` must match the embedding model's output

## Relationships

1. **Document → Embedding**: One-to-many relationship
   - One document can generate multiple embeddings (one per chunk)

2. **Embedding → VectorRecord**: One-to-one relationship
   - Each embedding corresponds to one vector record in the database

3. **CrawlConfig**: Standalone configuration entity

4. **EmbeddingConfig**: Standalone configuration entity

5. **QdrantConfig**: Standalone configuration entity

## State Transitions

### Document Processing States:
```
pending → in_progress → completed
              ↓
            failed
```

- `pending`: Document queued for processing
- `in_progress`: Document currently being processed
- `completed`: Document successfully processed
- `failed`: Document processing failed with error details