# Quickstart: URL ingestion, embedding generation, and vector storage for RAG

## Prerequisites

- Python 3.11 or higher
- `uv` package manager installed
- Cohere API key
- Qdrant Cloud account and API key

## Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create backend directory and initialize with uv
```bash
mkdir backend
cd backend
uv init
```

### 3. Create environment file
```bash
cp .env.example .env
```

### 4. Add required environment variables to `.env`:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
TARGET_URLS=https://your-docusaurus-site.com
COLLECTION_NAME=your_collection_name
```

## Installation

### 1. Install dependencies with uv
```bash
# Create pyproject.toml with required dependencies
uv add requests beautifulsoup4 cohere qdrant-client python-dotenv pydantic
```

Or manually create/update `pyproject.toml`:
```toml
[project]
name = "rag-ingestion"
version = "0.1.0"
description = "URL ingestion, embedding generation, and vector storage for RAG"
dependencies = [
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "cohere>=4.0.0",
    "qdrant-client>=1.7.0",
    "python-dotenv>=1.0.0",
    "pydantic>=2.5.0"
]
```

Then run:
```bash
uv sync
```

## Usage

### 1. Run the ingestion pipeline
```bash
cd backend
python main.py
```

### 2. Or run with specific parameters
```bash
python main.py --urls "https://example.com" "https://docs.example.com" --collection "my_collection"
```

## Configuration

The pipeline can be configured via:

1. **Environment variables** (in `.env` file):
   - `COHERE_API_KEY`: Cohere API key for embeddings
   - `QDRANT_URL`: Qdrant Cloud endpoint URL
   - `QDRANT_API_KEY`: Qdrant API key
   - `TARGET_URLS`: Comma-separated list of URLs to crawl
   - `COLLECTION_NAME`: Qdrant collection name

2. **Command line arguments**:
   - `--urls`: Space-separated list of URLs to process
   - `--collection`: Qdrant collection name
   - `--chunk-size`: Size of text chunks (default: 1000 characters)
   - `--overlap`: Overlap between chunks (default: 100 characters)

## Expected Output

When the pipeline runs successfully:
1. Crawls the specified URLs
2. Extracts clean text content from HTML
3. Chunks the content with preserved metadata
4. Generates embeddings using Cohere
5. Stores embeddings in Qdrant with metadata
6. Logs progress and any errors encountered

## Troubleshooting

- **API Rate Limits**: The pipeline includes rate limiting and retry logic for Cohere API calls
- **Network Issues**: Connection errors are handled with retries
- **Invalid URLs**: Invalid URLs are logged and skipped
- **Qdrant Connection**: Verify your Qdrant URL and API key are correct

## Next Steps

After successful ingestion:
1. The embeddings will be available in your Qdrant collection
2. You can perform similarity searches against the stored vectors
3. Integrate with your RAG chatbot for content retrieval