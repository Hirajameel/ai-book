# Quickstart: RAG Agent with OpenAI SDK

## Prerequisites

- Python 3.11+
- OpenAI API key
- Cohere API key
- Qdrant Cloud URL and API key
- Existing vector database with book content embeddings

## Setup

1. **Install dependencies**:
   ```bash
   pip install openai cohere qdrant-client python-dotenv pydantic
   ```

2. **Configure environment variables**:
   ```bash
   # Create .env file in backend/ directory
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=rag_embeddings
   ```

3. **Verify existing embeddings**:
   - Ensure your Qdrant collection contains book content embeddings
   - Verify the embedding model consistency (Cohere embed-english-v3.0)

## Usage

1. **Initialize the agent**:
   ```bash
   cd backend
   python agent.py
   ```

2. **Query the agent**:
   - The agent will accept natural language questions
   - It will retrieve relevant content from the vector database
   - It will respond with answers grounded in the retrieved content
   - Responses will include citations to original sections and URLs

## Testing

1. **Basic functionality**:
   ```bash
   # Test with a sample query
   python -c "from agent import test_agent; test_agent('What are Digital Twins in Robotics?')"
   ```

2. **Verify citations**:
   - Check that responses include source URLs and section headings
   - Verify that content is grounded in retrieved chunks

## Troubleshooting

- **No results returned**: Verify that Qdrant collection has content and API keys are correct
- **Poor quality responses**: Check embedding quality and similarity thresholds
- **API errors**: Verify all API keys are valid and have sufficient quota