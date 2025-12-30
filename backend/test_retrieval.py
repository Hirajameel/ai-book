#!/usr/bin/env python3
"""
Test script to verify the retrieval functionality is working properly
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from retrieval_tool import retrieve_content_with_validation
from config import get_agent_config

def test_retrieval():
    """Test the retrieval functionality directly"""
    print("Testing retrieval functionality...")

    try:
        # Get configuration
        config = get_agent_config()
        print(f"Using Qdrant collection: {config.qdrant_collection_name}")
        print(f"Retrieval top_k: {config.retrieval_top_k}")
        print(f"Retrieval threshold: {config.retrieval_threshold}")

        # Test query
        test_query = "What is artificial intelligence?"
        print(f"\nTesting query: '{test_query}'")

        # Call the retrieval function directly
        chunks = retrieve_content_with_validation(
            query=test_query,
            top_k=config.retrieval_top_k,
            threshold=config.retrieval_threshold
        )

        print(f"Retrieved {len(chunks)} chunks")

        if chunks:
            print("\nFirst few retrieved chunks:")
            for i, chunk in enumerate(chunks[:3]):  # Show first 3 chunks
                print(f"  Chunk {i+1}:")
                print(f"    ID: {chunk.id}")
                print(f"    Source: {chunk.source_url}")
                print(f"    Heading: {chunk.section_heading}")
                print(f"    Similarity: {chunk.similarity_score:.3f}")
                print(f"    Content preview: {chunk.content[:200]}...")
                print()
        else:
            print("No chunks retrieved - this might indicate an issue with the Qdrant data or retrieval settings.")

        return len(chunks) > 0

    except Exception as e:
        print(f"Error during retrieval test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def test_qdrant_connection():
    """Test direct Qdrant connection and collection"""
    print("\nTesting Qdrant connection...")

    try:
        from qdrant_client import QdrantClient
        from qdrant_client.http import models
        import cohere

        config = get_agent_config()

        # Create Qdrant client
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            timeout=10.0
        )

        # Test connection
        collections = client.get_collections()
        print(f"Available collections: {[col.name for col in collections.collections]}")

        # Check if our collection exists
        collection_exists = any(col.name == config.qdrant_collection_name for col in collections.collections)
        print(f"Collection '{config.qdrant_collection_name}' exists: {collection_exists}")

        if collection_exists:
            # Count vectors in collection
            count_result = client.count(collection_name=config.qdrant_collection_name)
            print(f"Vector count in collection: {count_result.count}")

            # Try a simple search with a test embedding
            cohere_client = cohere.Client(api_key=config.cohere_api_key)
            test_embedding = cohere_client.embed(
                texts=["test query"],
                model=config.cohere_model_name,
                input_type="search_query"
            ).embeddings[0]

            # Perform a search
            search_results = client.search(
                collection_name=config.qdrant_collection_name,
                query_vector=test_embedding,
                limit=5
            )

            print(f"Search test returned {len(search_results)} results")

            if search_results:
                print("Sample search result:")
                result = search_results[0]
                print(f"  ID: {result.id}")
                print(f"  Score: {result.score}")
                print(f"  Payload keys: {list(result.payload.keys()) if result.payload else 'None'}")

        return True

    except Exception as e:
        print(f"Error testing Qdrant connection: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("RAG Retrieval Test")
    print("=" * 50)

    # Test Qdrant connection first
    qdrant_ok = test_qdrant_connection()

    # Test retrieval functionality
    retrieval_ok = test_retrieval()

    print("\n" + "=" * 50)
    print("Test Summary:")
    print(f"  Qdrant Connection: {'✅ OK' if qdrant_ok else '❌ FAILED'}")
    print(f"  Retrieval Function: {'✅ OK' if retrieval_ok else '❌ FAILED'}")

    if qdrant_ok and retrieval_ok:
        print("\n🎉 Retrieval system is working properly!")
        return True
    else:
        print("\n💥 There are issues with the retrieval system.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)