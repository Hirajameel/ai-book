#!/usr/bin/env python3
"""
Script to clear existing Qdrant collection and re-run ingestion with fixed content storage
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from config import get_agent_config
from qdrant_client import QdrantClient

def clear_qdrant_collection():
    """Clear the existing Qdrant collection"""
    print("Clearing existing Qdrant collection...")

    try:
        config = get_agent_config()

        # Create Qdrant client
        qdrant_client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            timeout=30.0
        )

        # Get collection info
        collection_info = qdrant_client.get_collection(config.qdrant_collection_name)
        print(f"Current collection '{config.qdrant_collection_name}' has {collection_info.points_count} points")

        # Clear the collection by deleting and recreating it
        qdrant_client.delete_collection(config.qdrant_collection_name)
        print(f"Deleted collection '{config.qdrant_collection_name}'")

        # Recreate the collection
        from qdrant_client.http import models
        qdrant_client.create_collection(
            collection_name=config.qdrant_collection_name,
            vectors_config=models.VectorParams(
                size=config.qdrant_vector_size,
                distance=models.Distance[config.qdrant_distance_metric.upper()]
            )
        )
        print(f"Recreated collection '{config.qdrant_collection_name}'")

        # Verify it's empty
        collection_info = qdrant_client.get_collection(config.qdrant_collection_name)
        print(f"New collection has {collection_info.points_count} points")

        return True

    except Exception as e:
        print(f"Error clearing Qdrant collection: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = clear_qdrant_collection()
    if success:
        print("\n✅ Qdrant collection cleared successfully!")
        print("Now run 'python main.py' to re-ingest with fixed content storage")
    else:
        print("\n❌ Failed to clear Qdrant collection")
        sys.exit(1)