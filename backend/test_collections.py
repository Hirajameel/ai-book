import os
from qdrant_client import QdrantClient
from config import get_agent_config

# Get configuration
config = get_agent_config()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=config.qdrant_url,
    api_key=config.qdrant_api_key,
    timeout=10.0
)

# List all collections
collections = qdrant_client.get_collections()
print("Available collections:")
for collection in collections.collections:
    print(f"- {collection.name}")

    # Get collection info to see point count
    try:
        collection_info = qdrant_client.get_collection(collection.name)
        print(f"  - Points: {collection_info.points_count}")
        print(f"  - Vector size: {collection_info.config.params.vectors.size}")
    except Exception as e:
        print(f"  - Error getting details: {e}")
    print()