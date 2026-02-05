#!/usr/bin/env python3
"""Script to check the number of points in the Qdrant collection"""

from qdrant_client import QdrantClient
import os
import yaml

def check_point_count():
    # Load config
    with open("config.yaml", "r") as f:
        config = yaml.safe_load(f)

    # Initialize Qdrant client
    url = os.getenv('QDRANT_URL')
    api_key = os.getenv('QDRANT_API_KEY')
    client = QdrantClient(url=url, api_key=api_key)

    # Get collection info
    collection_name = config['storage']['collection_name']

    try:
        collection_info = client.get_collection(collection_name)
        print(f"Collection '{collection_name}' has {collection_info.points_count} points")

        # Also try to list some points to verify content
        scroll_result = client.scroll(
            collection_name=collection_name,
            limit=5  # Just get first 5 points to verify
        )

        print(f"\nSample points (first 5):")
        for i, point in enumerate(scroll_result[0]):
            payload = point.payload if point.payload else {}
            print(f"  Point {i+1}: URL={payload.get('url', 'N/A')[:50]}...")

    except Exception as e:
        print(f"Error checking collection: {e}")

if __name__ == "__main__":
    check_point_count()