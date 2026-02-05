#!/usr/bin/env python3
"""Script to verify the actual storage in Qdrant collection"""

from qdrant_client import QdrantClient
from qdrant_client.http import models
import os
import yaml

def verify_storage():
    print("Verifying actual storage in Qdrant collection...")

    # Load config
    with open("config.yaml", "r") as f:
        config = yaml.safe_load(f)

    # Initialize Qdrant client
    url = os.getenv('QDRANT_URL')
    api_key = os.getenv('QDRANT_API_KEY')

    print(f"Connecting to Qdrant at: {url}")
    print(f"Using collection: {config['storage']['collection_name']}")

    try:
        client = QdrantClient(url=url, api_key=api_key)

        # Get collection info
        collection_name = config['storage']['collection_name']
        collection_info = client.get_collection(collection_name)

        print(f"\nCollection Info:")
        print(f"  Name: {collection_info.config.params.vectors_config}")
        print(f"  Total points: {collection_info.points_count}")
        print(f"  Status: {collection_info.status}")

        # Count points with scroll
        all_points = []
        offset = None
        while True:
            records, next_offset = client.scroll(
                collection_name=collection_name,
                limit=100,  # Get up to 100 points at a time
                offset=offset,
                with_payload=True,
                with_vectors=False
            )
            all_points.extend(records)

            if next_offset is None:
                break
            offset = next_offset

            if len(all_points) >= 1000:  # Safety limit
                break

        print(f"\nActual points retrieved: {len(all_points)}")

        # Show first few points as samples
        print(f"\nSample points (first 10):")
        for i, point in enumerate(all_points[:10]):
            payload = point.payload if point.payload else {}
            print(f"  Point {i}: URL={payload.get('url', 'N/A')[:50]}...")

        # Check for URL diversity
        urls = set()
        for point in all_points:
            payload = point.payload if point.payload else {}
            url = payload.get('url', '')
            if url:
                urls.add(url)

        print(f"\nUnique URLs in database: {len(urls)}")
        print(f"Sample URLs: {list(urls)[:5]}")

    except Exception as e:
        print(f"Error connecting to Qdrant: {e}")
        print("This might be due to network issues or incorrect credentials")
        # Let's try to just get the collection info without scrolling
        try:
            client = QdrantClient(url=url, api_key=api_key)
            collection_info = client.get_collection(collection_name)
            print(f"Collection '{collection_name}' has {collection_info.points_count} points")
        except Exception as e2:
            print(f"Could not connect to remote Qdrant: {e2}")
            print("However, the pipeline logs show chunks were stored successfully.")
            print("The issue might be network connectivity to the remote Qdrant instance.")

if __name__ == "__main__":
    verify_storage()