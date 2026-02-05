#!/usr/bin/env python3
"""Debug script to understand the query_points return format"""

from qdrant_client import QdrantClient
import os
import yaml
from pathlib import Path

def debug_query_format():
    print("Debugging Qdrant query_points return format...")

    # Load config
    with open("config.yaml", "r") as f:
        config = yaml.safe_load(f)

    # Initialize Qdrant client
    url = os.getenv('QDRANT_URL')
    api_key = os.getenv('QDRANT_API_KEY')
    client = QdrantClient(url=url, api_key=api_key)

    # Test a simple query to see the return format
    try:
        # Generate a simple test query (using a random vector)
        test_query = [0.1] * 1024  # 1024-dimensional vector like our embeddings

        results = client.query_points(
            collection_name=config['storage']['collection_name'],
            query=test_query,
            limit=2,
            with_payload=True
        )

        print(f"Results type: {type(results)}")
        print(f"Results: {results}")

        # Check if it's a list or object
        if hasattr(results, '__iter__') and not isinstance(results, str):
            print(f"Results length: {len(results) if hasattr(results, '__len__') else 'N/A'}")
            for i, result in enumerate(results):
                print(f"Result {i} type: {type(result)}")
                print(f"Result {i}: {result}")

                # Check attributes
                attrs = dir(result)
                print(f"Result {i} attributes: {[attr for attr in attrs if not attr.startswith('_')]}")

                if hasattr(result, 'payload'):
                    print(f"Result {i} payload: {result.payload}")
                if hasattr(result, 'score'):
                    print(f"Result {i} score: {result.score}")

                break  # Just check the first one

        else:
            # It might be a response object
            print(f"Single result attributes: {[attr for attr in dir(results) if not attr.startswith('_')]}")

            if hasattr(results, 'points'):
                print(f"Has points attribute")
                for i, point in enumerate(results.points):
                    print(f"Point {i} type: {type(point)}")
                    if hasattr(point, 'payload'):
                        print(f"Point {i} payload: {point.payload}")
                    if hasattr(point, 'score'):
                        print(f"Point {i} score: {point.score}")
                    break  # Just check the first one

    except Exception as e:
        print(f"Error in debug query: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_query_format()