#!/usr/bin/env python3
"""
Test script to determine the exact method signature for Qdrant search
"""

from qdrant_client import QdrantClient
import inspect

def main():
    print("Testing actual QdrantClient method signatures...")

    # Check the signature of the search method directly
    if hasattr(QdrantClient, 'search'):
        try:
            sig = inspect.signature(QdrantClient.search)
            print(f"* search method exists with signature: {sig}")
        except Exception as e:
            print(f"x Could not get signature for search: {e}")
    else:
        print("x search method does not exist")

    # Check the signature of the query method
    if hasattr(QdrantClient, 'query'):
        try:
            sig = inspect.signature(QdrantClient.query)
            print(f"* query method exists with signature: {sig}")
        except Exception as e:
            print(f"x Could not get signature for query: {e}")
    else:
        print("x query method does not exist")

    # Check the signature of the query_points method
    if hasattr(QdrantClient, 'query_points'):
        try:
            sig = inspect.signature(QdrantClient.query_points)
            print(f"* query_points method exists with signature: {sig}")
        except Exception as e:
            print(f"x Could not get signature for query_points: {e}")
    else:
        print("x query_points method does not exist")

    # Check the signature of search_points method
    if hasattr(QdrantClient, 'search_points'):
        try:
            sig = inspect.signature(QdrantClient.search_points)
            print(f"* search_points method exists with signature: {sig}")
        except Exception as e:
            print(f"x Could not get signature for search_points: {e}")
    else:
        print("x search_points method does not exist")

    # Let's try to create a temporary client to see what methods are available at instance level
    print("\nTesting with a real client instance (will fail gracefully if no connection):")
    try:
        # Create a client with a fake URL to see instance methods
        client = QdrantClient(url="http://fake-url-for-testing", timeout=1)

        # Check if search method is available on instance
        instance_methods = [method for method in dir(client) if 'search' in method.lower() or 'query' in method.lower()]
        print(f"Search/query methods on instance: {instance_methods}")

    except Exception as e:
        print(f"Could not create test client (expected): {e}")

    # Based on the earlier output, it seems like the correct method might be query_points
    print("\nBased on our investigation, the most likely correct method for vector search is query_points or search.")
    print("We should use the query_points method with query vector for semantic search.")

if __name__ == "__main__":
    main()