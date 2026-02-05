#!/usr/bin/env python3
"""
Script to check available methods in Qdrant client and find the correct search method
"""

from qdrant_client import QdrantClient
import inspect

def main():
    print("Checking QdrantClient methods...")

    # Get all methods that contain 'search' or 'query'
    all_methods = [method for method in dir(QdrantClient) if not method.startswith('_')]

    search_methods = [method for method in all_methods if 'search' in method.lower()]
    query_methods = [method for method in all_methods if 'query' in method.lower()]

    print(f"Search-related methods: {search_methods}")
    print(f"Query-related methods: {query_methods}")

    # Let's check the actual QdrantClient class methods
    client_methods = []
    for attr_name in dir(QdrantClient):
        if not attr_name.startswith('_'):
            attr = getattr(QdrantClient, attr_name)
            if callable(attr) and not isinstance(attr, type):
                client_methods.append(attr_name)

    print("\nAll public methods in QdrantClient:")
    for method in sorted(client_methods):
        print(f"  - {method}")

    # Specifically check for the methods we need
    print("\nDetailed check for specific methods:")

    # Check if search method exists and get its signature
    if hasattr(QdrantClient, 'search'):
        print("✓ search method exists")
        try:
            sig = inspect.signature(QdrantClient.search)
            print(f"  Signature: {sig}")
        except Exception as e:
            print(f"  Could not get signature: {e}")
    else:
        print("✗ search method does not exist")

    # Check if query method exists and get its signature
    if hasattr(QdrantClient, 'query'):
        print("✓ query method exists")
        try:
            sig = inspect.signature(QdrantClient.query)
            print(f"  Signature: {sig}")
        except Exception as e:
            print(f"  Could not get signature: {e}")
    else:
        print("✗ query method does not exist")

    # Check for search_points
    if hasattr(QdrantClient, 'search_points'):
        print("✓ search_points method exists")
        try:
            sig = inspect.signature(QdrantClient.search_points)
            print(f"  Signature: {sig}")
        except Exception as e:
            print(f"  Could not get signature: {e}")
    else:
        print("✗ search_points method does not exist")

if __name__ == "__main__":
    main()