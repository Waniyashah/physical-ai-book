#!/usr/bin/env python3
"""
Test client for FastAPI RAG Integration

This module provides a simple test client to simulate frontend requests
to the backend API for testing end-to-end communication.
"""

import requests
import json
import time
from typing import Dict, Any, Optional
from datetime import datetime


class TestClient:
    """
    Simple test client to simulate frontend requests to the backend API.
    """

    def __init__(self, base_url: str = "http://localhost:8000"):
        """
        Initialize the test client.

        Args:
            base_url: Base URL for the API
        """
        self.base_url = base_url
        self.session = requests.Session()

        # Set default headers
        self.session.headers.update({
            "Content-Type": "application/json",
            "User-Agent": "FastAPI-RAG-Test-Client/1.0"
        })

    def health_check(self) -> Dict[str, Any]:
        """
        Perform a health check on the API.

        Returns:
            Health check response from the API
        """
        try:
            url = f"{self.base_url}/api/health"
            response = self.session.get(url)

            print(f"Health check status: {response.status_code}")

            if response.status_code == 200:
                health_data = response.json()
                print(f"Health check result: {health_data['status']}")
                print(f"Services status: {health_data['services']}")
                return health_data
            else:
                print(f"Health check failed with status {response.status_code}")
                print(f"Response: {response.text}")
                return {"error": f"Health check failed with status {response.status_code}"}

        except Exception as e:
            print(f"Error during health check: {str(e)}")
            return {"error": str(e)}

    def query(self, query_text: str, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Send a query to the API.

        Args:
            query_text: The query text to send
            options: Optional query options

        Returns:
            Response from the query endpoint
        """
        try:
            url = f"{self.base_url}/api/query"

            payload = {
                "query": query_text
            }

            if options:
                payload["options"] = options

            print(f"Sending query: {query_text[:50]}{'...' if len(query_text) > 50 else ''}")

            start_time = time.time()
            response = self.session.post(url, json=payload)
            end_time = time.time()

            processing_time = end_time - start_time

            print(f"Query response status: {response.status_code}")
            print(f"Processing time: {processing_time:.2f}s")

            if response.status_code == 200:
                result = response.json()
                print(f"Query successful: {result['success']}")
                print(f"Response length: {len(result['response'])} characters")
                print(f"Sources found: {len(result['sources'])}")

                return result
            else:
                print(f"Query failed with status {response.status_code}")
                print(f"Response: {response.text}")
                return {"error": f"Query failed with status {response.status_code}", "details": response.text}

        except Exception as e:
            print(f"Error during query: {str(e)}")
            return {"error": str(e)}

    def test_end_to_end_flow(self, test_queries: Optional[list] = None) -> Dict[str, Any]:
        """
        Test the complete end-to-end flow with multiple queries.

        Args:
            test_queries: List of test queries to run

        Returns:
            Summary of the end-to-end test results
        """
        if test_queries is None:
            test_queries = [
                "What is this documentation about?",
                "Explain the RAG system architecture",
                "How does the retrieval pipeline work?"
            ]

        results = {
            "start_time": datetime.now().isoformat(),
            "queries_run": 0,
            "successful_queries": 0,
            "failed_queries": 0,
            "query_results": [],
            "health_check_passed": False
        }

        print("Starting end-to-end flow test...")

        # First, perform a health check
        print("\n1. Performing health check...")
        health_result = self.health_check()
        results["health_check_passed"] = health_result.get("status") == "ok"

        # Run test queries
        print(f"\n2. Running {len(test_queries)} test queries...")
        for i, query in enumerate(test_queries, 1):
            print(f"\n  Query {i}/{len(test_queries)}: {query}")

            query_result = self.query(query)
            results["query_results"].append({
                "query": query,
                "result": query_result,
                "timestamp": datetime.now().isoformat()
            })

            results["queries_run"] += 1

            if query_result.get("success"):
                results["successful_queries"] += 1
            else:
                results["failed_queries"] += 1

        results["end_time"] = datetime.now().isoformat()
        results["total_duration"] = (
            datetime.fromisoformat(results["end_time"]) -
            datetime.fromisoformat(results["start_time"])
        ).total_seconds()

        print(f"\nEnd-to-end test completed:")
        print(f"  Total duration: {results['total_duration']:.2f}s")
        print(f"  Successful queries: {results['successful_queries']}/{results['queries_run']}")
        print(f"  Failed queries: {results['failed_queries']}/{results['queries_run']}")
        print(f"  Health check passed: {results['health_check_passed']}")

        return results


def main():
    """
    Main function to run the test client.
    """
    print("FastAPI RAG Integration - Test Client")
    print("=" * 50)

    # Create test client
    client = TestClient(base_url="http://localhost:8000")

    # Run end-to-end test
    test_results = client.test_end_to_end_flow([
        "What is the RAG system?",
        "How does the FastAPI integration work?",
        "Explain the architecture of this system."
    ])

    # Print summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)
    print(f"Health Check Passed: {test_results['health_check_passed']}")
    print(f"Queries Run: {test_results['queries_run']}")
    print(f"Successful: {test_results['successful_queries']}")
    print(f"Failed: {test_results['failed_queries']}")
    print(f"Success Rate: {(test_results['successful_queries']/test_results['queries_run']*100) if test_results['queries_run'] > 0 else 0:.1f}%")
    print(f"Total Duration: {test_results['total_duration']:.2f}s")


if __name__ == "__main__":
    main()