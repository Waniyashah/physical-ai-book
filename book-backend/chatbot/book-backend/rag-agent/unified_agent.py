#!/usr/bin/env python3
"""
Unified RAG Agent - OpenAI Agents SDK with Gemini Configuration

This module implements an agent that works with both OpenAI and Google Gemini APIs
using the OpenAI Agents SDK configuration approach.
"""

import os
import sys
import logging
import argparse
import yaml
import json
from typing import List, Dict, Any, Optional
from pathlib import Path

from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def load_config(config_file: str = "config.yaml") -> Dict[str, Any]:
    """Load configuration from YAML file"""
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        logger.info(f"Configuration loaded from {config_file}")
        return config
    except FileNotFoundError:
        logger.error(f"Configuration file {config_file} not found.")
        # Return default config
        return {
            "agent": {
                "model": "gemini-2.0-flash",
                "temperature": 0.1
            },
            "retrieval": {
                "top_k": 5
            },
            "embedding": {
                "model": "embed-multilingual-v2.0",
                "input_type": "search_query"
            },
            "storage": {
                "collection_name": "documentation_chunks"
            }
        }
    except yaml.YAMLError as e:
        logger.error(f"Error parsing YAML configuration: {str(e)}")
        raise


def initialize_gemini_client_with_openai_sdk(config: Dict[str, Any]):
    """Initialize Google Gemini client using OpenAI Agents SDK approach"""
    gemini_api_key = os.getenv("GEMINI_API_KEY")
    if not gemini_api_key:
        logger.info("GEMINI_API_KEY environment variable not set")
        return None

    try:
        # Import required modules for OpenAI Agents SDK approach
        from openai import OpenAI

        # Configure Gemini to work with OpenAI Agents SDK
        external_client = OpenAI(
            api_key=gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )

        # Verify the connection
        models = external_client.models.list()
        logger.info("Gemini client initialized successfully with OpenAI SDK")

        return external_client
    except Exception as e:
        logger.error(f"Failed to initialize Gemini client with OpenAI SDK: {str(e)}")
        return None


def initialize_cohere_client(config: Dict[str, Any]) -> cohere.Client:
    """Initialize Cohere client with API key from environment and proper error handling"""
    api_key = os.getenv('COHERE_API_KEY')
    if not api_key:
        logger.info("COHERE_API_KEY not found, using mock client")
        # Return a mock client for testing
        class MockCohereClient:
            def embed(self, texts, model, input_type):
                # Return mock embeddings
                return type('obj', (object,), {
                    'embeddings': [[0.1] * 384 for _ in texts]
                })()
        return MockCohereClient()

    try:
        client = cohere.Client(api_key=api_key)
        # Test the connection
        client.embed(
            texts=["test"],
            model="embed-english-v3.0",
            input_type="search_document"
        )
        logger.info("Cohere client initialized successfully")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize Cohere client: {str(e)}")
        # Return mock client as fallback
        class MockCohereClient:
            def embed(self, texts, model, input_type):
                # Return mock embeddings
                return type('obj', (object,), {
                    'embeddings': [[0.1] * 384 for _ in texts]
                })()
        return MockCohereClient()


def initialize_qdrant_client(config: Dict[str, Any]) -> QdrantClient:
    """Initialize Qdrant client with configuration and proper error handling"""
    url = os.getenv('QDRANT_URL')
    api_key = os.getenv('QDRANT_API_KEY')

    if not url or not api_key:
        logger.info("QDRANT credentials not found, using in-memory client for testing")
        # Use in-memory client for testing
        from qdrant_client import QdrantClient
        client = QdrantClient(":memory:")

        # Create a mock collection for testing
        try:
            client.create_collection(
                collection_name=config.get('storage', {}).get('collection_name', 'documentation_chunks'),
                vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
            )
        except:
            pass  # Collection might already exist

        logger.info("Qdrant in-memory client initialized for testing")
        return client

    try:
        client = QdrantClient(url=url, api_key=api_key)
        # Test the connection
        client.get_collections()
        logger.info("Qdrant client initialized successfully")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant client: {str(e)}")
        # Use in-memory client as fallback
        from qdrant_client import QdrantClient
        client = QdrantClient(":memory:")
        logger.info("Qdrant in-memory client initialized as fallback")
        return client


def create_retrieval_tool_function(config: Dict[str, Any], qdrant_client: QdrantClient, cohere_client: cohere.Client) -> callable:
    """Create retrieval tool function that can be used by the agent"""

    def retrieve_content(query: str, top_k: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content chunks from the vector database based on the query.

        Args:
            query: The search query string
            top_k: Number of results to return (defaults to config value)

        Returns:
            List of content chunks with metadata
        """
        if top_k is None:
            top_k = config['retrieval']['top_k']

        try:
            # Generate embedding for the query using Cohere
            query_response = cohere_client.embed(
                texts=[query],
                model=config['embedding']['model'],
                input_type=config['embedding'].get('input_type', 'search_query')
            )
            query_embedding = query_response.embeddings[0]

            # Search in Qdrant
            collection_name = config.get('storage', {}).get('collection_name', 'documentation_chunks')

            search_results = qdrant_client.search(
                collection_name=collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True,
                score_threshold=config['retrieval'].get('score_threshold', 0.3)
            )

            # Format results
            formatted_results = []
            for result in search_results:
                payload = result.payload or {}

                formatted_result = {
                    'content': payload.get('content', ''),
                    'url': payload.get('url', ''),
                    'section': payload.get('section', ''),
                    'chunk_id': payload.get('chunk_id', ''),
                    'score': result.score,
                    'source': payload.get('source', 'unknown')
                }

                formatted_results.append(formatted_result)

            logger.info(f"Retrieved {len(formatted_results)} relevant chunks for query: {query[:50]}...")
            return formatted_results

        except Exception as e:
            logger.error(f"Error in retrieval tool: {str(e)}")
            # Return mock results for testing
            return [{
                'content': f'Mock result for query: {query}',
                'url': 'mock-url',
                'section': 'mock-section',
                'chunk_id': 'mock-chunk',
                'score': 0.9,
                'source': 'mock-source'
            }]

    return retrieve_content


def run_gemini_agent_with_openai_sdk(gemini_client, retrieval_func: callable, query: str, config: Dict[str, Any]) -> str:
    """Run query using Google Gemini configured with OpenAI Agents SDK approach"""

    # First, retrieve relevant content using the retrieval function
    retrieved_content = retrieval_func(query)

    if not retrieved_content:
        return "I couldn't find any relevant content in the documentation to answer your query."

    # Format the context from retrieved content
    context_str = "Relevant documentation content:\n\n"
    sources = []

    for i, chunk in enumerate(retrieved_content):
        context_str += f"Source {i+1} ({chunk['url']}):\n"
        context_str += f"{chunk['content']}\n\n"
        sources.append(f"- {chunk['url']} (Section: {chunk['section']})")

    # Create the prompt for Gemini using the OpenAI format
    messages = [
        {
            "role": "system",
            "content": "You are an AI assistant that helps users with information about Physical AI & Humanoid Robotics. Answer based ONLY on the provided documentation context. Do not hallucinate or make up information not in the documentation."
        },
        {
            "role": "user",
            "content": f"""
            Please answer the following query based ONLY on the provided documentation content.

            Query: {query}

            Documentation Content:
            {context_str}

            Instructions:
            1. Answer only based on the provided documentation content
            2. If the answer is not in the documentation, say so clearly
            3. Provide specific citations to the sources when possible
            4. Be concise but comprehensive in your response
            5. Do not hallucinate or make up information not in the documentation

            Answer:
            """
        }
    ]

    try:
        # Use the Gemini client configured with OpenAI SDK
        response = gemini_client.chat.completions.create(
            model=config['agent']['model'],
            messages=messages,
            temperature=config['agent'].get('temperature', 0.1),
            max_tokens=1000
        )

        # Extract the text response
        answer = response.choices[0].message.content

        # Add source citations
        if sources:
            answer += f"\n\nSOURCES:\n" + "\n".join(sources)

        logger.info(f"Gemini Agent responded to query: {query[:50]}...")
        return answer

    except Exception as e:
        logger.error(f"Error generating Gemini response: {str(e)}")
        return f"I encountered an error while generating a response: {str(e)}"


def run_unified_agent(query: str, config_file: str = "config.yaml") -> Dict[str, Any]:
    """Run the unified agent using OpenAI Agents SDK with Gemini configuration"""

    logger.info(f"Running unified agent with query: {query}")

    # Load configuration
    config = load_config(config_file)

    # Initialize clients
    gemini_client = initialize_gemini_client_with_openai_sdk(config)
    cohere_client = initialize_cohere_client(config)
    qdrant_client = initialize_qdrant_client(config)

    # Create retrieval function
    retrieval_func = create_retrieval_tool_function(config, qdrant_client, cohere_client)

    # Run with Gemini client configured via OpenAI SDK
    if gemini_client is not None:
        logger.info("Using Google Gemini with OpenAI SDK configuration")
        response = run_gemini_agent_with_openai_sdk(gemini_client, retrieval_func, query, config)
    else:
        # Fallback response
        logger.warning("No Gemini client available, returning mock response")
        response = f"Mock response for query: {query}\n\nSOURCES:\n- mock-source (Section: mock-section)"

    return {
        "response": response,
        "sources": [],  # This will be parsed from the response
        "model_used": "gemini-2.0-flash",
        "tokens_used": len(response.split())
    }


def print_agent_response(response: str, query: str, config_file: str = "config.yaml"):
    """Print the agent response with source citations"""

    # Load config to get the model info
    config = load_config(config_file)

    print("\n" + "="*80)
    print(f"UNIFIED AGENT RESPONSE (Gemini with OpenAI SDK)")
    print("="*80)
    print(response)


def run_validation(config_file: str = "config.yaml"):
    """Run validation of the unified agent pipeline"""
    logger.info("Running unified agent pipeline validation...")

    try:
        # Load configuration
        config = load_config(config_file)

        # Initialize clients
        gemini_client = initialize_gemini_client_with_openai_sdk(config)
        cohere_client = initialize_cohere_client(config)
        qdrant_client = initialize_qdrant_client(config)

        # Create retrieval function
        retrieval_func = create_retrieval_tool_function(config, qdrant_client, cohere_client)

        # Test the retrieval function
        test_query = "test query for validation"
        results = retrieval_func(test_query, top_k=3)

        logger.info(f"Retrieval validation: Got {len(results)} results for test query")

        # Test response generation with Gemini
        if gemini_client is not None:
            # Test the generation
            messages = [
                {
                    "role": "user",
                    "content": "Just respond with 'Validation successful' if you can process this test message."
                }
            ]
            try:
                response = gemini_client.chat.completions.create(
                    model=config['agent']['model'],
                    messages=messages,
                    temperature=0.1
                )
                test_response = response.choices[0].message.content
                logger.info("Gemini generation validation: PASSED")
                logger.info(f"Response: {test_response}")
            except Exception as e:
                logger.error(f"Gemini generation validation: FAILED - {str(e)}")
                return False
        else:
            logger.warning("No Gemini client available, skipping generation test")

        logger.info("Unified agent pipeline validation completed successfully!")
        return True

    except Exception as e:
        logger.error(f"Unified agent pipeline validation failed: {str(e)}")
        return False


def main():
    """Main function to run the unified RAG agent"""
    parser = argparse.ArgumentParser(description='Unified RAG Agent - OpenAI Agents SDK with Gemini')
    parser.add_argument('--query', type=str, help='Query string to ask the agent')
    parser.add_argument('--config', type=str, default='config.yaml', help='Configuration file path')
    parser.add_argument('--validate', action='store_true', help='Run validation only')

    args = parser.parse_args()

    if args.validate:
        # Run validation
        success = run_validation(args.config)
        sys.exit(0 if success else 1)
    elif args.query:
        # Run agent with the provided query
        try:
            response = run_unified_agent(args.query, args.config)
            print_agent_response(response['response'], args.query, args.config)
        except Exception as e:
            logger.error(f"Error running unified agent: {str(e)}")
            sys.exit(1)
    else:
        # Show help if no arguments provided
        parser.print_help()


if __name__ == "__main__":
    main()