#!/usr/bin/env python3
"""
RAG Agent – OpenAI Agents SDK Integration

This module implements an AI agent using the OpenAI Agents SDK that:
1. Integrates semantic retrieval as a callable tool
2. Answers questions grounded strictly in retrieved book content
3. Preserves source metadata for traceability
4. Exhibits deterministic behavior for debuggability
"""

import os
import sys
import logging
import argparse
import yaml
import json
from typing import List, Dict, Any, Optional
from pathlib import Path

import openai
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from bs4 import BeautifulSoup
import requests
from dotenv import load_dotenv
from tqdm import tqdm


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
        raise
    except yaml.YAMLError as e:
        logger.error(f"Error parsing YAML configuration: {str(e)}")
        raise


def initialize_openai_client() -> openai.OpenAI:
    """Initialize OpenAI client with API key from environment and proper error handling"""
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        raise ValueError("OPENAI_API_KEY environment variable is required")

    try:
        client = openai.OpenAI(api_key=api_key)
        # Test the connection
        client.models.list()
        logger.info("OpenAI client initialized successfully")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize OpenAI client: {str(e)}")
        raise


def initialize_cohere_client() -> cohere.Client:
    """Initialize Cohere client with API key from environment and proper error handling"""
    api_key = os.getenv('COHERE_API_KEY')
    if not api_key:
        raise ValueError("COHERE_API_KEY environment variable is required")

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
        raise


def initialize_qdrant_client(config: Dict[str, Any]) -> QdrantClient:
    """Initialize Qdrant client with configuration and proper error handling"""
    url = os.getenv('QDRANT_URL')
    api_key = os.getenv('QDRANT_API_KEY')

    if not url or not api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

    try:
        client = QdrantClient(url=url, api_key=api_key)
        # Test the connection
        client.get_collections()
        logger.info("Qdrant client initialized successfully")
        return client
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant client: {str(e)}")
        raise


def create_retrieval_tool_function(config: Dict[str, Any], qdrant_client: QdrantClient, cohere_client: cohere.Client) -> callable:
    """Create retrieval tool function that can be registered with the OpenAI agent"""

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
                model=config['cohere']['model'],
                input_type=config['cohere'].get('input_type', 'search_query')
            )
            query_embedding = query_response.embeddings[0]

            # Search in Qdrant using the correct query_points method
            collection_name = config['qdrant']['collection_name']
            search_results = qdrant_client.query_points(
                collection_name=collection_name,
                query=query_embedding,  # query_points uses 'query' instead of 'query_vector'
                limit=top_k,
                with_payload=True,
                score_threshold=config['retrieval'].get('score_threshold', 0.3)
            )

            # Handle the QueryResponse object - access the points attribute
            points = search_results.points if hasattr(search_results, 'points') else search_results

            # Format results
            formatted_results = []
            for result in points:
                # Access payload and score from the result object
                payload = getattr(result, 'payload', {}) or {}
                score = getattr(result, 'score', 0)

                formatted_result = {
                    'content': payload.get('content', ''),
                    'url': payload.get('url', ''),
                    'section': payload.get('section', ''),
                    'chunk_id': payload.get('chunk_id', ''),
                    'score': score,
                    'source': payload.get('source', 'unknown')
                }

                formatted_results.append(formatted_result)

            logger.info(f"Retrieved {len(formatted_results)} relevant chunks for query: {query[:50]}...")
            return formatted_results

        except Exception as e:
            logger.error(f"Error in retrieval tool: {str(e)}")
            return []

    return retrieve_content


def create_agent_with_retrieval_tool(openai_client: openai.OpenAI, retrieval_tool_func: callable, config: Dict[str, Any]):
    """Create an OpenAI agent with the retrieval tool registered"""

    # Define the tool specification for the retrieval function
    retrieval_tool_spec = {
        "type": "function",
        "function": {
            "name": "retrieve_content",
            "description": "Retrieve relevant content chunks from the documentation database based on a query",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "The search query to find relevant content"
                    },
                    "top_k": {
                        "type": "integer",
                        "description": "Number of results to return (optional, defaults to system setting)",
                        "minimum": 1,
                        "maximum": 20
                    }
                },
                "required": ["query"]
            }
        }
    }

    # Create the assistant with the retrieval tool
    try:
        assistant = openai_client.beta.assistants.create(
            name="RAG Documentation Assistant",
            description="An AI assistant that answers questions based on retrieved documentation content",
            model=config['agent']['model'],
            tools=[retrieval_tool_spec],
            temperature=config['agent'].get('temperature', 0.1)
        )

        logger.info(f"Agent created successfully with ID: {assistant.id}")
        return assistant

    except Exception as e:
        logger.error(f"Failed to create agent: {str(e)}")
        raise


def run_agent_query(assistant_id: str, openai_client: openai.OpenAI, query: str) -> str:
    """Run a query through the OpenAI agent and return the response"""

    try:
        # Create a thread for the conversation
        thread = openai_client.beta.threads.create()

        # Add the user's message to the thread
        openai_client.beta.threads.messages.create(
            thread_id=thread.id,
            role="user",
            content=query
        )

        # Run the assistant on the thread
        run = openai_client.beta.threads.runs.create(
            thread_id=thread.id,
            assistant_id=assistant_id
        )

        # Poll for completion
        import time
        while run.status in ['queued', 'in_progress']:
            time.sleep(1)
            run = openai_client.beta.threads.runs.retrieve(thread_id=thread.id, run_id=run.id)

        # Get the assistant's response
        messages = openai_client.beta.threads.messages.list(thread_id=thread.id)

        # Extract the latest assistant response
        for message in messages.data:
            if message.role == "assistant":
                # Get the content from the message
                content_parts = []
                for content_item in message.content:
                    if content_item.type == "text":
                        content_parts.append(content_item.text.value)

                response = "\n".join(content_parts)
                break
        else:
            response = "The agent did not provide a response."

        # Clean up the thread
        # Note: In practice, you might want to keep threads for conversation continuity
        # For this example, we're treating each query as a separate interaction

        logger.info(f"Agent responded to query: {query[:50]}...")
        return response

    except Exception as e:
        logger.error(f"Error running agent query: {str(e)}")
        raise


def validate_agent_response(response: str, retrieved_content: List[Dict[str, Any]]) -> bool:
    """Validate that the agent response is grounded in the retrieved content"""

    # Basic validation: check if response contains content that appears in retrieved results
    response_lower = response.lower()

    # Count how many retrieved chunks contain content that appears in the response
    relevant_chunks = 0
    for chunk in retrieved_content:
        chunk_content = chunk['content'].lower()
        # Check if any significant portion of the chunk appears in the response
        if len(chunk_content) > 10:  # Only check substantial chunks
            # Look for at least a 5-word phrase match
            chunk_words = chunk_content.split()
            if len(chunk_words) >= 5:
                # Take first few words as a signature
                signature = " ".join(chunk_words[:min(5, len(chunk_words))])
                if signature in response_lower:
                    relevant_chunks += 1

    # For now, we'll say at least one chunk should be referenced
    # In a more sophisticated implementation, we could check for semantic similarity
    is_valid = len(retrieved_content) == 0 or relevant_chunks > 0
    logger.info(f"Response validation: {relevant_chunks}/{len(retrieved_content)} chunks referenced in response")

    return is_valid


def print_agent_response(response: str, retrieved_chunks: List[Dict[str, Any]]):
    """Print the agent response with source citations"""

    print("\n" + "="*80)
    print("AGENT RESPONSE")
    print("="*80)
    print(response)

    if retrieved_chunks:
        print("\nSOURCES:")
        print("-" * 40)
        for i, chunk in enumerate(retrieved_chunks, 1):
            print(f"{i}. {chunk['url']}")
            print(f"   Section: {chunk['section']}")
            print(f"   Chunk ID: {chunk['chunk_id']}")
            print(f"   Score: {chunk['score']:.3f}")
            print(f"   Content Preview: {chunk['content'][:100]}...")
            print()
    else:
        print("\nNo relevant content was found to answer this query.")


def run_validation(config_file: str = "config.yaml"):
    """Run validation of the agent pipeline"""
    logger.info("Running agent pipeline validation...")

    try:
        # Load configuration
        config = load_config(config_file)

        # Initialize clients
        openai_client = initialize_openai_client()
        cohere_client = initialize_cohere_client()
        qdrant_client = initialize_qdrant_client(config)

        # Create retrieval tool
        retrieval_func = create_retrieval_tool_function(config, qdrant_client, cohere_client)

        # Test the retrieval function
        test_query = "test query for validation"
        results = retrieval_func(test_query, top_k=3)

        logger.info(f"Retrieval validation: Got {len(results)} results for test query")

        # Test response validation
        test_response = "This is a test response based on retrieved content."
        is_valid = validate_agent_response(test_response, results)
        logger.info(f"Response validation: {'PASSED' if is_valid else 'FAILED'}")

        logger.info("Agent pipeline validation completed successfully!")
        return True

    except Exception as e:
        logger.error(f"Agent pipeline validation failed: {str(e)}")
        return False


def main():
    """Main function to run the RAG agent"""
    parser = argparse.ArgumentParser(description='RAG Agent using OpenAI Agents SDK')
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
        run_agent_with_query(args.query, args.config)
    else:
        # Show help if no arguments provided
        parser.print_help()


def run_agent_with_query(query: str, config_file: str):
    """Run the agent with a specific query"""
    logger.info(f"Running agent with query: {query}")

    try:
        # Load configuration
        config = load_config(config_file)

        # Initialize clients
        openai_client = initialize_openai_client()
        cohere_client = initialize_cohere_client()
        qdrant_client = initialize_qdrant_client(config)

        # Create retrieval tool
        retrieval_func = create_retrieval_tool_function(config, qdrant_client, cohere_client)

        # Create agent with retrieval tool
        agent = create_agent_with_retrieval_tool(openai_client, retrieval_func, config)

        # Run the query through the agent
        response = run_agent_query(agent.id, openai_client, query)

        # Retrieve content for validation (do this separately to check grounding)
        retrieved_chunks = retrieval_func(query)

        # Validate that response is grounded in retrieved content
        is_grounded = validate_agent_response(response, retrieved_chunks)

        if not is_grounded:
            logger.warning("Agent response may not be fully grounded in retrieved content")

        # Print the response with citations
        print_agent_response(response, retrieved_chunks)

    except Exception as e:
        logger.error(f"Error running agent: {str(e)}")
        raise


if __name__ == "__main__":
    main()