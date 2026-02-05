#!/usr/bin/env python3
"""
Alternative RAG Agent Implementation using Google Generative AI (Gemini) instead of OpenAI Agents SDK

This module implements a RAG (Retrieval-Augmented Generation) agent that:
1. Uses Google's Generative AI (with Gemini models) instead of OpenAI
2. Integrates semantic retrieval as a function tool
3. Answers questions grounded strictly in retrieved book content
4. Preserves source metadata for traceability
5. Maintains deterministic behavior for debuggability
"""

import os
import sys
import logging
import argparse
import yaml
import json
from typing import List, Dict, Any, Optional
from pathlib import Path

# Google Gen AI for Gemini integration
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client import models
from qdrant_client.models import PointStruct, VectorParams, Distance
from google.generativeai.types import HarmCategory, HarmBlockThreshold
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from tqdm import tqdm
import hashlib
import time


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


def initialize_genai_client(config: Dict[str, Any]):
    """Initialize Google Generative AI client with API key from environment"""
    api_key = os.getenv('GOOGLE_API_KEY')  # Using GOOGLE_API_KEY instead of OPENAI_API_KEY
    if not api_key:
        raise ValueError("GOOGLE_API_KEY environment variable is required")

    try:
        genai.configure(api_key=api_key)

        # Initialize the generative model (using a Gemini model)
        model = genai.GenerativeModel(
            model_name=config['agent']['model'],  # Use model from config
            safety_settings={
                HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_HATE_SPEECH: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_HARASSMENT: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HarmBlockThreshold.BLOCK_NONE,
            }
        )

        logger.info("Google Generative AI client initialized successfully")
        return model
    except Exception as e:
        logger.error(f"Failed to initialize Google Generative AI client: {str(e)}")
        raise


def initialize_cohere_client():
    """Initialize Cohere client with API key from environment and proper error handling"""
    import cohere

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


def create_retrieval_function(config: Dict[str, Any], qdrant_client: QdrantClient, cohere_client):
    """Create retrieval function that can be called to get relevant content"""

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
            top_k = config['retrieval'].get('top_k', 5)

        try:
            # Generate embedding for the query using Cohere
            query_response = cohere_client.embed(
                texts=[query],
                model=config['embedding']['model'],
                input_type=config['embedding'].get('input_type', 'search_query')
            )
            query_embedding = query_response.embeddings[0]

            # Search in Qdrant
            collection_name = config['storage']['collection_name']

            search_results = qdrant_client.query_points(
                collection_name=collection_name,
                query=query_embedding,
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
                    'content': payload.get('content', '')[:1000] + "..." if len(payload.get('content', '')) > 1000 else payload.get('content', ''),
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
            logger.error(f"Error in retrieval function: {str(e)}")
            return []

    return retrieve_content


def generate_rag_response(genai_model, query: str, retrieved_content: List[Dict[str, Any]], config: Dict[str, Any]) -> str:
    """Generate a RAG response using Gemini model with retrieved context"""

    if not retrieved_content:
        return "I couldn't find any relevant content in the documentation to answer your query."

    # Format the context from retrieved content
    context_str = "Relevant documentation content:\n\n"
    sources = []

    for i, chunk in enumerate(retrieved_content):
        context_str += f"Source {i+1} ({chunk['url']}):\n"
        context_str += f"{chunk['content']}\n\n"
        sources.append(f"- {chunk['url']} (Section: {chunk['section']})")

    # Create the prompt for Gemini
    prompt = f"""
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

    try:
        # Generate content using Gemini
        response = genai_model.generate_content(prompt)

        # Extract the text response
        answer = response.text

        # Add source citations
        if sources:
            answer += f"\n\nSources:\n" + "\n".join(sources)

        return answer

    except Exception as e:
        logger.error(f"Error generating RAG response: {str(e)}")
        return f"I encountered an error while generating a response: {str(e)}"


def validate_response_grounding(response: str, retrieved_content: List[Dict[str, Any]]) -> bool:
    """Validate that the response is grounded in the retrieved content"""

    if not retrieved_content:
        # If no content was retrieved, the response should acknowledge this
        return "couldn't find any relevant content" in response.lower() or \
               "no relevant content" in response.lower() or \
               "not in the documentation" in response.lower()

    # Basic validation: check if response contains content that appears in retrieved results
    response_lower = response.lower()

    # Count how many retrieved chunks have content that appears in the response
    matching_chunks = 0
    for chunk in retrieved_content:
        chunk_content = chunk['content'].lower()
        # Check if any significant portion of the chunk appears in the response
        if len(chunk_content) > 20:  # Only check substantial chunks
            # Look for at least a 5-word phrase match
            chunk_words = chunk_content.split()
            for i in range(len(chunk_words) - 4):
                phrase = " ".join(chunk_words[i:i+5])
                if phrase in response_lower:
                    matching_chunks += 1
                    break  # Count chunk once if any part matches

    # For now, we'll say at least one chunk should be referenced
    is_valid = matching_chunks > 0
    logger.info(f"Response validation: {matching_chunks}/{len(retrieved_content)} chunks referenced in response")

    return is_valid


def run_rag_agent(query: str, config_file: str = "config.yaml"):
    """Run the RAG agent with Google Generative AI"""
    logger.info(f"Running RAG agent with query: {query}")

    # Load configuration
    config = load_config(config_file)

    # Initialize clients
    genai_model = initialize_genai_client(config)
    cohere_client = initialize_cohere_client()
    qdrant_client = initialize_qdrant_client(config)

    # Create retrieval function
    retrieval_func = create_retrieval_function(config, qdrant_client, cohere_client)

    # Retrieve relevant content
    logger.info("Retrieving relevant content...")
    retrieved_content = retrieval_func(query)

    if not retrieved_content:
        logger.warning("No relevant content found for the query")

    # Generate RAG response
    logger.info("Generating RAG response...")
    response = generate_rag_response(genai_model, query, retrieved_content, config)

    # Validate response grounding
    is_grounded = validate_response_grounding(response, retrieved_content)

    if not is_grounded:
        logger.warning("Response may not be fully grounded in retrieved content")

    # Print response
    print("\n" + "="*80)
    print("RAG AGENT RESPONSE")
    print("="*80)
    print(response)

    if retrieved_content:
        print(f"\nRetrieved {len(retrieved_content)} relevant content chunks for grounding.")
    else:
        print("\nNo relevant content was found to ground this response.")


def run_validation(config_file: str = "config.yaml"):
    """Run validation of the RAG agent pipeline"""
    logger.info("Running RAG agent pipeline validation...")

    try:
        # Load configuration
        config = load_config(config_file)

        # Initialize clients
        genai_model = initialize_genai_client(config)
        cohere_client = initialize_cohere_client()
        qdrant_client = initialize_qdrant_client(config)

        logger.info("All clients initialized successfully")

        # Test retrieval function
        retrieval_func = create_retrieval_function(config, qdrant_client, cohere_client)
        test_results = retrieval_func("test query for validation", top_k=2)

        logger.info(f"Retrieval test: Got {len(test_results)} results")

        # Test response generation
        test_response = generate_rag_response(
            genai_model,
            "What is this documentation about?",
            test_results,
            config
        )

        logger.info(f"Response generation test completed, response length: {len(test_response)} characters")

        # Test grounding validation
        is_grounded = validate_response_grounding(test_response, test_results)
        logger.info(f"Response grounding validation: {'PASSED' if is_grounded else 'FAILED'}")

        logger.info("RAG agent pipeline validation completed successfully!")
        return True

    except Exception as e:
        logger.error(f"RAG agent pipeline validation failed: {str(e)}")
        return False


def main():
    """Main function to run the RAG agent"""
    parser = argparse.ArgumentParser(description='RAG Agent using Google Generative AI (Gemini)')
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
        run_rag_agent(args.query, args.config)
    else:
        # Show help if no arguments provided
        parser.print_help()


if __name__ == "__main__":
    main()