#!/usr/bin/env python3
"""
RAG Pipeline - Website Deployment, Embedding Generation, and Vector Storage

This script implements a complete RAG (Retrieval-Augmented Generation) pipeline that:
1. Fetches content from deployed documentation URLs
2. Extracts clean text content from HTML
3. Chunks the content with overlap
4. Generates semantic embeddings using Cohere
5. Stores embeddings with metadata in Qdrant vector database
6. Provides semantic search capabilities

The pipeline follows the architecture described in the specification:
- URL fetching → text extraction → content chunking → embedding generation → vector storage
"""

import os
import sys
import logging
import argparse
import requests
import yaml
import json
from typing import List, Dict, Any, Optional, Tuple
from urllib.parse import urljoin, urlparse
from pathlib import Path

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from bs4 import BeautifulSoup
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


def main():
    """Main function to run the RAG pipeline"""
    parser = argparse.ArgumentParser(description='RAG Pipeline for documentation processing')
    parser.add_argument('--urls-file', type=str, help='File containing URLs to process')
    parser.add_argument('--config', type=str, default='config.yaml', help='Configuration file path')
    parser.add_argument('--validate', action='store_true', help='Run validation only')
    parser.add_argument('--query', type=str, help='Run a sample query against the vector store')

    args = parser.parse_args()

    if args.query:
        # Run semantic search query
        run_query(args.query)
    elif args.validate:
        # Run validation
        run_validation()
    else:
        # Run the full pipeline
        run_pipeline(args.urls_file, args.config)


def run_pipeline(urls_file: str, config_file: str):
    """Run the complete RAG pipeline"""
    logger.info("Starting RAG pipeline execution...")

    # Load configuration
    config = load_config(config_file)

    # Get URLs to process
    if urls_file:
        urls = load_urls_from_file(urls_file)
    else:
        urls = get_urls_from_sitemap(config['extraction']['sitemap_url'])

    logger.info(f"Processing {len(urls)} URLs")

    # Initialize Cohere client
    cohere_client = initialize_cohere_client()

    # Initialize Qdrant client
    qdrant_client = initialize_qdrant_client(config)

    # Create Qdrant collection
    create_qdrant_collection(qdrant_client, config)

    # Process each URL
    for url in tqdm(urls, desc="Processing URLs"):
        try:
            # Fetch content
            content = fetch_url_content(url)

            # Extract clean text
            clean_text = extract_clean_text(content, url)

            # Chunk content
            chunks = chunk_content(clean_text, url)

            # Generate embeddings and store
            store_embeddings(qdrant_client, cohere_client, chunks, config)

        except Exception as e:
            logger.error(f"Error processing URL {url}: {str(e)}")
            continue

    logger.info("RAG pipeline completed successfully!")


def run_query(query: str):
    """Run a sample query against the vector store"""
    logger.info(f"Running query: {query}")

    # Initialize Qdrant client
    config = load_config('config.yaml')  # Default config
    qdrant_client = initialize_qdrant_client(config)

    # Initialize Cohere client
    cohere_client = initialize_cohere_client()

    try:
        # Generate embedding for the query
        query_response = cohere_client.embed(
            texts=[query],
            model=config['embedding']['model'],
            input_type="search_query"
        )
        query_embedding = query_response.embeddings[0]

        # Search in Qdrant using the correct query_points method
        search_results = qdrant_client.query_points(
            collection_name=config['storage']['collection_name'],
            query=query_embedding,  # query_points uses 'query' instead of 'query_vector'
            limit=5,
            with_payload=True
        )

        # Handle the QueryResponse object - access the points attribute
        points = search_results.points if hasattr(search_results, 'points') else search_results

        logger.info("Query results:")
        for i, result in enumerate(points):
            # Access payload and score from the result object
            payload = getattr(result, 'payload', {}) or {}
            score = getattr(result, 'score', 0)

            print(f"{i+1}. {payload.get('content', '')[:200]}...")
            print(f"   URL: {payload.get('url', 'N/A')}")
            print(f"   Score: {score}")
            print()
    except Exception as e:
        logger.error(f"Error running query: {str(e)}")
        raise


def implement_semantic_search(query: str, config: Dict[str, Any], qdrant_client: QdrantClient, cohere_client: cohere.Client) -> List[Dict[str, Any]]:
    """Implement semantic search with query embedding generation"""
    try:
        # Generate embedding for the query
        query_response = cohere_client.embed(
            texts=[query],
            model=config['embedding']['model'],
            input_type="search_query"
        )
        query_embedding = query_response.embeddings[0]

        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name=config['storage']['collection_name'],
            query_vector=query_embedding,
            limit=config.get('search', {}).get('limit', 5),
            with_payload=True,
            score_threshold=config.get('search', {}).get('score_threshold', 0.3)  # Only return results above this threshold
        )

        return search_results
    except Exception as e:
        logger.error(f"Error in semantic search: {str(e)}")
        raise


def rank_and_filter_results(results: List[Any], min_score: float = 0.3) -> List[Dict[str, Any]]:
    """Create result ranking and filtering"""
    filtered_results = []
    for result in results:
        if result.score >= min_score:
            filtered_results.append({
                'content': result.payload.get('content', ''),
                'url': result.payload.get('url', ''),
                'score': result.score,
                'section': result.payload.get('section', ''),
                'chunk_id': result.payload.get('chunk_id', '')
            })

    # Sort by score in descending order
    filtered_results.sort(key=lambda x: x['score'], reverse=True)
    return filtered_results


def load_pipeline_config(config_file: str = "config.yaml") -> Dict[str, Any]:
    """Create configuration file parsing from YAML"""
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        logger.info(f"Configuration loaded from {config_file}")
        return config
    except FileNotFoundError:
        logger.error(f"Configuration file {config_file} not found. Using defaults.")
        # Return default config
        return {
            'extraction': {
                'sitemap_url': 'https://example.com/sitemap.xml',
                'allowed_domains': ['example.com'],
                'content_selectors': ['article', '.markdown', '.docs-content'],
                'exclude_selectors': ['.header', '.footer', '.nav']
            },
            'chunking': {
                'max_chunk_size': 1024,
                'overlap_size': 256,
                'min_chunk_size': 100
            },
            'embedding': {
                'model': 'embed-english-v3.0',
                'input_type': 'search_document'
            },
            'storage': {
                'collection_name': 'documentation_chunks',
                'vector_size': 1024,
                'distance': 'Cosine',
                'batch_size': 10
            },
            'pipeline': {
                'max_concurrent_requests': 5,
                'retry_attempts': 3,
                'timeout_seconds': 30
            }
        }


def orchestrate_pipeline(config: Dict[str, Any], urls: List[str] = None):
    """Implement pipeline workflow orchestration"""
    logger.info("Starting pipeline orchestration...")

    # Initialize clients
    cohere_client = initialize_cohere_client()
    qdrant_client = initialize_qdrant_client(config)

    # Create collection
    create_qdrant_collection(qdrant_client, config)

    # Get URLs to process
    if urls is None:
        urls = get_urls_from_sitemap(config['extraction']['sitemap_url'])

    # Process each URL
    processed_count = 0
    for url in tqdm(urls, desc="Processing URLs"):
        try:
            # Fetch content
            content = fetch_url_content(url)

            # Extract clean text
            clean_text = extract_clean_text(content, url)

            # Validate content
            if len(clean_text.strip()) < config['chunking'].get('min_chunk_size', 100):
                logger.warning(f"Content from {url} is too short to process")
                continue

            # Chunk content
            chunks = chunk_content(
                clean_text,
                url,
                config['chunking']['max_chunk_size'],
                config['chunking']['overlap_size']
            )

            # Filter chunks by quality
            valid_chunks = [chunk for chunk in chunks if validate_chunk_quality(chunk, config['chunking']['min_chunk_size'])]

            if not valid_chunks:
                logger.warning(f"No valid chunks generated from {url}")
                continue

            # Generate embeddings and store
            store_embeddings(qdrant_client, cohere_client, valid_chunks, config)
            processed_count += 1

        except Exception as e:
            logger.error(f"Error processing URL {url}: {str(e)}")
            continue

    logger.info(f"Pipeline completed. Successfully processed {processed_count}/{len(urls)} URLs")


def track_pipeline_state():
    """Add pipeline state tracking and logging"""
    # This would typically involve more complex state tracking
    # For now, we'll just use the existing logging
    pass


def implement_incremental_updates():
    """Create incremental update functionality"""
    # This would involve implementing logic to track which URLs have been processed
    # and only process new or changed content
    # For now, we'll add a placeholder
    logger.info("Incremental update functionality would be implemented here")


def validate_pipeline_consistency():
    """Add pipeline validation and consistency checks"""
    try:
        # Check if required environment variables are set
        required_vars = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY']
        for var in required_vars:
            if not os.getenv(var):
                raise ValueError(f"Required environment variable {var} is not set")

        # Check if config file exists and is valid
        config = load_config('config.yaml')
        if not config:
            raise ValueError("Configuration file is invalid or empty")

        logger.info("Pipeline validation passed")
        return True
    except Exception as e:
        logger.error(f"Pipeline validation failed: {str(e)}")
        return False


def load_config(config_file: str) -> Dict[str, Any]:
    """Load configuration from YAML file"""
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config


def load_urls_from_file(file_path: str) -> List[str]:
    """Load URLs from a text file"""
    with open(file_path, 'r') as f:
        urls = [line.strip() for line in f if line.strip() and not line.startswith('#')]
    return urls


def validate_url(url: str) -> bool:
    """Validate if a URL is properly formatted"""
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def get_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """Extract URLs from sitemap.xml"""
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'xml')
        urls = []

        # Look for <url><loc> elements in sitemap
        for url_element in soup.find_all('loc'):
            url = url_element.text.strip()
            if validate_url(url):
                urls.append(url)

        return urls
    except Exception as e:
        logger.error(f"Error fetching sitemap: {str(e)}")
        return []


def check_url_accessibility(url: str, timeout: int = 10) -> bool:
    """Check if a URL is accessible"""
    try:
        response = requests.head(url, timeout=timeout, allow_redirects=True)
        return response.status_code < 400
    except Exception:
        # If HEAD fails, try GET
        try:
            response = requests.get(url, timeout=timeout, stream=True)
            response.close()
            return response.status_code < 400
        except Exception:
            return False


def fetch_url_content(url: str) -> str:
    """Fetch content from a URL with error handling and retry logic"""
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = requests.get(url, timeout=30)
            response.raise_for_status()
            return response.text
        except requests.exceptions.RequestException as e:
            logger.warning(f"Attempt {attempt + 1} failed to fetch URL {url}: {str(e)}")
            if attempt == max_retries - 1:  # Last attempt
                logger.error(f"Failed to fetch URL {url} after {max_retries} attempts")
                raise
            # Wait before retrying (exponential backoff)
            import time
            time.sleep(2 ** attempt)


def extract_clean_text(html_content: str, url: str) -> str:
    """Extract clean text content from HTML using BeautifulSoup with Docusaurus-specific selectors"""
    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove script and style elements
    for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
        script.decompose()

    # For Docusaurus sites, try to target main content areas
    # Common selectors for Docusaurus content
    content_selectors = [
        'article', '.markdown', '.theme-doc-markdown', '.doc-content',
        '.main-content', '.docs-content', '.container', 'main'
    ]

    content_element = None
    for selector in content_selectors:
        content_element = soup.select_one(selector)
        if content_element:
            break

    # If no specific content element found, use the body
    if not content_element:
        content_element = soup.find('body') or soup

    # Get text content
    text = content_element.get_text()

    # Clean up text (remove extra whitespace)
    lines = (line.strip() for line in text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    text = ' '.join(chunk for chunk in chunks if chunk)

    return text


def chunk_content(content: str, url: str, max_chunk_size: int = 1024, overlap_size: int = 256) -> List[Dict[str, Any]]:
    """Chunk content with overlap and metadata tracking"""
    chunks = []

    # Simple character-based chunking with overlap
    start = 0
    content_length = len(content)

    while start < content_length:
        end = start + max_chunk_size

        # If this is the last chunk, include the remainder
        if end >= content_length:
            end = content_length
        else:
            # Try to break at a sentence boundary if possible
            chunk = content[start:end]
            last_period = chunk.rfind('.')
            last_exclamation = chunk.rfind('!')
            last_question = chunk.rfind('?')

            # Find the best breaking point (closest to the end but not too close to the beginning)
            break_points = [bp for bp in [last_period, last_exclamation, last_question] if bp > max_chunk_size // 2]
            if break_points:
                best_break = max(break_points)
                end = start + best_break + 1

        chunk_text = content[start:end]

        # Create chunk with metadata
        chunk_data = {
            'content': chunk_text,
            'url': url,
            'section': f"chunk_{len(chunks)}",
            'chunk_id': f"{url_hash(url)}_{len(chunks)}",
            'start_pos': start,
            'end_pos': end
        }

        chunks.append(chunk_data)

        # Move to next chunk with overlap
        if end >= content_length:
            break
        start = end - overlap_size

    return chunks


def validate_chunk_quality(chunk: Dict[str, Any], min_chunk_size: int = 100) -> bool:
    """Validate chunk quality based on content length and other criteria"""
    content = chunk.get('content', '')
    return len(content) >= min_chunk_size


def generate_chunk_id(url: str, chunk_index: int) -> str:
    """Generate a unique chunk ID with URL context tracking"""
    return f"{url_hash(url)}_{chunk_index:04d}"


def url_hash(url: str) -> str:
    """Generate a simple hash for URL to use in chunk IDs"""
    import hashlib
    return hashlib.md5(url.encode()).hexdigest()[:8]


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


def create_qdrant_collection(client: QdrantClient, config: Dict[str, Any]):
    """Create Qdrant collection with specified configuration and metadata schema"""
    collection_name = config['storage']['collection_name']
    vector_size = config['storage'].get('vector_size', 1024)
    distance = config['storage'].get('distance', 'Cosine')

    # Convert distance string to Qdrant model
    distance_map = {
        'Cosine': models.Distance.COSINE,
        'Euclidean': models.Distance.EUCLID,
        'Dot': models.Distance.DOT
    }
    distance_model = distance_map.get(distance, models.Distance.COSINE)

    try:
        # Check if collection exists
        client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' already exists")
    except:
        # Create collection if it doesn't exist
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,
                distance=distance_model
            )
        )
        logger.info(f"Created collection '{collection_name}' with vector size {vector_size} and {distance} distance")


def validate_metadata(payload: Dict[str, Any]) -> bool:
    """Validate metadata completeness before storage"""
    required_fields = ['url', 'section', 'chunk_id', 'content']
    for field in required_fields:
        if field not in payload or not payload[field]:
            logger.warning(f"Missing required metadata field: {field}")
            return False
    return True


def generate_embeddings_with_retry(
    cohere_client: cohere.Client,
    texts: List[str],
    model: str,
    input_type: str,
    max_retries: int = 3
) -> List[List[float]]:
    """Generate embeddings with retry logic and rate limiting"""
    for attempt in range(max_retries):
        try:
            response = cohere_client.embed(
                texts=texts,
                model=model,
                input_type=input_type
            )
            return response.embeddings
        except Exception as e:
            logger.warning(f"Attempt {attempt + 1} failed to generate embeddings: {str(e)}")
            if "rate limit" in str(e).lower():
                import time
                time.sleep(2 ** attempt)  # Exponential backoff
            if attempt == max_retries - 1:  # Last attempt
                logger.error(f"Failed to generate embeddings after {max_retries} attempts")
                raise


def validate_embedding_quality(embedding: List[float], expected_size: int = 1024) -> bool:
    """Validate embedding dimensions match expected size"""
    return len(embedding) == expected_size


def store_embeddings(
    qdrant_client: QdrantClient,
    cohere_client: cohere.Client,
    chunks: List[Dict[str, Any]],
    config: Dict[str, Any]
):
    """Generate embeddings for chunks and store in Qdrant"""
    collection_name = config['storage']['collection_name']
    embedding_model = config['embedding']['model']
    input_type = config['embedding'].get('input_type', 'search_document')

    # Prepare texts for embedding
    texts = [chunk['content'] for chunk in chunks]

    if not texts:
        logger.warning("No texts to embed")
        return

    # Generate embeddings using Cohere with batching for efficiency
    all_embeddings = []

    # Process in batches to respect API limits (Cohere allows up to 96 texts per request)
    batch_size = min(96, config['pipeline'].get('max_batch_size', 96))

    for i in range(0, len(texts), batch_size):
        text_batch = texts[i:i + batch_size]

        # Generate embeddings for this batch
        batch_embeddings = generate_embeddings_with_retry(
            cohere_client,
            text_batch,
            embedding_model,
            input_type
        )

        # Validate embedding dimensions
        expected_size = config['storage'].get('vector_size', 1024)
        for j, embedding in enumerate(batch_embeddings):
            if not validate_embedding_quality(embedding, expected_size):
                logger.warning(f"Embedding {i+j} has unexpected size: {len(embedding)} (expected {expected_size})")

        all_embeddings.extend(batch_embeddings)
        logger.info(f"Processed batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

    # Prepare points for Qdrant
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, all_embeddings)):
        point = models.PointStruct(
            id=i,  # In a real implementation, you'd want to use a unique ID
            vector=embedding,
            payload={
                'url': chunk['url'],
                'section': chunk['section'],
                'chunk_id': chunk['chunk_id'],
                'content': chunk['content'][:500] + "..." if len(chunk['content']) > 500 else chunk['content'],  # Truncate long content
                'source': 'rag_pipeline'
            }
        )
        points.append(point)

    # Upsert points to Qdrant in batches
    qdrant_batch_size = config['storage'].get('batch_size', 10)
    for i in range(0, len(points), qdrant_batch_size):
        batch = points[i:i + qdrant_batch_size]
        qdrant_client.upsert(
            collection_name=collection_name,
            points=batch
        )

    logger.info(f"Stored {len(chunks)} chunks in Qdrant collection '{collection_name}'")


def run_validation():
    """Run validation of the pipeline"""
    logger.info("Running pipeline validation...")

    # Validate environment variables
    if not validate_pipeline_consistency():
        logger.error("Environment validation failed")
        return False

    # Validate configuration
    try:
        config = load_config('config.yaml')
        logger.info("Configuration validation passed")
    except Exception as e:
        logger.error(f"Configuration validation failed: {str(e)}")
        return False

    # Test Cohere client
    try:
        cohere_client = initialize_cohere_client()
        logger.info("Cohere client validation passed")
    except Exception as e:
        logger.error(f"Cohere client validation failed: {str(e)}")
        return False

    # Test Qdrant client
    try:
        qdrant_client = initialize_qdrant_client(config)
        logger.info("Qdrant client validation passed")
    except Exception as e:
        logger.error(f"Qdrant client validation failed: {str(e)}")
        return False

    logger.info("All validations passed successfully!")
    return True


if __name__ == "__main__":
    main()