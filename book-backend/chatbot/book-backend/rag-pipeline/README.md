# RAG Pipeline Retrieval and Validation

This module provides retrieval and validation functionality for the RAG (Retrieval-Augmented Generation) pipeline. It allows querying the vector database using semantic embeddings and validates the retrieval accuracy, relevance, and consistency.

## Features

- Semantic search using Cohere embeddings
- Retrieval from Qdrant vector database
- Metadata validation (URL, section, chunk ID)
- Performance validation (latency, throughput)
- Accuracy and relevance metrics
- Determinism validation (consistency across runs)

## Setup

1. Install dependencies:
   ```bash
   pip install cohere qdrant-client python-dotenv pyyaml
   ```

2. Configure environment variables in `.env`:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=documentation_chunks
   ```

3. Create a configuration file (`config.yaml`) with retrieval and validation parameters.

## Usage

### Basic Retrieval

```bash
python retrieve.py --query "your search query here"
```

### With Custom Configuration

```bash
python retrieve.py --query "your search query" --config custom_config.yaml
```

### With Custom Top-K Value

```bash
python retrieve.py --query "your search query" --top-k 10
```

### Run Validation Only

```bash
python retrieve.py --validate --config config.yaml
```

## Configuration

The system uses a `config.yaml` file with the following structure:

```yaml
retrieval:
  top_k: 5                    # Number of results to retrieve
  score_threshold: 0.3        # Minimum similarity score threshold
  timeout_seconds: 30         # Request timeout

validation:
  accuracy_threshold: 0.90    # Minimum accuracy rate
  latency_threshold_ms: 500   # Maximum response time in ms
  consistency_test_runs: 100  # Number of runs for consistency validation

cohere:
  model: "embed-english-v3.0" # Cohere embedding model
  input_type: "search_query"  # Cohere input type

qdrant:
  collection_name: "documentation_chunks" # Qdrant collection name
  vector_size: 1024                       # Vector dimension
  distance: "Cosine"                      # Distance metric
```

## Validation Metrics

The system validates:

1. **Performance**: Response time against configured thresholds
2. **Accuracy**: Proportion of results with scores above threshold
3. **Relevance**: Semantic similarity of retrieved content to query
4. **Consistency**: Whether identical queries produce identical results

## Architecture

The retrieval process follows these steps:

1. Query preprocessing and embedding generation using Cohere
2. Vector similarity search in Qdrant
3. Result validation and metadata extraction
4. Performance and accuracy validation
5. Formatted output with validation report

## Error Handling

- Graceful handling of network timeouts and API errors
- Validation of required metadata fields
- Logging of warnings for missing or inconsistent data
- Fallback behaviors for common failure modes

## Security

- API keys are loaded from environment variables only
- No sensitive information is logged
- Configuration validation to prevent invalid setups