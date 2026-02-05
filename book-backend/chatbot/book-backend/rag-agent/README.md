# Alternative RAG Agent - Google Generative AI Implementation

This directory contains an alternative implementation of the RAG agent using Google's Generative AI (Gemini) instead of OpenAI's Agent SDK, designed for users who prefer not to use OpenAI's paid API.

## Features

- **Google Generative AI Integration**: Uses Google's Gemini models instead of OpenAI
- **Semantic Retrieval**: Integrates with existing Qdrant + Cohere pipeline
- **Grounded Responses**: Generates responses strictly from retrieved content
- **Source Tracking**: Preserves and cites source metadata for traceability
- **Configurable**: Highly configurable through YAML configuration files
- **Robust Error Handling**: Includes retry logic, rate limiting, and comprehensive error handling

## Prerequisites

- Python 3.9+
- Google API key for Gemini access
- Cohere API key
- Qdrant Cloud account and API key
- Access to the existing Qdrant collection with documentation embeddings

## Setup

1. **Install dependencies**:
   ```bash
   cd book-backend/rag-agent
   pip install -r requirements.txt
   ```

2. **Configure environment variables in `.env`**:
   ```env
   GOOGLE_API_KEY=your_google_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=documentation_chunks
   ```

3. **Update the configuration in `config.yaml` as needed**.

## Usage

### Run the agent with a query:

```bash
python alternative_agent.py --query "What does the documentation say about API usage?"
```

### Run validation:

```bash
python alternative_agent.py --validate
```

### Run with custom configuration:

```bash
python alternative_agent.py --query "Your question here" --config custom_config.yaml
```

## Configuration

The system uses a `config.yaml` file with the following structure:

```yaml
agent:
  model: "gemini-pro"          # Google Gemini model to use
  temperature: 0.1             # Lower for more deterministic responses
  max_tokens: 1000             # Maximum response length

retrieval:
  top_k: 5                     # Number of results to retrieve
  score_threshold: 0.3         # Minimum similarity score threshold
  timeout_seconds: 30          # Request timeout

embedding:
  model: "embed-english-v3.0"  # Cohere embedding model
  input_type: "search_document" # Cohere input type

storage:
  collection_name: "documentation_chunks" # Qdrant collection name
  vector_size: 1024                       # Expected vector dimension
  distance: "Cosine"                      # Distance metric
  batch_size: 10                          # Batch size for storage operations

pipeline:
  max_concurrent_requests: 5   # Max concurrent requests
  retry_attempts: 3            # Number of retry attempts
  timeout_seconds: 30          # Timeout for operations
```

## Architecture

The agent follows this flow:
```
[User Query]
        ↓ (Google Gemini Agent)
[Query Processing with Tools]
        ↓ (Retrieval Tool Called)
[Qdrant + Cohere Pipeline]
        ↓ (Retrieved Content + Metadata)
[Grounded Response Generation]
        ↓ (Validated Response with Citations)
[Source-Cited Answer]
```

## Validation

The system includes validation capabilities:
- **Grounding validation**: Ensures responses only contain information from retrieved content
- **Source validation**: Verifies metadata preservation and citation accuracy
- **Determinism validation**: Confirms consistent responses for identical inputs
- **Tool validation**: Verifies retrieval tool functionality

## Error Handling

- Graceful handling of API rate limits and timeouts
- Validation of required metadata fields
- Logging of warnings for missing or inconsistent data
- Fallback behaviors for common failure modes

## Security

- API keys are loaded from environment variables only
- No sensitive information is logged
- Configuration validation to prevent invalid setups
- Safe handling of user queries and content retrieval