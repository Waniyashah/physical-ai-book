# Implementation Plan: RAG Pipeline Technical Implementation

**Feature**: 1-rag-pipeline
**Created**: 2025-12-31
**Status**: Draft
**Author**: AI Architect

## 1. Project Initialization

### Backend Directory Structure
```
book-backend/
├── chatbot/                 # Current directory (already exists)
├── rag-pipeline/           # New directory for this implementation
│   ├── __init__.py
│   ├── main.py             # Single main file for ingestion pipeline
│   ├── requirements.txt    # Dependencies including cohere, qdrant-client, beautifulsoup4
│   ├── .env               # Environment variables (not committed)
│   ├── .uv.lock           # UV lock file
│   └── pyproject.toml     # Project configuration for UV
└── README.md              # Updated documentation
```

### UV Environment Setup
- Initialize UV environment for fast package management
- Create `pyproject.toml` with project dependencies
- Generate `requirements.txt` from `pyproject.toml`
- Set up virtual environment with `uv venv`
- Install dependencies with `uv pip install`

### Dependencies to Install
- `cohere`: For embedding generation
- `qdrant-client`: For Qdrant database interaction
- `beautifulsoup4`: For HTML content extraction
- `requests`: For HTTP requests
- `python-dotenv`: For environment variable management
- `tqdm`: For progress bars during processing

## 2. Ingestion Pipeline Architecture (main.py)

### Single File Structure
The `main.py` will contain a modular implementation with these key functions:

1. **URL Fetching Module**
   - Fetch content from provided URLs
   - Handle HTTP errors and retries
   - Support for sitemap parsing

2. **Text Extraction Module**
   - Use BeautifulSoup to extract clean text from HTML
   - Remove navigation, headers, footers
   - Preserve semantic structure

3. **Content Chunking Module**
   - Split content into fixed-size chunks
   - Implement overlap for context preservation
   - Track chunk metadata (URL, section, chunk ID)

4. **Embedding Generation Module**
   - Interface with Cohere API for vector generation
   - Handle rate limiting and errors
   - Batch processing for efficiency

5. **Qdrant Storage Module**
   - Create collection with appropriate schema
   - Store embeddings with metadata
   - Handle upsert operations

### Main Pipeline Flow
```
main() → fetch_urls() → extract_text() → chunk_content() → generate_embeddings() → store_in_qdrant()
```

## 3. Embedding Generation and Qdrant Storage

### Cohere Integration
- Use `embed-english-v3.0` model for embeddings
- Implement proper API key management
- Handle rate limiting with exponential backoff
- Batch requests for efficiency (max 96 texts per request)

### Qdrant Schema Design
- **Collection name**: `documentation_chunks`
- **Vector size**: 1024 (for embed-english-v3.0)
- **Distance metric**: Cosine
- **Payload schema**:
  ```json
  {
    "url": "string",
    "section": "string",
    "chunk_id": "string",
    "content": "string",
    "source": "string"
  }
  ```

### Storage Implementation
- Create collection if it doesn't exist
- Use upsert operation for idempotency
- Implement batch storage for performance
- Add metadata validation before storage

## 4. Key Technical Decisions

### Decision 1: Chunk Size and Overlap
- **Selected**: 1024 tokens with 256-token overlap
- **Rationale**: Balances context preservation with retrieval precision
- **Implementation**: Use character-based splitting with overlap
- **Metadata tracking**: Each chunk will have unique ID with URL context

### Decision 2: Embedding Model
- **Selected**: Cohere `embed-english-v3.0`
- **Rationale**: Best performance for English documentation content
- **Configuration**: Use `search_document` input type for retrieval tasks
- **Fallback**: Plan for model version updates

### Decision 3: Qdrant Collection Schema
- **Selected**: Single collection with rich metadata payload
- **Rationale**: Enables flexible querying and filtering
- **Distance**: Cosine similarity for semantic search
- **Vector size**: 1024 dimensions to match embedding model

### Decision 4: Error Handling Strategy
- **Retry logic**: Exponential backoff for API calls
- **Fallback storage**: Local cache if Qdrant unavailable
- **Partial processing**: Continue with other documents if one fails
- **Logging**: Comprehensive error logging for debugging

## 5. Validation and Testing Approach

### Pipeline Validation
1. **URL Accessibility Check**
   - Verify all URLs in the input list are accessible
   - Test response times and error handling

2. **Text Extraction Validation**
   - Confirm clean text extraction without HTML tags
   - Validate content completeness and quality

3. **Embedding Quality Check**
   - Verify embedding dimensions match expected size
   - Test embedding consistency across multiple runs

4. **Storage Verification**
   - Confirm vectors stored successfully in Qdrant
   - Validate metadata integrity in payloads

### Sample Query Testing
- Implement semantic search functionality
- Test with sample queries against stored content
- Validate relevance of returned results
- Measure query response times

### Acceptance Criteria
- [ ] All URLs processed without critical errors
- [ ] Text extraction achieves >90% content retention
- [ ] Embeddings generated successfully for all chunks
- [ ] All vectors stored in Qdrant with complete metadata
- [ ] Sample queries return relevant results (>80% relevance)
- [ ] Pipeline completes within acceptable time limits

## 6. Implementation Phases

### Phase 1: Project Setup (Day 1)
- Create `rag-pipeline/` directory
- Set up UV environment
- Create `pyproject.toml` and install dependencies
- Set up basic `main.py` structure

### Phase 2: Core Pipeline (Day 2)
- Implement URL fetching functionality
- Create text extraction module
- Build content chunking module
- Add basic error handling

### Phase 3: Embedding and Storage (Day 3)
- Integrate Cohere API for embeddings
- Implement Qdrant storage functionality
- Add metadata tracking and validation
- Implement batch processing

### Phase 4: Validation and Testing (Day 4)
- Create sample query functionality
- Implement comprehensive validation
- Test with real documentation content
- Optimize performance and fix issues

## 7. Configuration Management

### Environment Variables (.env)
```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
DOCS_BASE_URL=https://your-docs-site.com
```

### Command Line Interface
```
python main.py --urls-file urls.txt --config config.json
python main.py --validate --query "sample search query"
```

## 8. Error Handling and Resilience

### API Rate Limiting
- Implement retry logic with exponential backoff
- Queue requests to respect rate limits
- Log rate limit events for monitoring

### Network Resilience
- Handle timeouts and connection failures
- Implement circuit breaker pattern
- Provide fallback mechanisms

### Data Validation
- Validate URL format before processing
- Check embedding dimensions before storage
- Verify metadata completeness

## 9. Performance Considerations

### Memory Management
- Process documents in batches to control memory usage
- Clear variables after processing each document
- Implement streaming where possible

### Processing Efficiency
- Parallelize URL fetching where possible
- Batch embedding requests to Cohere API
- Optimize Qdrant upsert operations

This implementation plan provides a comprehensive roadmap for building the RAG pipeline with a focus on the technical requirements specified.