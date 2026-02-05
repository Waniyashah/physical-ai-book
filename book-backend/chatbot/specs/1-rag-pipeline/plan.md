# Architecture Plan: RAG Pipeline – Website Deployment, Embedding Generation, and Vector Storage

**Feature**: 1-rag-pipeline
**Created**: 2025-12-31
**Status**: Draft
**Author**: AI Architect

## 1. Scope and Dependencies

### In Scope
- Deployment of Docusaurus-based documentation to GitHub Pages
- Content extraction from deployed documentation URLs
- Text chunking with optimal size and overlap strategy
- Semantic embedding generation using Cohere models
- Vector storage in Qdrant database with metadata
- End-to-end ingestion pipeline with configuration management
- Testing and validation framework

### Out of Scope
- Frontend chatbot UI
- OpenAI Agent or orchestration logic
- FastAPI endpoints or API authentication
- Advanced ranking, reranking, or hybrid search
- User-selected text querying logic
- Real-time content updates (only batch processing)

### External Dependencies
- **Cohere API**: For embedding generation
- **Qdrant Cloud**: Vector database service
- **GitHub Pages**: Static website hosting
- **Docusaurus**: Documentation framework
- **Python ecosystem**: For pipeline implementation

## 2. Key Decisions and Rationale

### Decision 1: URL Crawling vs Sitemap-based Extraction
- **Options Considered**:
  - Web crawler to discover all pages
  - Sitemap.xml parsing for known URLs
  - Manual URL list configuration
- **Chosen Approach**: Sitemap-based extraction with optional crawler fallback
- **Rationale**: Sitemap provides authoritative list of pages, crawler handles edge cases
- **Trade-offs**: Sitemap may miss some content but is more reliable; crawler adds complexity

### Decision 2: Chunk Size and Overlap Strategy
- **Options Considered**:
  - Fixed-size chunks (512, 1024, 2048 tokens)
  - Semantic boundary chunks
  - Overlapping chunks with stride
- **Chosen Approach**: 1024-token chunks with 256-token overlap
- **Rationale**: Balances context preservation with retrieval precision
- **Trade-offs**: Larger chunks preserve more context but reduce precision; overlap increases storage

### Decision 3: Cohere Embedding Model Selection
- **Options Considered**:
  - embed-english-v3.0
  - embed-multilingual-v3.0
  - embed-english-light-v3.0
- **Chosen Approach**: embed-english-v3.0 (or latest stable)
- **Rationale**: Best performance for English documentation content
- **Trade-offs**: Higher cost/performance vs lightweight alternatives

### Decision 4: Qdrant Collection Schema and Distance Metric
- **Options Considered**:
  - Cosine similarity vs Euclidean vs Dot product
  - Vector dimensions (depends on embedding model)
  - Payload schema for metadata storage
- **Chosen Approach**: Cosine similarity with metadata payload
- **Rationale**: Standard for semantic search, preserves metadata for traceability
- **Trade-offs**: Cosine is normalized and works well for embeddings

### Decision 5: Update Strategy (Full Reindex vs Incremental)
- **Options Considered**:
  - Full reindex on content updates
  - Incremental updates with change detection
  - Hybrid approach with full backup
- **Chosen Approach**: Incremental updates with full backup option
- **Rationale**: Efficient for regular content updates while maintaining consistency
- **Trade-offs**: More complex implementation but better performance for frequent updates

## 3. Interfaces and API Contracts

### Public APIs
- **Configuration Interface**: Environment variables and config files
- **CLI Interface**: Command-line tools for pipeline execution
- **Pipeline Interface**: Modular functions for each pipeline stage

### API Contract Example
```
Input: List of URLs to process
Output: Success/Failure status with processed count
Errors: HTTP status codes, exception types
```

### Versioning Strategy
- Semantic versioning (v1.x.x) for pipeline releases
- Backward compatibility for configuration files
- Migration scripts for breaking changes

### Error Taxonomy
- **400**: Invalid configuration or input format
- **401**: Authentication failure with external services
- **429**: Rate limit exceeded from external APIs
- **500**: Internal processing errors
- **NetworkError**: Connectivity issues with external services

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance
- **p95 latency**: <500ms for vector similarity queries
- **Throughput**: Process 100 pages per hour during ingestion
- **Resource caps**: <4GB memory during processing, <2 CPU cores

### Reliability
- **SLOs**: 99.5% uptime for pipeline execution
- **Error budget**: 0.5% failure rate tolerance
- **Degradation strategy**: Fallback to cached embeddings during API outages

### Security
- **AuthN/AuthZ**: API key authentication for Cohere and Qdrant
- **Data handling**: No PII processing, only documentation content
- **Secrets management**: Environment variables with encryption at rest
- **Auditing**: Log all pipeline executions and API calls

### Cost
- **Unit economics**: Target < $10/month for basic usage (under 1M tokens)
- **Cohere costs**: Estimated based on token count and API usage
- **Qdrant costs**: Based on vector count and storage requirements

## 5. Data Management and Migration

### Source of Truth
- **Documentation source**: GitHub repository with Docusaurus content
- **Vector store**: Qdrant database as secondary index
- **Metadata**: Stored in Qdrant payloads with URL references

### Schema Evolution
- Versioned configuration files for schema changes
- Migration scripts for payload structure updates
- Backward compatibility for metadata fields

### Data Migration and Rollback
- Backup strategy for Qdrant collections before updates
- Rollback procedures to previous vector database states
- Data validation checks during migration

### Data Retention
- Retain vector embeddings aligned with documentation lifecycle
- Automatic cleanup of obsolete content during updates
- Archival strategy for historical versions

## 6. Operational Readiness

### Observability
- **Logs**: Structured logging for pipeline execution
- **Metrics**: Processing time, success rates, API call counts
- **Traces**: End-to-end request tracing across pipeline stages

### Alerting
- **Thresholds**: Processing time > 10 minutes, success rate < 95%
- **On-call owners**: Development team for pipeline issues
- **Alert channels**: Slack notifications for critical failures

### Runbooks
- **Common tasks**: Pipeline restart, configuration updates, data validation
- **Troubleshooting**: API timeout handling, rate limit management
- **Emergency procedures**: Rollback process, manual data fixes

### Deployment and Rollback Strategies
- **Deployment**: CI/CD pipeline with environment-specific configs
- **Rollback**: Versioned pipeline deployments with quick rollback capability
- **Feature Flags**: Configuration-based feature toggles for pipeline components

## 7. Risk Analysis and Mitigation

### Risk 1: Cohere API Rate Limits
- **Impact**: Pipeline execution delays
- **Blast radius**: Entire embedding generation stage
- **Mitigation**: Implement retry logic with exponential backoff, request batching
- **Kill switch**: Fallback to cached embeddings during outages

### Risk 2: Qdrant Service Unavailability
- **Impact**: Inability to store or retrieve embeddings
- **Blast radius**: Vector storage and retrieval operations
- **Mitigation**: Implement circuit breakers, local cache fallback
- **Kill switch**: Pause ingestion until service is restored

### Risk 3: Content Extraction Failures
- **Impact**: Incomplete documentation indexing
- **Blast radius**: Coverage of searchable content
- **Mitigation**: Robust HTML parsing with fallback strategies, validation checks
- **Guardrails**: Content quality validation before embedding generation

## 8. Evaluation and Validation

### Definition of Done
- [ ] Pipeline successfully processes all documentation URLs
- [ ] Embeddings stored in Qdrant with complete metadata
- [ ] Semantic queries return relevant results (manual validation)
- [ ] All tests pass (unit, integration, end-to-end)
- [ ] Performance benchmarks met
- [ ] Security scans passed

### Output Validation
- **Format validation**: Check vector dimensions match model expectations
- **Requirements validation**: Verify all URLs processed successfully
- **Safety validation**: Ensure no inappropriate content indexed

## 9. Architecture Overview

### High-Level Architecture
```
[Documentation Source]
        ↓ (Deploy to GitHub Pages)
[Public Documentation URLs]
        ↓ (Content Extraction)
[Clean Text Content]
        ↓ (Chunking)
[Text Chunks with Metadata]
        ↓ (Embedding Generation)
[Embedding Vectors + Metadata]
        ↓ (Vector Storage)
[Qdrant Vector Database]
        ↓ (Query Interface)
[Semantic Search Results]
```

### Component Architecture
1. **Deployment Module**: Handles Docusaurus site deployment to GitHub Pages
2. **Extraction Module**: Extracts clean text from HTML documentation
3. **Chunking Module**: Splits content into optimally-sized chunks
4. **Embedding Module**: Generates semantic embeddings using Cohere API
5. **Storage Module**: Stores vectors and metadata in Qdrant
6. **Validation Module**: Tests retrieval quality and system health

### Data Flow
1. URLs are read from sitemap or configuration
2. Content is extracted and cleaned from each URL
3. Content is chunked with overlap to preserve context
4. Embeddings are generated for each chunk
5. Embeddings and metadata are stored in Qdrant
6. Validation tests verify retrieval quality

## 10. Implementation Phases

### Phase 1: Foundation (Week 1)
- Set up GitHub Pages deployment for Docusaurus
- Implement basic content extraction from URLs
- Create configuration management system

### Phase 2: Core Pipeline (Week 2)
- Implement text chunking with configurable parameters
- Integrate Cohere API for embedding generation
- Set up Qdrant collection and storage logic

### Phase 3: Integration and Validation (Week 3)
- End-to-end pipeline integration
- Testing and validation framework
- Performance optimization and error handling

### Phase 4: Production Readiness (Week 4)
- Monitoring and observability setup
- Documentation and runbooks
- Backup and rollback procedures

## 11. Technology Stack

### Primary Technologies
- **Language**: Python 3.9+ for pipeline implementation
- **Embeddings**: Cohere API with embed-english-v3.0 model
- **Vector DB**: Qdrant Cloud (Free Tier)
- **Web Scraping**: BeautifulSoup or similar for content extraction
- **HTTP Client**: Requests or aiohttp for API calls
- **Configuration**: YAML or JSON-based configuration files

### Supporting Libraries
- **qdrant-client**: Python client for Qdrant interaction
- **cohere**: Official Cohere Python SDK
- **beautifulsoup4**: HTML parsing for content extraction
- **python-dotenv**: Environment variable management
- **pytest**: Testing framework
- **structlog**: Structured logging

## 12. Configuration and Environment Setup

### Required Environment Variables
```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
DOCS_BASE_URL=https://your-docs-site.com
```

### Configuration File Example
```yaml
extraction:
  sitemap_url: "https://your-docs-site.com/sitemap.xml"
  allowed_domains: ["your-docs-site.com"]
  content_selectors: ["article", ".markdown", ".docs-content"]
  exclude_selectors: [".header", ".footer", ".nav"]

chunking:
  max_chunk_size: 1024
  overlap_size: 256
  min_chunk_size: 100

embedding:
  model: "embed-english-v3.0"
  input_type: "search_document"

storage:
  collection_name: "documentation_chunks"
  vector_size: 1024
  distance: "Cosine"
  batch_size: 10

pipeline:
  max_concurrent_requests: 5
  retry_attempts: 3
  timeout_seconds: 30
```

This architecture plan provides a comprehensive roadmap for implementing the RAG pipeline with clear technical decisions, risk mitigation strategies, and implementation phases.