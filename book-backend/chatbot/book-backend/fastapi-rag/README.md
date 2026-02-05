# FastAPI RAG Integration

This project provides a FastAPI-based backend service that integrates RAG (Retrieval-Augmented Generation) agent functionality for AI-powered question answering over documentation.

## Features

- FastAPI-based REST API for RAG functionality
- Integration with existing RAG agent implementations
- Support for both OpenAI and Google Gemini models
- Structured response format with source citations
- Health check endpoint for monitoring
- Proper error handling and logging

## Prerequisites

- Python 3.9+
- pip
- Access to API keys for the AI services you want to use:
  - OpenAI API key (optional)
  - Google Gemini API key (optional)
  - Cohere API key (for embeddings)
  - Qdrant API key (for vector storage)

## Installation

1. Clone the repository
2. Navigate to the `book-backend/fastapi-rag` directory
3. Install the required dependencies:

```bash
pip install -r requirements.txt
```

## Configuration

Copy the `.env.example` file to `.env` and update the values:

```bash
cp .env.example .env
```

Then edit the `.env` file with your actual API keys and configuration values:

- `OPENAI_API_KEY`: Your OpenAI API key (if using OpenAI)
- `GOOGLE_API_KEY`: Your Google Gemini API key (if using Google)
- `COHERE_API_KEY`: Your Cohere API key (required for embeddings)
- `QDRANT_URL`: Your Qdrant cluster URL (required for retrieval)
- `QDRANT_API_KEY`: Your Qdrant API key (required for retrieval)
- `QDRANT_COLLECTION_NAME`: Name of the collection with documentation chunks

## Running the Service

Start the service with:

```bash
cd book-backend/fastapi-rag
python main.py
```

The API will be available at `http://localhost:8000` by default.

## API Endpoints

### Health Check
- `GET /api/health` - Check the health status of the service and its dependencies

### Query Processing
- `POST /api/query` - Submit a query to the RAG agent

Request body:
```json
{
  "query": "Your question here",
  "options": {
    "temperature": 0.1,
    "max_tokens": 1000
  }
}
```

Response:
```json
{
  "success": true,
  "response": "AI-generated answer",
  "sources": [
    {
      "url": "source-url",
      "section": "section-title",
      "chunk_id": "unique-chunk-id",
      "score": 0.85
    }
  ],
  "metadata": {
    "processing_time": 1.2,
    "timestamp": "2023-10-20T15:30:00Z",
    "model_used": "gpt-4",
    "tokens_used": 120
  }
}
```

## Testing

You can use the included test client to verify the service is working:

```bash
python test_client.py
```

## Architecture

The service follows a modular architecture:

- `main.py`: Application entry point and FastAPI setup
- `api/`: API routes and endpoints
- `models/`: Pydantic models for request/response validation
- `services/`: Business logic and integration with RAG agents
- `utils/`: Utility functions (logging, helpers)
- `config.py`: Configuration management

## Error Handling

The service implements comprehensive error handling:

- Input validation with Pydantic models
- Proper HTTP status codes for different error scenarios
- Structured error responses with relevant details
- Detailed logging for debugging

## Logging

The service uses structlog for structured logging. Logs include:
- Request and response information
- Processing times
- Error details
- Service health status

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[Specify license here]