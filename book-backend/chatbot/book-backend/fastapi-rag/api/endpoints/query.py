#!/usr/bin/env python3
"""
Query endpoint for FastAPI RAG Integration

This module provides the main query endpoint that accepts user queries and processes
them through the RAG agent.
"""

from fastapi import APIRouter, HTTPException, BackgroundTasks
from datetime import datetime
import time
import asyncio
from typing import Dict, Any, Optional

from models.request_models import QueryRequest
from models.response_models import QueryResponse, ErrorResponse, SourceInfo, ResponseMetadata
from services.rag_agent_adapter import RAGAgentAdapter
from utils.logging import get_logger, log_request_info, log_response_info, log_error
from config import get_settings

# Create router
router = APIRouter()

# Get logger
logger = get_logger("query")


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Query endpoint to process user queries through the RAG agent.

    Args:
        request: QueryRequest containing the user's query and options

    Returns:
        QueryResponse: The AI-generated response with sources and metadata
    """
    try:
        # Log the incoming request
        request_data = {
            "query": request.query,
            "options": request.options.dict() if request.options else {},
            "timestamp": datetime.now().isoformat()
        }
        log_request_info(logger, request_data)

        # Validate query length
        settings = get_settings()
        if len(request.query) > settings.query_max_length:
            raise HTTPException(
                status_code=422,
                detail=f"Query exceeds maximum length of {settings.query_max_length} characters"
            )

        # Start timing
        start_time = time.time()

        # Initialize the RAG agent adapter
        rag_adapter = RAGAgentAdapter()

        # Process the query
        result = await rag_adapter.process_query(
            query=request.query,
            options=request.options
        )

        # Calculate processing time
        processing_time = time.time() - start_time

        # Create source info objects
        sources = []
        if result.get("sources"):
            for source_data in result["sources"]:
                source_info = SourceInfo(
                    url=source_data.get("url", ""),
                    section=source_data.get("section", ""),
                    chunk_id=source_data.get("chunk_id", ""),
                    score=source_data.get("score", 0.0)
                )
                sources.append(source_info)

        # Create response metadata
        metadata = ResponseMetadata(
            processing_time=processing_time,
            timestamp=datetime.now(),
            model_used=result.get("model_used"),
            tokens_used=result.get("tokens_used")
        )

        # Create and return response
        response = QueryResponse(
            success=True,
            response=result["response"],
            sources=sources,
            metadata=metadata
        )

        # Log the response
        response_data = {
            "success": response.success,
            "response_length": len(response.response),
            "processing_time": response.metadata.processing_time,
            "source_count": len(response.sources)
        }
        log_response_info(logger, response_data)

        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        log_error(logger, e, "query_endpoint")

        # Create error response
        error_response = ErrorResponse(
            success=False,
            error=f"Error processing query: {str(e)}",
            error_code="PROCESSING_ERROR",
            details={"query": request.query[:100] + "..." if len(request.query) > 100 else request.query}
        )

        # Log the error response
        logger.error(
            "Query processing failed",
            query=request.query[:100],
            error=str(e),
            error_type=type(e).__name__
        )

        # Raise HTTP exception
        raise HTTPException(
            status_code=500,
            detail=error_response.dict()
        )


# Additional endpoint for testing purposes
@router.post("/query-test", response_model=Dict[str, Any])
async def query_test_endpoint(request: QueryRequest):
    """
    Test endpoint to process queries without full response validation.

    Args:
        request: QueryRequest containing the user's query and options

    Returns:
        Dict: Raw response from the RAG agent for testing purposes
    """
    try:
        # Start timing
        start_time = time.time()

        # Initialize the RAG agent adapter
        rag_adapter = RAGAgentAdapter()

        # Process the query
        result = await rag_adapter.process_query(
            query=request.query,
            options=request.options
        )

        # Calculate processing time
        processing_time = time.time() - start_time

        # Add processing time to result
        result["processing_time"] = processing_time
        result["timestamp"] = datetime.now().isoformat()

        return result
    except Exception as e:
        logger.error("Test query processing failed", error=str(e))
        raise HTTPException(status_code=500, detail=str(e))