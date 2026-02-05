#!/usr/bin/env python3
"""
Pydantic models for response validation in FastAPI RAG Integration

This module defines the Pydantic models used for validating outgoing responses
from the FastAPI RAG integration service.
"""

from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class SourceInfo(BaseModel):
    """
    Model representing information about a source used in the response.
    """
    url: str = Field(
        ...,
        description="URL of the source document"
    )
    section: str = Field(
        ...,
        description="Section or heading in the source document"
    )
    chunk_id: Optional[str] = Field(
        default=None,
        description="Unique identifier for the content chunk"
    )
    score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Similarity score for the retrieved content (0.0-1.0)"
    )

    class Config:
        schema_extra = {
            "example": {
                "url": "https://example.com/doc.html",
                "section": "Introduction",
                "chunk_id": "chunk_123",
                "score": 0.85
            }
        }


class ResponseMetadata(BaseModel):
    """
    Model representing metadata about the response.
    """
    processing_time: float = Field(
        ...,
        description="Time taken to process the query in seconds"
    )
    timestamp: datetime = Field(
        ...,
        description="Timestamp when the response was generated"
    )
    model_used: Optional[str] = Field(
        default=None,
        description="Name of the model used for response generation"
    )
    tokens_used: Optional[int] = Field(
        default=None,
        description="Number of tokens used in the response"
    )

    class Config:
        schema_extra = {
            "example": {
                "processing_time": 2.5,
                "timestamp": "2023-10-20T15:30:00Z",
                "model_used": "gpt-4",
                "tokens_used": 120
            }
        }


class QueryResponse(BaseModel):
    """
    Model for responses from the RAG agent query endpoint.
    """
    success: bool = Field(
        ...,
        description="Indicates whether the query was processed successfully"
    )
    response: str = Field(
        ...,
        description="The AI-generated response to the user's query"
    )
    sources: List[SourceInfo] = Field(
        default=[],
        description="List of sources used to generate the response"
    )
    metadata: ResponseMetadata = Field(
        ...,
        description="Metadata about the response generation"
    )

    class Config:
        schema_extra = {
            "example": {
                "success": True,
                "response": "The capital of France is Paris.",
                "sources": [
                    {
                        "url": "https://en.wikipedia.org/wiki/France",
                        "section": "Geography",
                        "chunk_id": "geo_france_001",
                        "score": 0.92
                    }
                ],
                "metadata": {
                    "processing_time": 1.2,
                    "timestamp": "2023-10-20T15:30:00Z",
                    "model_used": "gpt-4",
                    "tokens_used": 15
                }
            }
        }


class ErrorResponse(BaseModel):
    """
    Model for error responses from the API.
    """
    success: bool = Field(
        default=False,
        description="Always False for error responses"
    )
    error: str = Field(
        ...,
        description="Human-readable error message"
    )
    error_code: Optional[str] = Field(
        default=None,
        description="Machine-readable error code"
    )
    details: Optional[dict] = Field(
        default=None,
        description="Additional error details"
    )

    class Config:
        schema_extra = {
            "example": {
                "success": False,
                "error": "Invalid query format",
                "error_code": "INVALID_QUERY",
                "details": {
                    "field": "query",
                    "reason": "Query too short"
                }
            }
        }


class HealthCheckResponse(BaseModel):
    """
    Model for health check responses.
    """
    status: str = Field(
        ...,
        description="Overall system status ('ok', 'warning', 'error')"
    )
    timestamp: datetime = Field(
        ...,
        description="Timestamp of the health check"
    )
    services: dict = Field(
        ...,
        description="Status of individual services"
    )
    uptime: Optional[float] = Field(
        default=None,
        description="Uptime in seconds"
    )

    class Config:
        schema_extra = {
            "example": {
                "status": "ok",
                "timestamp": "2023-10-20T15:30:00Z",
                "services": {
                    "rag_agent": "available",
                    "qdrant": "connected",
                    "cohere": "connected"
                },
                "uptime": 3600.5
            }
        }