#!/usr/bin/env python3
"""
Pydantic models for request validation in FastAPI RAG Integration

This module defines the Pydantic models used for validating incoming requests
to the FastAPI RAG integration service.
"""

from pydantic import BaseModel, Field, validator
from typing import Optional, Dict, Any
from enum import Enum


class QueryOptions(BaseModel):
    """
    Options for query processing.
    """
    temperature: Optional[float] = Field(
        default=0.1,
        ge=0.0,
        le=1.0,
        description="Controls randomness in response generation (0.0-1.0)"
    )
    max_tokens: Optional[int] = Field(
        default=1000,
        ge=1,
        le=4000,
        description="Maximum number of tokens in the response"
    )
    top_p: Optional[float] = Field(
        default=1.0,
        ge=0.0,
        le=1.0,
        description="Controls diversity via nucleus sampling (0.0-1.0)"
    )
    frequency_penalty: Optional[float] = Field(
        default=0.0,
        ge=-2.0,
        le=2.0,
        description="Penalty for repeated tokens (-2.0 to 2.0)"
    )
    presence_penalty: Optional[float] = Field(
        default=0.0,
        ge=-2.0,
        le=2.0,
        description="Penalty for new tokens based on presence (-2.0 to 2.0)"
    )

    class Config:
        schema_extra = {
            "example": {
                "temperature": 0.5,
                "max_tokens": 500,
                "top_p": 1.0
            }
        }


class QueryRequest(BaseModel):
    """
    Model for query requests to the RAG agent.
    """
    query: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The user's query to be processed by the RAG agent"
    )
    options: Optional[QueryOptions] = Field(
        default=None,
        description="Additional options for query processing"
    )

    @validator('query')
    def validate_query(cls, v):
        """
        Validate the query field.

        Args:
            v: The query string

        Returns:
            The validated query string
        """
        if not v or not v.strip():
            raise ValueError('Query cannot be empty or whitespace only')
        return v.strip()

    class Config:
        schema_extra = {
            "example": {
                "query": "What is the capital of France?",
                "options": {
                    "temperature": 0.3,
                    "max_tokens": 200
                }
            }
        }


class HealthCheckRequest(BaseModel):
    """
    Model for health check requests (currently empty, but defined for consistency).
    """
    pass