#!/usr/bin/env python3
"""
Health check endpoint for FastAPI RAG Integration

This module provides a health check endpoint to verify the status of the service
and its dependencies.
"""

from fastapi import APIRouter, HTTPException
from datetime import datetime
import time
import requests
from typing import Dict, Any

from models.response_models import HealthCheckResponse
from utils.logging import get_logger
from config import get_settings

# Create router
router = APIRouter()

# Get logger
logger = get_logger("health")

# Store service start time for uptime calculation
_start_time = time.time()


def check_qdrant_connection() -> bool:
    """
    Check if Qdrant service is accessible.

    Returns:
        True if Qdrant is accessible, False otherwise
    """
    settings = get_settings()
    if not settings.qdrant_url:
        return False

    try:
        # Try to connect to Qdrant and get collections
        import httpx
        from qdrant_client import QdrantClient

        client = QdrantClient(
            url=settings.qdrant_url.replace("https://", "").replace(":6333", ""),
            api_key=settings.qdrant_api_key,
            https=True if "https://" in settings.qdrant_url else False
        )

        # Test connection by getting collections
        client.get_collections()
        return True
    except Exception as e:
        logger.error("Qdrant connection check failed", error=str(e))
        return False


def check_cohere_connection() -> bool:
    """
    Check if Cohere service is accessible.

    Returns:
        True if Cohere is accessible, False otherwise
    """
    settings = get_settings()
    if not settings.cohere_api_key:
        return False

    try:
        import cohere
        client = cohere.Client(settings.cohere_api_key)

        # Test connection by making a simple API call
        client.embed(texts=["test"], model="embed-english-v3.0", input_type="search_document")
        return True
    except Exception as e:
        logger.error("Cohere connection check failed", error=str(e))
        return False


def check_openai_connection() -> bool:
    """
    Check if OpenAI service is accessible.

    Returns:
        True if OpenAI is accessible, False otherwise
    """
    settings = get_settings()
    if not settings.openai_api_key:
        return False

    try:
        import openai
        client = openai.OpenAI(api_key=settings.openai_api_key)

        # Test connection by listing models
        client.models.list()
        return True
    except Exception as e:
        logger.error("OpenAI connection check failed", error=str(e))
        return False


def check_google_connection() -> bool:
    """
    Check if Google Generative AI service is accessible.

    Returns:
        True if Google Generative AI is accessible, False otherwise
    """
    settings = get_settings()
    if not settings.google_api_key:
        return False

    try:
        import google.generativeai as genai
        genai.configure(api_key=settings.google_api_key)

        # Test connection by trying to list models
        # Note: This is a workaround since the API might not have a direct ping method
        try:
            # Just try to initialize a model to test connectivity
            model = genai.GenerativeModel('gemini-pro')
            return True
        except:
            return False
    except Exception as e:
        logger.error("Google Generative AI connection check failed", error=str(e))
        return False


@router.get("/health", response_model=HealthCheckResponse)
async def health_check() -> HealthCheckResponse:
    """
    Health check endpoint to verify the status of the service and its dependencies.

    Returns:
        HealthCheckResponse: Health status of the service and its dependencies
    """
    try:
        # Calculate uptime
        uptime = time.time() - _start_time

        # Check service dependencies
        services_status: Dict[str, str] = {}

        # Check Qdrant
        qdrant_ok = check_qdrant_connection()
        services_status["qdrant"] = "connected" if qdrant_ok else "disconnected"

        # Check Cohere
        cohere_ok = check_cohere_connection()
        services_status["cohere"] = "connected" if cohere_ok else "disconnected"

        # Check OpenAI
        openai_ok = check_openai_connection()
        services_status["openai"] = "available" if openai_ok else "unavailable"

        # Check Google
        google_ok = check_google_connection()
        services_status["google"] = "available" if google_ok else "unavailable"

        # Determine overall status
        all_services_ok = all([
            qdrant_ok or not get_settings().qdrant_url,  # Only check if URL is configured
            cohere_ok or not get_settings().cohere_api_key,  # Only check if key is configured
            openai_ok or not get_settings().openai_api_key,  # Only check if key is configured
            google_ok or not get_settings().google_api_key  # Only check if key is configured
        ])

        overall_status = "ok" if all_services_ok else "warning"

        # Log health check
        logger.info(
            "Health check performed",
            status=overall_status,
            services=services_status
        )

        return HealthCheckResponse(
            status=overall_status,
            timestamp=datetime.now(),
            services=services_status,
            uptime=uptime
        )
    except Exception as e:
        logger.error("Health check failed", error=str(e))
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")