#!/usr/bin/env python3
"""
API Router for FastAPI RAG Integration

This module defines the main API router and includes all the individual endpoint routers.
"""

from fastapi import APIRouter
from .endpoints.health import router as health_router
from .endpoints.query import router as query_router


# Create the main API router
api_router = APIRouter()

# Include individual endpoint routers
api_router.include_router(health_router, tags=["health"])
api_router.include_router(query_router, tags=["query"])