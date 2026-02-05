#!/usr/bin/env python3
"""
FastAPI RAG Integration - Main Entry Point

This is the main entry point for the FastAPI RAG integration service.
It sets up the FastAPI application, configures middleware, and includes the API routes.
"""

import os
import sys
import logging
from contextlib import asynccontextmanager

import uvicorn
import structlog
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure structlog
structlog.configure(
    processors=[
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.UnicodeDecoder(),
        structlog.processors.JSONRenderer()
    ],
    context_class=dict,
    logger_factory=structlog.stdlib.LoggerFactory(),
    wrapper_class=structlog.stdlib.BoundLogger,
    cache_logger_on_first_use=True,
)

logger = structlog.get_logger()

# Set logging level based on environment
log_level = os.getenv("LOG_LEVEL", "INFO")
logging.basicConfig(level=getattr(logging, log_level.upper()))

# Import routers after setting up logging to avoid circular imports
from api.router import api_router
from config import get_settings


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for application startup and shutdown.
    """
    # Startup
    logger.info("Starting FastAPI RAG Integration service")

    # Perform any startup tasks here
    settings = get_settings()
    logger.info(f"Service configured with host: {settings.host}, port: {settings.port}")

    yield

    # Shutdown
    logger.info("Shutting down FastAPI RAG Integration service")


# Create FastAPI app instance
app = FastAPI(
    title="FastAPI RAG Integration API",
    description="API for integrating RAG agent functionality with FastAPI",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict this to your frontend domains
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Uncomment the next line if you need to expose headers to the frontend
    # expose_headers=["Access-Control-Allow-Origin"]
)

# Include API routes
app.include_router(api_router, prefix="/api", tags=["api"])

@app.get("/")
async def root():
    """
    Root endpoint for the API.
    """
    return {"message": "Welcome to FastAPI RAG Integration API"}

if __name__ == "__main__":
    # Get settings
    settings = get_settings()

    # Run the application
    uvicorn.run(
        "main:app",
        host=settings.host,
        port=settings.port,
        reload=True,  # Enable hot reload for development
        log_level=settings.log_level.lower()
    )