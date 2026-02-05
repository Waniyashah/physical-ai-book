#!/usr/bin/env python3
"""
Logging utilities for FastAPI RAG Integration

This module provides centralized logging configuration and utility functions
for the FastAPI RAG integration service.
"""

import structlog
import logging
from typing import Any, Dict


def setup_logging(log_level: str = "INFO") -> None:
    """
    Set up structlog with appropriate processors and configuration.

    Args:
        log_level: The logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    """
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

    # Set the root logging level
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format="%(message)s"
    )


def get_logger(name: str = None) -> structlog.BoundLogger:
    """
    Get a configured logger instance.

    Args:
        name: Optional name for the logger

    Returns:
        Configured structlog logger instance
    """
    if name:
        return structlog.get_logger(name)
    else:
        return structlog.get_logger()


def log_request_info(logger: structlog.BoundLogger, request_data: Dict[str, Any]) -> None:
    """
    Log information about an incoming request.

    Args:
        logger: The logger instance to use
        request_data: Dictionary containing request information
    """
    logger.info(
        "Incoming request",
        query=request_data.get("query", "")[:100],  # First 100 chars of query
        options=request_data.get("options", {}),
        timestamp=request_data.get("timestamp")
    )


def log_response_info(logger: structlog.BoundLogger, response_data: Dict[str, Any]) -> None:
    """
    Log information about an outgoing response.

    Args:
        logger: The logger instance to use
        response_data: Dictionary containing response information
    """
    logger.info(
        "Outgoing response",
        success=response_data.get("success"),
        response_length=len(str(response_data.get("response", ""))),
        processing_time=response_data.get("metadata", {}).get("processing_time"),
        source_count=len(response_data.get("sources", []))
    )


def log_error(logger: structlog.BoundLogger, error: Exception, context: str = "") -> None:
    """
    Log an error with appropriate context.

    Args:
        logger: The logger instance to use
        error: The exception that occurred
        context: Additional context about where the error occurred
    """
    logger.error(
        "Error occurred",
        error=str(error),
        error_type=type(error).__name__,
        context=context
    )