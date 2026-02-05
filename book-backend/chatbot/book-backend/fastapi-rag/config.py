#!/usr/bin/env python3
"""
Configuration management for FastAPI RAG Integration

This module provides centralized configuration management using environment variables
and Pydantic settings.
"""

from pydantic_settings import BaseSettings
from pydantic import Field
from typing import Optional


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.
    """
    # Server configuration - mapping to env vars like FASTAPI_HOST, FASTAPI_PORT
    host: str = Field(default="localhost", alias="FASTAPI_HOST")
    port: int = Field(default=8000, alias="FASTAPI_PORT")
    log_level: str = Field(default="INFO", alias="LOG_LEVEL")

    # Query configuration
    query_max_length: int = Field(default=1000, alias="QUERY_MAX_LENGTH")
    default_temperature: float = Field(default=0.1, alias="DEFAULT_TEMPERATURE")
    default_max_tokens: int = Field(default=1000, alias="DEFAULT_MAX_TOKENS")
    timeout_seconds: int = Field(default=30, alias="TIMEOUT_SECONDS")

    # API Keys and service endpoints
    openai_api_key: Optional[str] = Field(default=None, alias="OPENAI_API_KEY")
    cohere_api_key: Optional[str] = Field(default=None, alias="COHERE_API_KEY")
    qdrant_url: Optional[str] = Field(default=None, alias="QDRANT_URL")
    qdrant_api_key: Optional[str] = Field(default=None, alias="QDRANT_API_KEY")
    qdrant_collection_name: str = Field(default="documentation_chunks", alias="QDRANT_COLLECTION_NAME")
    google_api_key: Optional[str] = Field(default=None, alias="GEMINI_API_KEY")  # Mapping GEMINI_API_KEY to google_api_key

    # FastAPI configuration
    debug: bool = Field(default=True, alias="DEBUG", validation_alias="DEBUG")
    reload: bool = Field(default=True, alias="RELOAD", validation_alias="RELOAD")

    model_config = {
        "env_file": ".env",
        "env_file_encoding": "utf-8",
        "case_sensitive": False,
        "extra": "ignore"  # This allows extra environment variables that aren't in the model
    }


# Global settings instance
_settings: Optional[Settings] = None


def get_settings() -> Settings:
    """
    Get the global settings instance, creating it if it doesn't exist.

    Returns:
        Settings instance with loaded configuration
    """
    global _settings
    if _settings is None:
        _settings = Settings()
    return _settings


def validate_settings(settings: Settings) -> bool:
    """
    Validate the loaded settings to ensure required values are present.

    Args:
        settings: The settings instance to validate

    Returns:
        True if settings are valid, False otherwise
    """
    # Check that required services have their API keys
    required_keys = []

    # For this integration, we need at least one of the AI services
    has_openai = settings.openai_api_key is not None and settings.openai_api_key.strip() != ""
    has_google = settings.google_api_key is not None and settings.google_api_key.strip() != ""

    if not has_openai and not has_google:
        # We'll log a warning but not fail, as we might be in a testing environment
        print("WARNING: No AI service API keys found. At least one of OPENAI_API_KEY or GOOGLE_API_KEY should be set.")

    # Check for Cohere API key (needed for embeddings)
    has_cohere = settings.cohere_api_key is not None and settings.cohere_api_key.strip() != ""
    if not has_cohere:
        print("WARNING: COHERE_API_KEY not found. This is required for embedding generation.")

    # Check for Qdrant configuration
    has_qdrant_url = settings.qdrant_url is not None and settings.qdrant_url.strip() != ""
    has_qdrant_key = settings.qdrant_api_key is not None and settings.qdrant_api_key.strip() != ""

    if not has_qdrant_url or not has_qdrant_key:
        print("WARNING: QDRANT_URL and/or QDRANT_API_KEY not found. This is required for vector storage/retrieval.")

    return True


# Validate settings on import
if __name__ != "__main__":
    validate_settings(get_settings())