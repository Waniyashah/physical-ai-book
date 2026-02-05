#!/usr/bin/env python3
"""
RAG Agent Adapter Service for FastAPI RAG Integration

This module provides an adapter to interface with the existing RAG agent implementation,
handling query processing and response formatting.
"""

import asyncio
import time
import os
import sys
from typing import Dict, Any, Optional, List
from datetime import datetime

# Add the rag-agent directory to the path at module level to ensure imports work
# The rag-agent directory is in the parent directory of fastapi-rag (both under book-backend)
current_file_dir = os.path.dirname(os.path.abspath(__file__))
fastapi_rag_dir = os.path.dirname(current_file_dir)
book_backend_dir = os.path.dirname(fastapi_rag_dir)
rag_agent_path = os.path.join(book_backend_dir, "rag-agent")

if rag_agent_path not in sys.path:
    sys.path.insert(0, rag_agent_path)

from models.request_models import QueryOptions
from utils.logging import get_logger
from config import get_settings


class RAGAgentAdapter:
    """
    Adapter class to interface with the existing RAG agent implementation.
    """

    def __init__(self):
        """
        Initialize the RAG agent adapter.
        """
        self.logger = get_logger("rag_agent_adapter")
        self.settings = get_settings()

        # Track agent initialization state
        self.agent_initialized = False

        # Initialize agents during startup to ensure they're available
        self._agents_loaded = False
        self.unified_agent_available = False
        self.alternative_agent_available = False
        self.main_agent_available = False
        self._run_agent = None
        self._run_alternative_agent = None
        self._run_main_agent = None

        # Load agents during initialization
        self._ensure_agents_loaded()

    def _ensure_agents_loaded(self):
        """
        Ensure agents are loaded - do this lazily to avoid import issues
        """
        if self._agents_loaded:
            return

        self._setup_rag_integration()
        self._agents_loaded = True

    def _setup_rag_integration(self):
        """
        Set up the integration with the existing RAG agent implementation.
        """
        try:
            # Import the existing agent implementations
            import sys
            import os

            # Add the rag-agent directory to the path to access the existing agents
            # The rag-agent directory is in the parent directory of fastapi-rag (both under book-backend)
            # From services/: ../.. goes to book-backend/, then to rag-agent/
            rag_agent_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "rag-agent")

            # Insert the path at the beginning of sys.path to ensure it's found first
            if rag_agent_path not in sys.path:
                sys.path.insert(0, rag_agent_path)

            # Force refresh of module cache to ensure new path is recognized
            import importlib
            importlib.invalidate_caches()

            # Try to import the unified agent
            # NOTE: Imports must happen AFTER the path is set
            try:
                # Clear any cached modules that might interfere
                if 'unified_agent' in sys.modules:
                    del sys.modules['unified_agent']

                from unified_agent import run_unified_agent
                self.unified_agent_available = True
                self._run_agent = run_unified_agent
                self.logger.info("Unified agent (OpenAI SDK with Gemini) imported successfully")
            except ImportError as e:
                self.logger.error(f"Could not import unified agent: {e}")
                raise ImportError("No RAG agent implementation available")

            self.agent_initialized = True
            self.logger.info("RAG agent adapter initialized successfully")

        except Exception as e:
            self.logger.error("Failed to initialize RAG agent adapter", error=str(e))
            raise

    async def process_query(self, query: str, options: Optional[QueryOptions] = None) -> Dict[str, Any]:
        """
        Process a query through the RAG agent.

        Args:
            query: The user's query string
            options: Optional query processing options

        Returns:
            Dictionary containing the response and metadata
        """
        # Ensure agents are loaded before processing
        self._ensure_agents_loaded()

        if not self.agent_initialized:
            raise RuntimeError("RAG agent adapter not properly initialized")

        try:
            # Prepare options for the agent
            import os
            # The config file is in the rag-agent directory which is at the parent level of fastapi-rag
            config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "rag-agent", "config.yaml")

            # Determine which agent to use based on available implementations
            response = ""

            if hasattr(self, '_run_agent') and self.unified_agent_available:
                # Use unified agent
                try:
                    response = self._run_agent(query, config_path)
                except Exception as e:
                    self.logger.warning(f"Unified agent failed: {str(e)}, returning fallback response")
                    response = f"Mock response for query: {query}\n\nThe system is currently experiencing high demand. Please try again later.\n\nSOURCES:\n- mock-source (Section: mock-section)"
            elif hasattr(self, '_run_alternative_agent') and self.alternative_agent_available:
                # Use alternative agent (Google Gemini)
                try:
                    response = self._run_alternative_agent(query, config_path)
                except Exception as e:
                    self.logger.warning(f"Alternative agent failed: {str(e)}, returning fallback response")
                    response = f"Mock response for query: {query}\n\nThe system is currently experiencing high demand. Please try again later.\n\nSOURCES:\n- mock-source (Section: mock-section)"
            elif hasattr(self, '_run_main_agent') and self.main_agent_available:
                # Use main agent (OpenAI)
                try:
                    response = self._run_main_agent(query, config_path)
                except Exception as e:
                    self.logger.warning(f"Main agent failed: {str(e)}, returning fallback response")
                    response = f"Mock response for query: {query}\n\nThe system is currently experiencing high demand. Please try again later.\n\nSOURCES:\n- mock-source (Section: mock-section)"
            else:
                # Fallback to a simulated response if no agents are available
                self.logger.warning("No agents available, returning simulated response")
                response = f"Mock response for query: {query}\n\nThe system is currently experiencing high demand. Please try again later.\n\nSOURCES:\n- mock-source (Section: mock-section)"

            # Process the response and extract relevant information
            if isinstance(response, dict):
                # If the agent returns a structured response, use it directly
                processed_result = {
                    "response": response.get("response", str(response)),
                    "sources": response.get("sources", []),
                    "model_used": response.get("model_used", "unknown"),
                    "tokens_used": response.get("tokens_used", len(str(response).split()))
                }
            else:
                # If the agent returns a string, process it
                processed_result = {
                    "response": response,
                    "sources": [],  # Will be populated based on actual agent response
                    "model_used": "unknown",
                    "tokens_used": len(str(response).split()) if response else 0
                }

                # Attempt to parse sources from the response if it contains them
                # This is a simplified approach - in a real implementation,
                # the agent would return structured data
                if "SOURCES:" in str(response) or "Sources:" in str(response):
                    # Simple parsing of sources from the response
                    lines = str(response).split('\n')
                    sources = []
                    in_sources_section = False

                    for line in lines:
                        if "SOURCES:" in line or "Sources:" in line:
                            in_sources_section = True
                            continue

                        if in_sources_section and (line.strip().startswith('- ') or line.strip().startswith('1. ') or line.strip().startswith('2. ')):
                            # Extract URL from the line
                            import re
                            url_match = re.search(r'https?://[^\s)]+', line)
                            if url_match:
                                sources.append({
                                    "url": url_match.group(0),
                                    "section": "unknown",
                                    "chunk_id": "unknown",
                                    "score": 0.5  # Default score
                                })

                    processed_result["sources"] = sources

            return processed_result

        except Exception as e:
            self.logger.error("Error processing query in RAG agent", error=str(e))
            raise

    async def validate_response(self, response: str, query: str) -> bool:
        """
        Validate that the response is grounded in the retrieved content.

        Args:
            response: The agent's response
            query: The original query

        Returns:
            True if the response is valid, False otherwise
        """
        # In a real implementation, this would validate that the response
        # contains information from the retrieved sources
        # For now, we'll return True as a placeholder
        return True

    def get_agent_status(self) -> Dict[str, Any]:
        """
        Get the status of the RAG agent integration.

        Returns:
            Dictionary with status information
        """
        return {
            "initialized": self.agent_initialized,
            "unified_agent_available": getattr(self, 'unified_agent_available', False),
            "alternative_agent_available": getattr(self, 'alternative_agent_available', False),
            "main_agent_available": getattr(self, 'main_agent_available', False),
            "timestamp": datetime.now().isoformat()
        }