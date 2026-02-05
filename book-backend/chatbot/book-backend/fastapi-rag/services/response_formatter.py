#!/usr/bin/env python3
"""
Response Formatter Service for FastAPI RAG Integration

This module provides functionality to format responses from the RAG agent
into the structured format expected by the API.
"""

import json
import re
from datetime import datetime
from typing import Dict, Any, List, Optional
from urllib.parse import urlparse

from models.response_models import SourceInfo, ResponseMetadata
from utils.logging import get_logger


class ResponseFormatter:
    """
    Service class to format responses from the RAG agent into the expected API format.
    """

    def __init__(self):
        """
        Initialize the response formatter.
        """
        self.logger = get_logger("response_formatter")

    def format_response(
        self,
        raw_response: str,
        query: str,
        processing_time: float,
        sources: Optional[List[Dict[str, Any]]] = None,
        model_used: Optional[str] = None,
        tokens_used: Optional[int] = None
    ) -> Dict[str, Any]:
        """
        Format a raw response from the RAG agent into the structured API format.

        Args:
            raw_response: The raw response string from the RAG agent
            query: The original query that generated the response
            processing_time: Time taken to process the query
            sources: Optional list of sources used in the response
            model_used: Name of the model used to generate the response
            tokens_used: Number of tokens used in the response

        Returns:
            Dictionary with formatted response data
        """
        try:
            # Extract sources from the raw response if not provided
            extracted_sources = sources or self._extract_sources(raw_response)

            # Create SourceInfo objects
            source_objects = []
            for source_data in extracted_sources:
                source_obj = SourceInfo(
                    url=source_data.get("url", ""),
                    section=source_data.get("section", ""),
                    chunk_id=source_data.get("chunk_id", ""),
                    score=source_data.get("score", 0.0)
                )
                source_objects.append(source_obj)

            # Create metadata
            metadata = ResponseMetadata(
                processing_time=processing_time,
                timestamp=datetime.now(),
                model_used=model_used,
                tokens_used=tokens_used
            )

            # Format the response
            formatted_response = {
                "success": True,
                "response": self._clean_response_text(raw_response),
                "sources": source_objects,
                "metadata": metadata
            }

            self.logger.info(
                "Response formatted successfully",
                query_length=len(query),
                response_length=len(formatted_response["response"]),
                source_count=len(source_objects)
            )

            return formatted_response

        except Exception as e:
            self.logger.error("Error formatting response", error=str(e))
            raise

    def _extract_sources(self, raw_response: str) -> List[Dict[str, Any]]:
        """
        Extract source information from the raw response text.

        Args:
            raw_response: The raw response text from the RAG agent

        Returns:
            List of dictionaries containing source information
        """
        sources = []

        try:
            # Look for common source patterns in the response
            lines = raw_response.split('\n')

            # Flag to know when we're in a sources section
            in_sources_section = False

            for line in lines:
                # Check for sources header
                if re.search(r'(?:^|\n)(?:Sources?|References?|Citations?)[\s:]', line, re.IGNORECASE):
                    in_sources_section = True
                    continue

                # If we're in the sources section, extract URLs and other info
                if in_sources_section:
                    # Look for URLs
                    url_pattern = r'https?://(?:[-\w.])+(?:[:\d]+)?(?:/(?:[\w/_.])*(?:\?(?:[\w&=%.])*)?(?:#(?:\w*))?)?'
                    urls = re.findall(url_pattern, line)

                    for url in urls:
                        # Extract additional context like section title
                        section = self._extract_section_context(line, url)

                        source = {
                            "url": url,
                            "section": section,
                            "chunk_id": self._generate_chunk_id(url, section),
                            "score": 0.8  # Default score for extracted sources
                        }
                        sources.append(source)

                # Alternative: look for sources throughout the entire response
                # not just in designated sections
                if not in_sources_section:
                    url_pattern = r'https?://(?:[-\w.])+(?:[:\d]+)?(?:/(?:[\w/_.])*(?:\?(?:[\w&=%.])*)?(?:#(?:\w*))?)?'
                    urls = re.findall(url_pattern, line)

                    for url in urls:
                        section = self._extract_section_context(line, url)

                        # Avoid duplicates
                        if not any(src['url'] == url for src in sources):
                            source = {
                                "url": url,
                                "section": section,
                                "chunk_id": self._generate_chunk_id(url, section),
                                "score": 0.7  # Slightly lower score for non-designated sources
                            }
                            sources.append(source)

        except Exception as e:
            self.logger.error("Error extracting sources", error=str(e))

        # Limit to top 5 sources to avoid overwhelming the response
        return sources[:5]

    def _extract_section_context(self, line: str, url: str) -> str:
        """
        Extract section or context information related to a URL in the line.

        Args:
            line: The line containing the URL
            url: The URL to extract context for

        Returns:
            Section or context information
        """
        try:
            # Remove the URL from the line to get the surrounding context
            context = line.replace(url, '').strip()

            # Clean up the context by removing common prefixes
            prefixes_to_remove = [
                '- ', '• ', '* ', '1. ', '2. ', '3. ', '4. ', '5. ',
                'Source: ', 'Reference: ', 'Citation: '
            ]

            for prefix in prefixes_to_remove:
                if context.startswith(prefix):
                    context = context[len(prefix):].strip()

            # Limit to first 100 characters and remove any trailing punctuation
            context = context[:100].rstrip('.!?:,;')

            # If context looks like it might be a section title, return it
            # Otherwise return a generic "Referenced content" indication
            if context and len(context) > 5:
                return context
            else:
                return "Referenced content"

        except Exception as e:
            self.logger.error("Error extracting section context", error=str(e))
            return "Referenced content"

    def _generate_chunk_id(self, url: str, section: str) -> str:
        """
        Generate a unique chunk ID based on URL and section.

        Args:
            url: The URL of the source
            section: The section title

        Returns:
            Generated chunk ID
        """
        try:
            import hashlib

            # Create a hash of the URL and section to generate a unique ID
            combined = f"{url}:{section}".encode('utf-8')
            hash_obj = hashlib.md5(combined)
            return f"chunk_{hash_obj.hexdigest()[:8]}"
        except Exception as e:
            self.logger.error("Error generating chunk ID", error=str(e))
            return "unknown_chunk"

    def _clean_response_text(self, raw_response: str) -> str:
        """
        Clean up the raw response text by removing source citations that might
        interfere with the main response content.

        Args:
            raw_response: The raw response text

        Returns:
            Cleaned response text
        """
        try:
            # Split into lines
            lines = raw_response.split('\n')

            # Look for the sources section and exclude it from the main response
            cleaned_lines = []
            in_sources_section = False

            for line in lines:
                # Check if this line starts a sources section
                if re.search(r'(?:^|\n)(?:Sources?|References?|Citations?)[\s:]', line, re.IGNORECASE):
                    in_sources_section = True
                    # Don't include the sources header line in the response
                    continue

                # If we're in the sources section, skip the line
                if in_sources_section:
                    continue

                # Otherwise, add the line to the cleaned response
                cleaned_lines.append(line)

            # Join the cleaned lines back together
            cleaned_response = '\n'.join(cleaned_lines)

            # Remove any trailing blank lines
            cleaned_response = cleaned_response.rstrip('\n ')

            return cleaned_response

        except Exception as e:
            self.logger.error("Error cleaning response text", error=str(e))
            # If cleaning fails, return the original response
            return raw_response

    def format_error_response(
        self,
        error_message: str,
        error_code: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Format an error response.

        Args:
            error_message: The main error message
            error_code: Optional error code
            details: Optional additional error details

        Returns:
            Dictionary with formatted error response data
        """
        try:
            error_response = {
                "success": False,
                "error": error_message,
                "error_code": error_code,
                "details": details or {}
            }

            self.logger.error(
                "Error response formatted",
                error_message=error_message,
                error_code=error_code
            )

            return error_response

        except Exception as e:
            self.logger.error("Error formatting error response", error=str(e))
            raise