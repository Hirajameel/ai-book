"""
Utility functions for the RAG Agent API

This module contains utility functions for error handling, logging, and other
common operations needed across the API.
"""

import logging
from typing import Any, Dict, Optional
from fastapi import HTTPException
from datetime import datetime
import traceback
import sys
import uuid


class APIErrorHandler:
    """
    Utility class for handling API errors consistently
    """

    @staticmethod
    def log_error(error: Exception, context: str = "", request_data: Optional[Dict] = None):
        """
        Log error with context and request data
        """
        logger = logging.getLogger(__name__)
        logger.error(f"Error in {context}: {str(error)}")
        logger.error(f"Traceback: {traceback.format_exc()}")
        if request_data:
            logger.error(f"Request data: {request_data}")

    @staticmethod
    def create_error_response(error_type: str, message: str, status_code: int = 500) -> Dict[str, Any]:
        """
        Create a standardized error response
        """
        return {
            "success": False,
            "error": {
                "type": error_type,
                "message": message,
                "timestamp": datetime.now().isoformat(),
                "status_code": status_code
            }
        }

    @staticmethod
    def handle_validation_error(detail: str) -> HTTPException:
        """
        Create a standardized validation error
        """
        return HTTPException(
            status_code=422,
            detail=f"Validation Error: {detail}"
        )

    @staticmethod
    def handle_bad_request(detail: str) -> HTTPException:
        """
        Create a standardized bad request error
        """
        return HTTPException(
            status_code=400,
            detail=f"Bad Request: {detail}"
        )

    @staticmethod
    def handle_not_found(detail: str) -> HTTPException:
        """
        Create a standardized not found error
        """
        return HTTPException(
            status_code=404,
            detail=f"Not Found: {detail}"
        )

    @staticmethod
    def handle_internal_error(detail: str) -> HTTPException:
        """
        Create a standardized internal error
        """
        return HTTPException(
            status_code=500,
            detail=f"Internal Server Error: {detail}"
        )


def setup_logging():
    """
    Set up basic logging configuration
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def generate_query_id() -> str:
    """
    Generate a unique query ID
    """
    return f"query_{uuid.uuid4().hex[:8]}"


def validate_query_text(query: str) -> bool:
    """
    Validate query text meets requirements
    """
    if not query or len(query.strip()) == 0:
        return False
    if len(query) > 2000:  # Max length from spec
        return False
    return True


def validate_context_type(context_type: str) -> bool:
    """
    Validate context type is valid
    """
    return context_type in ["full_book", "selected_text"]


def validate_selected_text(selected_text: Optional[str], context_type: str) -> bool:
    """
    Validate selected text based on context type
    """
    if context_type == "selected_text":
        return selected_text is not None and len(selected_text.strip()) > 0
    return True  # Not required for full_book context