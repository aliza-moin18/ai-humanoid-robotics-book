"""
Utility functions for text processing in the book embeddings ingestion system.
"""
import re
from typing import List


def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace, normalizing line breaks, etc.
    
    Args:
        text: Raw text to clean
        
    Returns:
        Cleaned text
    """
    if not text:
        return ""
    
    # Replace multiple whitespace characters with a single space
    text = re.sub(r'\s+', ' ', text)
    
    # Remove leading/trailing whitespace
    text = text.strip()
    
    # Normalize line breaks
    text = re.sub(r'\n+', '\n', text)
    
    return text


def chunk_text(text: str, chunk_size: int, overlap: int = 0) -> List[str]:
    """
    Split text into chunks of specified size with optional overlap.
    
    Args:
        text: Text to chunk
        chunk_size: Size of each chunk (in characters)
        overlap: Number of characters to overlap between chunks
        
    Returns:
        List of text chunks
    """
    if not text:
        return []
    
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    
    if overlap >= chunk_size:
        raise ValueError("overlap must be less than chunk_size")
    
    chunks = []
    start = 0
    
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)
        
        # Move start position by (chunk_size - overlap)
        start = end - overlap
        
        # If we're at the end and have a small remaining piece, include it
        if start < len(text) <= end:
            # If the remaining text is too small compared to chunk_size, 
            # we can append it to the last chunk
            remaining = text[start:]
            if len(remaining) > chunk_size // 2:  # If remaining is more than half chunk size
                chunks.append(remaining)
                break
    
    # Remove any empty chunks
    chunks = [chunk for chunk in chunks if chunk.strip()]
    
    return chunks


def count_tokens(text: str) -> int:
    """
    Estimate the number of tokens in the text.
    This is a simple approximation - 1 token is roughly 4 characters.
    
    Args:
        text: Text to count tokens for
        
    Returns:
        Estimated number of tokens
    """
    if not text:
        return 0
    
    # Simple approximation: 1 token ~ 4 characters
    # This is a rough estimate; for more accurate counting, use a tokenizer
    return len(text) // 4


def extract_title_from_html(html_content: str) -> str:
    """
    Extract title from HTML content if available.
    
    Args:
        html_content: HTML content to extract title from
        
    Returns:
        Extracted title or empty string if not found
    """
    from bs4 import BeautifulSoup
    
    soup = BeautifulSoup(html_content, 'html.parser')
    title_tag = soup.find('title')
    
    if title_tag:
        return title_tag.get_text().strip()
    
    # Try h1 as alternative
    h1_tag = soup.find('h1')
    if h1_tag:
        return h1_tag.get_text().strip()
    
    return ""