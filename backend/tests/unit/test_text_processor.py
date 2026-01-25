"""
Unit tests for utility functions.
"""
import unittest
from src.lib.utils import clean_text, chunk_text, count_tokens, extract_title_from_html


class TestTextUtils(unittest.TestCase):
    
    def test_clean_text(self):
        """Test text cleaning functionality."""
        # Arrange
        raw_text = "  This   is  a   test.\n\n\nMultiple    spaces\tand\nnewlines.  "

        # Act
        cleaned = clean_text(raw_text)

        # Assert
        self.assertEqual(cleaned, "This is a test. Multiple spaces and newlines.")
    
    def test_clean_text_empty(self):
        """Test cleaning empty text."""
        # Arrange
        raw_text = ""
        
        # Act
        cleaned = clean_text(raw_text)
        
        # Assert
        self.assertEqual(cleaned, "")
    
    def test_chunk_text(self):
        """Test text chunking functionality."""
        # Arrange
        text = "This is a sample text for chunking. " * 5  # 5 repetitions
        chunk_size = 20
        
        # Act
        chunks = chunk_text(text, chunk_size, overlap=5)
        
        # Assert
        self.assertGreater(len(chunks), 1)  # Should be split into multiple chunks
        for chunk in chunks:
            self.assertLessEqual(len(chunk), chunk_size + 5)  # Allow for overlap
    
    def test_chunk_text_no_overlap(self):
        """Test text chunking without overlap."""
        # Arrange
        text = "This is a sample text for chunking."
        chunk_size = 10
        
        # Act
        chunks = chunk_text(text, chunk_size, overlap=0)
        
        # Assert
        self.assertGreater(len(chunks), 1)
        for chunk in chunks:
            self.assertLessEqual(len(chunk), chunk_size)
    
    def test_chunk_text_invalid_params(self):
        """Test chunking with invalid parameters."""
        # Test with negative chunk size
        with self.assertRaises(ValueError):
            chunk_text("test", -1)
        
        # Test with overlap >= chunk_size
        with self.assertRaises(ValueError):
            chunk_text("test", 5, overlap=10)
    
    def test_count_tokens(self):
        """Test token counting functionality."""
        # Arrange
        text = "This is a sample text for token counting."
        
        # Act
        token_count = count_tokens(text)
        
        # Assert
        # The approximation is 1 token ~ 4 characters
        expected_tokens = len(text) // 4
        self.assertEqual(token_count, expected_tokens)
    
    def test_count_tokens_empty(self):
        """Test token counting for empty text."""
        # Arrange
        text = ""
        
        # Act
        token_count = count_tokens(text)
        
        # Assert
        self.assertEqual(token_count, 0)
    
    def test_extract_title_from_html(self):
        """Test title extraction from HTML."""
        # Arrange
        html_with_title = "<html><head><title>Test Title</title></head><body>Content</body></html>"
        
        # Act
        title = extract_title_from_html(html_with_title)
        
        # Assert
        self.assertEqual(title, "Test Title")
    
    def test_extract_title_from_h1(self):
        """Test title extraction from h1 when no title tag exists."""
        # Arrange
        html_with_h1 = "<html><body><h1>Header Title</h1>Content</body></html>"
        
        # Act
        title = extract_title_from_html(html_with_h1)
        
        # Assert
        self.assertEqual(title, "Header Title")
    
    def test_extract_title_none(self):
        """Test title extraction when no title or h1 exists."""
        # Arrange
        html_without_title = "<html><body>Content</body></html>"
        
        # Act
        title = extract_title_from_html(html_without_title)
        
        # Assert
        self.assertEqual(title, "")


if __name__ == '__main__':
    unittest.main()