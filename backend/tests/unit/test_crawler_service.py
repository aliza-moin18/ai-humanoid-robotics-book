"""
Unit tests for CrawlerService.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
from src.services.crawler_service import CrawlerService
from src.models.book_content import BookContent
from src.models.text_chunk import TextChunk
from src.lib.errors import ContentExtractionError


class TestCrawlerService(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.crawler_service = CrawlerService(delay=0)  # Set delay to 0 for faster tests
    
    @patch('src.services.crawler_service.requests.Session.get')
    def test_fetch_content_success(self, mock_get):
        """Test successful content fetching."""
        # Arrange
        mock_response = Mock()
        mock_response.text = "<html><body>Test content</body></html>"
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response
        test_url = "https://example.com"
        
        # Act
        result = self.crawler_service.fetch_content(test_url)
        
        # Assert
        self.assertEqual(result, "<html><body>Test content</body></html>")
        mock_get.assert_called_once_with(test_url, timeout=30)
    
    @patch('src.services.crawler_service.requests.Session.get')
    def test_fetch_content_request_exception(self, mock_get):
        """Test content fetching with request exception."""
        # Arrange
        from requests.exceptions import RequestException
        mock_get.side_effect = RequestException("Network error")
        test_url = "https://example.com"

        # Act & Assert
        with self.assertRaises(ContentExtractionError):
            self.crawler_service.fetch_content(test_url)
    
    def test_extract_text(self):
        """Test text extraction from HTML."""
        # Arrange
        html_content = """
        <html>
            <head><title>Test Title</title></head>
            <body>
                <p>This is a test paragraph.</p>
                <script>console.log('test');</script>
                <style>body { color: red; }</style>
            </body>
        </html>
        """
        test_url = "https://example.com"
        
        # Act
        result = self.crawler_service.extract_text(html_content, test_url)
        
        # Assert
        self.assertIn("This is a test paragraph.", result)
        self.assertNotIn("console.log", result)
        self.assertNotIn("color: red", result)
    
    def test_chunk_content(self):
        """Test content chunking."""
        # Arrange
        book_content = BookContent(
            id="test-id",
            source_url="https://example.com",
            content="This is a sample content for testing chunking functionality. " * 10,  # Make it longer
            title="Test Book"
        )
        
        # Act
        chunks = self.crawler_service.chunk_content(book_content, chunk_size=50, overlap=10)
        
        # Assert
        self.assertGreater(len(chunks), 0)
        for chunk in chunks:
            self.assertIsInstance(chunk, TextChunk)
            self.assertEqual(chunk.book_content_id, "test-id")
            self.assertLessEqual(len(chunk.content), 50)
    
    def test_create_book_content(self):
        """Test creation of BookContent object."""
        # Arrange
        test_url = "https://example.com/book"
        test_content = "<html><head><title>Test Book Title</title></head><body>Book content here.</body></html>"
        
        # Act
        result = self.crawler_service.create_book_content(test_url, test_content)
        
        # Assert
        self.assertIsInstance(result, BookContent)
        self.assertEqual(result.source_url, test_url)
        self.assertEqual(result.content, test_content)
        self.assertEqual(result.title, "Test Book Title")
        self.assertGreater(result.word_count, 0)
    
    @patch('src.services.crawler_service.CrawlerService.fetch_content')
    @patch('src.services.crawler_service.CrawlerService.extract_text')
    @patch('src.services.crawler_service.CrawlerService.create_book_content')
    def test_crawl(self, mock_create_book, mock_extract_text, mock_fetch_content):
        """Test the complete crawl method."""
        # Arrange
        test_url = "https://example.com"
        mock_fetch_content.return_value = "<html>content</html>"
        mock_extract_text.return_value = "cleaned content"
        expected_book_content = BookContent(
            id="test-id",
            source_url=test_url,
            content="cleaned content",
            title="Test"
        )
        mock_create_book.return_value = expected_book_content

        # Act
        result = self.crawler_service.crawl(test_url)

        # Assert
        self.assertEqual(result, expected_book_content)
        mock_fetch_content.assert_called_once_with(test_url)
        mock_extract_text.assert_called_once_with("<html>content</html>", test_url)
        mock_create_book.assert_called_once_with(test_url, "cleaned content")


if __name__ == '__main__':
    unittest.main()