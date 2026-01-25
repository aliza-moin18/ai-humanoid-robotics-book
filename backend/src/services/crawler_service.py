"""
Service for crawling and extracting content from book URLs.
"""
import requests
from bs4 import BeautifulSoup
from typing import List
from urllib.parse import urljoin, urlparse
import time

from src.models.book_content import BookContent   # Absolute import (already fixed in main.py)
from src.models.text_chunk import TextChunk
from src.lib.logger import app_logger
from src.lib.errors import ContentExtractionError
from src.lib.utils import clean_text, extract_title_from_html, chunk_text


class CrawlerService:
    """
    Service for crawling and extracting content from book URLs.
    Supports both single pages and full sitemap crawling.
    """

    def __init__(self, delay: float = 1.0):
        self.delay = delay
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; BookIngestionBot/1.0; +https://example.com/bot-info)'
        })

    def fetch_content(self, url: str) -> str:
        try:
            app_logger.info(f"Fetching content from URL: {url}")
            response = self.session.get(url, timeout=30)
            response.raise_for_status()
            time.sleep(self.delay)
            return response.text
        except requests.exceptions.RequestException as e:
            app_logger.error(f"Error fetching content from {url}: {str(e)}")
            raise ContentExtractionError(f"Failed to fetch content from {url}: {str(e)}")

    def extract_text(self, html_content: str, url: str) -> str:
        try:
            soup = BeautifulSoup(html_content, 'html.parser')
            for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                script.decompose()

            text = soup.get_text(separator=" ", strip=True)
            cleaned_text = clean_text(text)
            app_logger.info(f"Extracted {len(cleaned_text)} characters from {url}")
            return cleaned_text
        except Exception as e:
            app_logger.error(f"Error extracting text from {url}: {str(e)}")
            raise ContentExtractionError(f"Failed to extract text from {url}: {str(e)}")

    def create_book_content(self, url: str, content: str) -> BookContent:
        from uuid import uuid4
        title = extract_title_from_html(content)
        word_count = len(content.split())

        book_content = BookContent(
            id=str(uuid4()),
            source_url=url,
            content=content,
            title=title if title else urlparse(url).path.split('/')[-1] or "Untitled",
            word_count=word_count
        )
        app_logger.info(f"Created BookContent with ID {book_content.id} from {url}")
        return book_content

    def _crawl_single_page(self, url: str) -> BookContent:
        html_content = self.fetch_content(url)
        text_content = self.extract_text(html_content, url)
        return self.create_book_content(url, text_content)

    def crawl_from_sitemap(self, sitemap_url: str) -> List[BookContent]:
        app_logger.info(f"Detected sitemap: {sitemap_url}. Fetching all listed URLs...")
        try:
            response = self.session.get(sitemap_url, timeout=30)
            response.raise_for_status()
            soup = BeautifulSoup(response.content, 'xml')
            loc_tags = soup.find_all('loc')
            urls = [loc.text.strip() for loc in loc_tags if loc.text.strip()]

            # Filter only actual book pages (Docusaurus sitemap mein /docs/ ya pages hote hain)
            book_urls = [u for u in urls if '/docs/' in u or u.endswith('/') and not u.endswith('sitemap.xml')]

            app_logger.info(f"Found {len(book_urls)} relevant book pages in sitemap")

            all_book_contents = []
            for i, page_url in enumerate(book_urls, 1):
                app_logger.info(f"[{i}/{len(book_urls)}] Crawling: {page_url}")
                try:
                    book_content = self._crawl_single_page(page_url)
                    all_book_contents.append(book_content)
                    time.sleep(self.delay)
                except Exception as e:
                    app_logger.error(f"Failed to crawl {page_url}: {str(e)}")
                    continue

            app_logger.info(f"Successfully crawled {len(all_book_contents)} pages from sitemap")
            return all_book_contents

        except Exception as e:
            app_logger.error(f"Failed to parse sitemap {sitemap_url}: {str(e)}")
            raise ContentExtractionError(f"Invalid or inaccessible sitemap: {sitemap_url}")

    def crawl(self, url: str) -> List[BookContent]:
        """
        Main crawl method. Always returns List[BookContent]
        """
        app_logger.info(f"Starting crawl of {url}")

        # Auto-detect sitemap
        if url.endswith('sitemap.xml') or '/sitemap' in url.lower():
            return self.crawl_from_sitemap(url)
        
        # Try if there's a sitemap at /sitemap.xml
        base_url = url.rstrip("/") 
        sitemap_url = f"{base_url}/sitemap.xml"
        try:
            test_response = self.session.head(sitemap_url, timeout=10)
            if test_response.status_code == 200:
                app_logger.info(f"Sitemap found at {sitemap_url}, crawling all pages...")
                return self.crawl_from_sitemap(sitemap_url)
        except:
            pass  # No sitemap, continue with single page

        # Otherwise crawl single page
        app_logger.info(f"No sitemap found, crawling single page: {url}")
        single_content = self._crawl_single_page(url)
        return [single_content]

    def chunk_content(self, book_contents: List[BookContent], chunk_size: int, overlap: int = 0) -> List[TextChunk]:
        """
        Updated to accept List[BookContent] and chunk all of them
        """
        from uuid import uuid4
        all_text_chunks = []

        for book_content in book_contents:
            chunks = chunk_text(book_content.content, chunk_size, overlap)
            for i, chunk_text_content in enumerate(chunks):
                text_chunk = TextChunk(
                    id=str(uuid4()),
                    book_content_id=book_content.id,
                    content=chunk_text_content,
                    chunk_index=i,
                    source_url=book_content.source_url,
                    source_location=f"Chunk {i+1}",
                )
                all_text_chunks.append(text_chunk)

        app_logger.info(f"Created total {len(all_text_chunks)} chunks from {len(book_contents)} pages")
        return all_text_chunks