"""
Crawler module for RAG Ingestion Pipeline
Handles URL crawling, text extraction, and content processing
"""
import logging
import requests
from bs4 import BeautifulSoup
from urllib.robotparser import RobotFileParser
from urllib.parse import urljoin, urlparse
import time
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple
from models import Document, DocumentStatus
from config import CrawlConfig


def is_sitemap_url(url: str) -> bool:
    """Check if a URL is a sitemap.xml file"""
    parsed = urlparse(url)
    return 'sitemap' in parsed.path.lower() and (url.lower().endswith('.xml') or 'sitemap' in url.lower())


def parse_sitemap(sitemap_url: str) -> List[str]:
    """Parse a sitemap.xml file and extract all URLs"""
    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (compatible; RAG-Ingestion-Bot/1.0; +http://example.com/bot)'
        }

        response = requests.get(sitemap_url, headers=headers, timeout=30)
        response.raise_for_status()

        # Parse the XML content
        root = ET.fromstring(response.content)

        # Handle both regular sitemaps and sitemap indexes
        urls = []

        # Check if it's a sitemap index (contains sitemap elements)
        sitemap_namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        sitemap_elements = root.findall('sitemap:sitemap/sitemap:loc', sitemap_namespace)

        if sitemap_elements:
            # This is a sitemap index, need to fetch individual sitemaps
            for sitemap_elem in sitemap_elements:
                sitemap_loc = sitemap_elem.text.strip()
                urls.extend(parse_sitemap(sitemap_loc))
        else:
            # This is a regular sitemap, extract URLs
            url_namespace = {'url': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
            url_elements = root.findall('url:url/url:loc', url_namespace)

            for url_elem in url_elements:
                url = url_elem.text.strip()
                if url:
                    urls.append(url)

        logging.info(f"Parsed sitemap {sitemap_url}, found {len(urls)} URLs")
        return urls
    except Exception as e:
        logging.error(f"Error parsing sitemap {sitemap_url}: {str(e)}")
        return []


def create_url_validator():
    """Create utility functions for URL validation and processing"""
    import re
    from urllib.parse import urlparse

    def is_valid_url(url: str) -> bool:
        """Validate if a string is a proper URL"""
        try:
            result = urlparse(url)
            return all([result.scheme, result.netloc])
        except Exception:
            return False

    def normalize_url(url: str) -> str:
        """Normalize URL to standard format"""
        if not url.startswith(('http://', 'https://')):
            url = 'https://' + url
        return url.rstrip('/')

    return is_valid_url, normalize_url


def check_robots_txt(url: str, user_agent: str = "*") -> bool:
    """Check if URL is allowed by robots.txt"""
    try:
        parsed_url = urlparse(url)
        robots_url = f"{parsed_url.scheme}://{parsed_url.netloc}/robots.txt"

        rp = RobotFileParser()
        rp.set_url(robots_url)
        rp.read()

        return rp.can_fetch(user_agent, url)
    except:
        # If robots.txt is not accessible, assume we can crawl
        return True


def extract_text_from_html(html_content: str, url: str = "") -> Tuple[str, str]:
    """Extract clean text from HTML content using BeautifulSoup"""
    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove script and style elements
    for script in soup(["script", "style"]):
        script.decompose()

    # Try to extract the main content from Docusaurus pages
    # Look for main content containers that are typical in Docusaurus
    main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='container')

    if main_content:
        content_text = main_content.get_text()
    else:
        # Fallback to body content
        body = soup.find('body')
        if body:
            content_text = body.get_text()
        else:
            content_text = soup.get_text()

    # Clean up the text
    lines = (line.strip() for line in content_text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    text = ' '.join(chunk for chunk in chunks if chunk)

    # Clean text to handle problematic Unicode characters
    # Remove or replace zero-width space and other problematic characters
    text = text.replace('\u200b', '')  # Zero-width space
    text = text.replace('\u200c', '')  # Zero-width non-joiner
    text = text.replace('\u200d', '')  # Zero-width joiner
    text = text.replace('\ufeff', '')  # Byte order mark

    # Replace other problematic Unicode characters with standard equivalents
    text = text.replace('\u2013', '-')  # en dash
    text = text.replace('\u2014', '-')  # em dash
    text = text.replace('\u2018', "'")  # left single quotation mark
    text = text.replace('\u2019', "'")  # right single quotation mark
    text = text.replace('\u201c', '"')  # left double quotation mark
    text = text.replace('\u201d', '"')  # right double quotation mark

    # Limit text length to prevent memory issues
    if len(text) > 100000:  # Limit to 100k characters
        logging.warning(f"Truncating content from {url} due to length: {len(text)} characters")
        text = text[:100000]

    # Extract the main heading if available
    main_heading = ""
    if soup.find('h1'):
        main_heading = soup.find('h1').get_text().strip()
    elif soup.find('title'):
        main_heading = soup.find('title').get_text().strip()

    return text, main_heading


def crawl_url(url: str, crawl_config: CrawlConfig, delay: float = 1.0) -> Optional[Document]:
    """Implement URL crawling function with requests library respecting robots.txt"""
    try:
        # Check robots.txt
        if crawl_config.respect_robots_txt and not check_robots_txt(url):
            logging.warning(f"Robots.txt disallows crawling {url}")
            return None

        # Add delay to respect rate limits
        time.sleep(delay)

        headers = {
            'User-Agent': 'Mozilla/5.0 (compatible; RAG-Ingestion-Bot/1.0; +http://example.com/bot)'
        }

        response = requests.get(url, headers=headers, timeout=30)
        response.raise_for_status()

        # Extract text and heading
        content_text, section_heading = extract_text_from_html(response.text, url)

        # Create document
        doc = Document(
            id=f"doc_{hash(url)}",  # Simple ID generation
            source_url=url,
            content_text=content_text,
            section_heading=section_heading,
            processing_status=DocumentStatus.COMPLETED
        )

        logging.info(f"Successfully crawled {url}")
        return doc

    except requests.exceptions.RequestException as e:
        logging.error(f"Error crawling {url}: {str(e)}")
        # Create a failed document
        doc = Document(
            id=f"doc_{hash(url)}",
            source_url=url,
            processing_status=DocumentStatus.FAILED
        )
        return doc
    except Exception as e:
        logging.error(f"Unexpected error crawling {url}: {str(e)}")
        # Create a failed document
        doc = Document(
            id=f"doc_{hash(url)}",
            source_url=url,
            processing_status=DocumentStatus.FAILED
        )
        return doc


def crawl_docusaurus_site(crawl_config: CrawlConfig) -> List[Document]:
    """Implement navigation through Docusaurus site structure with proper URL handling"""
    documents = []
    all_urls = []

    for url in crawl_config.target_urls:
        if not url:
            continue

        # Check if it's a sitemap URL
        if is_sitemap_url(url):
            logging.info(f"Detected sitemap URL: {url}, parsing...")
            sitemap_urls = parse_sitemap(url)
            all_urls.extend(sitemap_urls)
        else:
            # Validate URL
            is_valid_url, normalize_url = create_url_validator()
            if not is_valid_url(url):
                logging.error(f"Invalid URL: {url}")
                continue

            # Normalize URL
            normalized_url = normalize_url(url)
            all_urls.append(normalized_url)

    # Crawl all URLs (either from sitemap or original list)
    for url in all_urls:
        # Crawl the URL
        doc = crawl_url(url, crawl_config, crawl_config.crawl_delay)
        if doc:
            documents.append(doc)

    return documents


def chunk_content(content: str, chunk_size: int = 1000, overlap: int = 100) -> List[dict]:
    """Implement content chunking function with configurable chunk size and overlap"""
    if not content:
        logging.info("Content is empty, returning empty chunks")
        return []

    chunks = []
    start = 0
    content_length = len(content)

    logging.info(f"Starting chunking: content_length={content_length}, chunk_size={chunk_size}, overlap={overlap}")

    chunk_count = 0
    max_chunks = content_length // max(1, chunk_size - overlap) + 1  # Upper bound to prevent infinite loops

    while start < content_length and chunk_count < max_chunks:
        end = start + chunk_size

        # If this is the last chunk and it's smaller than chunk_size, include it
        if end > content_length:
            end = content_length

        chunk_text = content[start:end]

        # Debug logging for each chunk
        logging.debug(f"Chunk {chunk_count}: start={start}, end={end}, length={len(chunk_text)}")

        chunks.append({
            'text': chunk_text,
            'start_idx': start,
            'end_idx': end
        })

        # Move start position by chunk_size - overlap to create overlapping chunks
        # Ensure we advance the position to prevent infinite loops
        if chunk_size > overlap:
            start = end - overlap
        else:
            # If overlap is >= chunk_size, move by chunk_size to prevent infinite loop
            start = end

        # Additional safety check to prevent infinite loop
        if start <= 0:
            start += 1
        elif start >= content_length:
            break  # We've reached the end of content

        chunk_count += 1

        # Safety check to prevent infinite loops
        if chunk_count >= max_chunks:
            logging.warning(f"Reached maximum chunk count ({max_chunks}), stopping to prevent infinite loop")
            break

    logging.info(f"Chunking completed: {len(chunks)} chunks created")

    # Format chunks to include only the text
    return [{'text': chunk['text']} for chunk in chunks]


def preserve_metadata_for_chunk(original_doc: Document, chunk_text: str, chunk_index: int, total_chunks: int) -> dict:
    """Create metadata preservation function to maintain source URL and section context"""
    metadata = {
        'source_url': original_doc.source_url,
        'section_heading': original_doc.section_heading,
        'chunk_index': chunk_index,
        'total_chunks': total_chunks,
        'original_doc_id': original_doc.id,
        'created_at': original_doc.created_at.isoformat() if original_doc.created_at else None
    }
    return metadata


def validate_chunk_semantics(chunk_text: str) -> bool:
    """Add validation to ensure chunks maintain semantic meaning"""
    # Basic validation: chunk should have meaningful content
    if not chunk_text or len(chunk_text.strip()) == 0:
        return False

    # Check if chunk has enough content to be meaningful
    words = chunk_text.split()
    if len(words) < 5:  # At least 5 words for semantic meaning
        return False

    return True