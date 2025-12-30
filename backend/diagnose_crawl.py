import requests
from bs4 import BeautifulSoup

# Test the specific extraction function from the crawler
def extract_text_from_html(html_content, url=""):
    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove script and style elements
    for script in soup(["script", "style"]):
        script.decompose()

    # Try to extract the main content from Docusaurus pages
    # Look for main content containers that are typical in Docusaurus
    main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='container')

    print(f"Found main content element: {main_content is not None}")

    if main_content:
        content_text = main_content.get_text()
        print(f"Content from main element: {len(content_text)} characters")
    else:
        # Fallback to body content
        body = soup.find('body')
        if body:
            content_text = body.get_text()
            print(f"Content from body element: {len(content_text)} characters")
        else:
            content_text = soup.get_text()
            print(f"Content from full soup: {len(content_text)} characters")

    # Clean up the text
    lines = (line.strip() for line in content_text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    text = ' '.join(chunk for chunk in chunks if chunk)

    # Extract the main heading if available
    main_heading = ""
    if soup.find('h1'):
        main_heading = soup.find('h1').get_text().strip()
        print(f"H1 heading: {main_heading}")
    elif soup.find('title'):
        main_heading = soup.find('title').get_text().strip()
        print(f"Title: {main_heading}")

    return text, main_heading

# Test crawling the site
url = "https://hirajameel.github.io/ai-book/"
headers = {
    'User-Agent': 'Mozilla/5.0 (compatible; RAG-Ingestion-Bot/1.0; +http://example.com/bot)'
}

print(f"Attempting to crawl: {url}")
response = requests.get(url, headers=headers)
print(f"Status code: {response.status_code}")
print(f"Content length: {len(response.text)}")

# Test the extraction function
extracted_text, heading = extract_text_from_html(response.text, url)
print(f"Extracted text length: {len(extracted_text)}")
print(f"Heading: {heading}")

# Check if the text is meaningful
if len(extracted_text) > 10:
    print("SUCCESS: Content extraction worked!")
    print(f"First 200 chars: {extracted_text[:200]}...")
else:
    print("ISSUE: Content extraction failed or returned very little content")