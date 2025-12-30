"""
RAG Ingestion Pipeline: Orchestrates the complete pipeline by importing and coordinating modules.
"""
import os
from config import load_config_from_env, validate_config
from crawler import crawl_docusaurus_site, chunk_content, validate_chunk_semantics
from embedder import create_cohere_client, create_embeddings_from_documents
from storage import create_qdrant_client, create_qdrant_collection, handle_qdrant_storage_errors, verify_storage_success
from utils import setup_logging
from models import Document

def main():
    setup_logging(); crawl_config, embedding_config, qdrant_config = load_config_from_env()
    if not validate_config(crawl_config, embedding_config, qdrant_config): print("Config validation failed. Exiting..."); return
    qdrant_client, cohere_client = create_qdrant_client(qdrant_config), create_cohere_client(embedding_config)
    documents = crawl_docusaurus_site(crawl_config); print(f"Extracted {len(documents)} docs.")
    if not documents: print("No documents extracted"); return
    chunked_docs = []
    for doc in documents:
        if doc.content_text:
            chunks = chunk_content(doc.content_text, int(os.getenv("CHUNK_SIZE", "1000")), int(os.getenv("CHUNK_OVERLAP", "100")))
            for i, chunk in enumerate(chunks):
                if validate_chunk_semantics(chunk['text']):
                    chunk_doc = Document(id=f"{doc.id}_chunk_{i}", source_url=doc.source_url, content_text=chunk['text'], section_heading=doc.section_heading, chunk_index=i)
                    chunked_docs.append(chunk_doc)
    if not chunked_docs: print("No chunked documents created"); return
    embeddings = create_embeddings_from_documents(chunked_docs, cohere_client, embedding_config, qdrant_config)
    if not embeddings: print("No embeddings generated"); return
    create_qdrant_collection(qdrant_client, qdrant_config.collection_name, qdrant_config.vector_size, qdrant_config.distance_metric)
    success = handle_qdrant_storage_errors(qdrant_client, embeddings, qdrant_config.collection_name)
    if success and verify_storage_success(qdrant_client, qdrant_config.collection_name): print("Pipeline completed successfully!")
    else: print("Pipeline failed."); exit(1)

if __name__ == "__main__": main()