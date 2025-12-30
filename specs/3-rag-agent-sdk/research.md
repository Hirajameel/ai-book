# Research: RAG Agent Construction

## Decision: OpenAI Agents SDK Integration Approach
**Rationale**: Using OpenAI's Assistants API allows for creating persistent agents that can maintain conversation context and integrate tools for RAG functionality. This approach aligns with the requirement to create an agent using the OpenAI Agents SDK.

**Alternatives considered**:
- OpenAI Chat Completions API (requires manual context management)
- LangChain agents (adds complexity with additional abstraction layer)
- Custom agent framework (reinventing existing solutions)

## Decision: Retrieval Integration Pattern
**Rationale**: Using a tool-based approach where the RAG retrieval functionality is exposed as a function/tool that the OpenAI agent can call when needed. This allows the agent to dynamically retrieve relevant content based on the user query.

**Alternatives considered**:
- Pre-retrieving content and passing to agent (less dynamic, might not be relevant)
- Embedding retrieval directly in agent (violates separation of concerns)
- Separate retrieval step before agent invocation (loses dynamic adaptation)

## Decision: Cohere Embeddings for Query Processing
**Rationale**: Consistent with the existing retrieval infrastructure (Spec-2) which already uses Cohere embeddings. This ensures compatibility with existing vector database content and maintains consistency in the embedding space.

**Alternatives considered**:
- OpenAI embeddings (would require re-embedding existing content)
- Sentence Transformers (would require additional model hosting)
- Other embedding providers (would require migration from existing setup)

## Decision: Qdrant Vector Database Integration
**Rationale**: Reusing the existing Qdrant infrastructure ensures consistency with the existing retrieval system and avoids data duplication or migration. The existing collection already contains the book content embeddings.

**Alternatives considered**:
- OpenAI embeddings + vector store (would require re-embedding and migration)
- Pinecone (would require new infrastructure)
- Local vector stores (would not match existing infrastructure)

## Decision: Citation Format for Responses
**Rationale**: Citations will be included directly in the agent response using a consistent format that includes section headings and URLs from the retrieved content metadata. This ensures users can verify information and navigate to original content.

**Alternatives considered**:
- Separate citation list at end (less contextual)
- Footnote-style citations (harder to correlate with content)
- No citations (violates functional requirement FR-005)

## Decision: Error Handling for No Results
**Rationale**: When no relevant content is found, the agent will respond with an appropriate message indicating that the requested information is not available in the book content, maintaining the requirement to ground responses only in retrieved content.

**Alternatives considered**:
- Generating responses from general knowledge (violates requirement to use only retrieved content)
- Returning empty responses (poor user experience)
- Suggesting related topics (might lead to hallucination)