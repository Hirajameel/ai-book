# Data Model: RAG Agent Construction

## Entities

### UserQuery
- **Description**: Natural language question submitted by the user to the agent
- **Fields**:
  - query_text: string (the actual question from the user)
  - timestamp: datetime (when the query was submitted)
  - session_id: string (to track conversation context)
  - user_id: string (optional, to track user-specific interactions)

### RetrievedContentChunk
- **Description**: Relevant text segment retrieved from Qdrant vector database based on semantic similarity
- **Fields**:
  - id: string (unique identifier from vector database)
  - content: string (the actual text content retrieved)
  - similarity_score: float (relevance score from vector search)
  - metadata: object (source information like URL, section heading)
  - source_url: string (URL where the content originated)
  - section_heading: string (heading of the section containing the content)
  - chunk_index: integer (position of this chunk within the original document)

### AgentResponse
- **Description**: Generated answer that is grounded in retrieved content with citations
- **Fields**:
  - response_text: string (the agent's answer to the user)
  - retrieved_chunks: array[RetrievedContentChunk] (content used to generate the response)
  - citations: array[object] (structured citation information)
  - timestamp: datetime (when the response was generated)
  - query_id: string (link back to the original user query)

### SourceCitation
- **Description**: Reference to specific section headings and URLs from the original content
- **Fields**:
  - url: string (URL of the source content)
  - section: string (section heading of the source)
  - content_preview: string (brief preview of the cited content)
  - relevance_score: float (similarity score from vector search)

## Relationships

- One `UserQuery` generates one `AgentResponse`
- One `AgentResponse` references multiple `RetrievedContentChunk` items
- One `RetrievedContentChunk` can be referenced by multiple `AgentResponse` items
- One `AgentResponse` contains multiple `SourceCitation` items
- Each `SourceCitation` corresponds to a `RetrievedContentChunk`

## Validation Rules

- `UserQuery.query_text` must not be empty
- `RetrievedContentChunk.similarity_score` must be between 0 and 1
- `AgentResponse.retrieved_chunks` must not be empty (ensures grounding in content)
- `SourceCitation.url` must be a valid URL format
- `RetrievedContentChunk.content` must not be empty when used in a response

## State Transitions

- `UserQuery` transitions from "received" to "processed" when the retrieval is complete
- `AgentResponse` transitions from "generating" to "completed" when the response is ready
- `RetrievedContentChunk` state remains "retrieved" once fetched from vector database