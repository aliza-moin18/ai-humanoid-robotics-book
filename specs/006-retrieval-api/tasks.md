# Implementation Tasks: Retrieval API for Book Content Queries

**Feature**: 006-retrieval-api | **Date**: 2026-01-08 | **Spec**: specs/006-retrieval-api/spec.md

**Input**: Feature specification from `/specs/006-retrieval-api/spec.md` and design artifacts

## Summary

Implementation of a RetrievalService using Cohere embeddings and Qdrant vector search to enable fast and accurate retrieval of book content chunks. The service exposes a FastAPI /query endpoint that supports both general queries and queries with optional selected/highlighted text context, returning the top-5 most relevant chunks with full text, source URL/section, and relevance scores.

## Dependencies

- User Story 2 depends on User Story 1 (requires core retrieval functionality)
- User Story 3 is integrated into User Story 1 (returns chunk information)

## Parallel Execution Examples

- [P] T002-T004 can be executed in parallel (setting up models, service, and API)
- [P] T010-T012 can be executed in parallel (unit tests for different components)

## Implementation Strategy

MVP scope includes User Story 1 (core query functionality) with minimal viable implementation of the retrieval service. Subsequent stories enhance functionality with contextual queries and detailed chunk information.

---

## Phase 1: Setup

**Goal**: Initialize project structure and install dependencies

- [X] T001 Create project directory structure per implementation plan
- [X] T002 Install required dependencies (FastAPI, Cohere, Qdrant, Pydantic)
- [ ] T003 Set up environment variables for Cohere API key and Qdrant connection

## Phase 2: Foundational

**Goal**: Implement core models and service foundation

- [X] T004 Create QueryRequest model in retrieve.py with validation rules
- [X] T005 Create Chunk model in retrieve.py with validation rules
- [X] T006 Create QueryResponse model in retrieve.py with validation rules
- [X] T007 Implement CohereClient initialization in RetrievalService
- [X] T008 Implement QdrantClient initialization in RetrievalService
- [X] T009 Create embed_text method in RetrievalService using Cohere

## Phase 3: [US1] Query Book Content

**Goal**: Enable developers to query book content API and retrieve relevant chunks based on user questions

**Independent Test**: Send a query to the API and verify it returns relevant book chunks with proper metadata within performance requirements (<1 second)

- [X] T010 [P] [US1] Implement search method in RetrievalService for similarity search
- [X] T011 [P] [US1] Create FastAPI /query endpoint in retrieve.py
- [X] T012 [US1] Add request/response validation to /query endpoint
- [X] T013 [US1] Connect /query endpoint to RetrievalService search method
- [X] T014 [US1] Implement top-5 similarity search functionality
- [X] T015 [US1] Add performance monitoring to measure query time
- [ ] T016 [US1] Test with "ROS2 fundamentals" query to verify relevant results
- [ ] T017 [US1] Test with "digital twin simulation" query to verify <1 second response

## Phase 4: [US2] Query with Selected Text Context

**Goal**: Allow developers to provide selected/highlighted text context along with queries to prioritize relevant results

**Independent Test**: Send a query with selected text context and verify results are more contextually relevant than without context

- [X] T018 [US2] Modify embed_text method to handle combined user_query and selected_text
- [X] T019 [US2] Update search method to incorporate selected_text context
- [ ] T020 [US2] Test with "navigation" query and "ROS2 navigation stack" selected text
- [ ] T021 [US2] Verify results are specifically about ROS2 navigation rather than general navigation

## Phase 5: [US3] Access Detailed Chunk Information

**Goal**: Provide developers with detailed information about each retrieved chunk for proper attribution and display

**Independent Test**: Verify each returned chunk includes full text, source URL/section, and relevance score

- [X] T022 [US3] Validate Chunk model contains text, source_url, and relevance_score fields
- [X] T023 [US3] Ensure search results map correctly to Chunk model with all required fields
- [X] T024 [US3] Test that each chunk in response contains full text content
- [X] T025 [US3] Test that each chunk in response contains valid source URL/section
- [X] T026 [US3] Test that each chunk in response contains relevance score between 0 and 1

## Phase 6: Testing & Validation

**Goal**: Implement comprehensive tests to validate all functionality

- [X] T027 Create unit tests for QueryRequest model validation
- [X] T028 Create unit tests for Chunk model validation
- [X] T029 Create unit tests for QueryResponse model validation
- [X] T030 Create unit tests for embed_text functionality
- [X] T031 Create unit tests for search functionality
- [ ] T032 Create integration tests for /query endpoint
- [ ] T033 Create contract tests for API response schema
- [ ] T034 Run performance tests to verify <1 second response time
- [ ] T035 Run edge case tests (long queries, no matches, invalid inputs)

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Complete implementation with documentation, error handling, and optimizations

- [X] T036 Add comprehensive error handling and appropriate HTTP status codes
- [X] T037 Add logging for debugging and monitoring purposes
- [X] T038 Add input sanitization to prevent injection attacks
- [X] T039 Update README with usage instructions
- [X] T040 Add health check endpoint
- [ ] T041 Optimize performance for concurrent requests
- [ ] T042 Document API endpoints with OpenAPI/Swagger
- [ ] T043 Add configuration options for Qdrant connection
- [ ] T044 Run full test suite to validate all functionality