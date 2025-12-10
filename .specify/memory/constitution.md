<!--
Sync Impact Report:
Version change: None -> 1.0.0 (MAJOR: Initial creation, establishing foundational principles and structure)
Modified principles:
  - PRINCIPLE_1_NAME -> I. Technical Accuracy and Reproducibility
  - PRINCIPLE_2_NAME -> II. Clear and Structured Explanations
  - PRINCIPLE_3_NAME -> III. Source-Grounded Content
  - PRINCIPLE_4_NAME -> (Removed - content moved to Key Standards)
  - PRINCIPLE_5_NAME -> (Removed - content moved to Key Standards)
  - PRINCIPLE_6_NAME -> (Removed - not used)
Added sections:
  - Key Standards
  - Constraints and Requirements
Removed sections:
  - (None, placeholders replaced or content absorbed)
Templates requiring updates:
  - .specify/templates/plan-template.md (⚠ pending - check for alignment with new principles and sections)
  - .specify/templates/spec-template.md (⚠ pending - check for scope/requirements alignment)
  - .specify/templates/tasks-template.md (⚠ pending - check for task categorization alignment)
  - .specify/templates/commands/*.md (⚠ pending - verify no outdated references, especially agent-specific names)
Follow-up TODOs: None
-->
# Unified Book + RAG Chatbot + Physical AI & Humanoid Robotics (GIAIC) Constitution

## Core Principles

### I. Technical Accuracy and Reproducibility
All content and code related to ROS 2, Gazebo, NVIDIA Isaac, and general AI systems MUST ensure technical accuracy and reproducibility. Simulations and code examples MUST be verifiable and runnable.

### II. Clear and Structured Explanations
Content MUST be presented with clear, structured explanations suitable for an intermediate-to-advanced Computer Science audience. Explanations MUST be comprehensive and easy to follow.

### III. Source-Grounded Content
All claims and technical information MUST be grounded in authoritative sources, including official documentation, SDKs, or peer-reviewed academic/industry papers.

## Key Standards

- Minimum 40% of all sources MUST be from official documentation or academic/industry references.
- Citation style MUST adhere to either APA or IEEE guidelines.
- NO hallucinated code or API information is permitted. All code examples MUST be functional and accurate.
- Writing MUST be clear, concise, and educational in tone.

## Constraints and Requirements

- The primary output MUST be a multi-chapter book delivered via Docusaurus and integrated with Spec-Kit Plus.
- An embedded RAG (Retrieval Augmented Generation) chatbot MUST be included, utilizing OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant.
- The chatbot's answers MUST be strictly based on the content of the book.
- The book MUST cover four distinct modules: ROS 2, Digital Twin (Gazebo+Unity), NVIDIA Isaac, and VLA + Capstone.
- All provided code and simulations MUST be reproducible and verifiable.

## Governance
This constitution supersedes all other practices and guidelines within the project. Adherence to these principles and standards is mandatory for all contributions. Amendments to this constitution require documentation, review, and approval by project leads, along with a clear migration plan for any affected aspects. All pull requests and code reviews MUST verify compliance with these rules.

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
