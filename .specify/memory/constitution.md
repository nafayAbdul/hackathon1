<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.0.1 (PATCH: added documentation platform standard)
- Modified principles: None
- Added sections: Documentation Platform Standard in Key Standards
- Removed sections: None
- Templates requiring updates: ✅ No updates needed
- Follow-up TODOs: Ratification date still needs to be determined
-->

# Physical AI & Humanoid Robotics Constitution

## Core Principles

### AI-Native Documentation
The textbook's primary purpose is to serve as the authoritative knowledge base for the RAG Chatbot. Documentation must be generated and structured using AI-native tools (e.g., Claude Code/Spec-Kit Plus) to ensure efficiency and integration readiness.

### Actionable Knowledge Base
The book must be optimized for machine readability and retrieval. Content must be clear, granular, and easily translatable into a structured database to maximize the performance of the integrated RAG system.

### Comprehensive Coverage
The final textbook must provide a complete and holistic understanding of the entire system architecture, from the ROS 2 Nervous System up through the VLA Cognitive Brain.

### Technical Accuracy Standard
All content—including code snippets, mathematical derivations, and technical specifications—must be rigorously checked for correctness and align with the latest versions of ROS 2 and the NVIDIA Isaac Platform.

### Modular Structure Standard
The textbook must be organized into four distinct, sequential modules as outlined in the curriculum, ensuring logical flow and ease of indexing for the RAG Chatbot.

### Tool-Specific Format
The book's final output format and style must comply with the specifications and conventions enforced by the generative tool used (Claude Code/Spec-Kit Plus) to ensure compatibility and consistency.

## Key Standards

### Documentation Platform Standard
All final documentation output (the textbook chapters) must be rendered into Markdown files that strictly adhere to the file naming and front-matter conventions required for publishing on the Docusaurus documentation framework. This ensures the content is ready for immediate deployment as a web-based document.

## Success Criteria

### Functional RAG Chatbot
A fully operational RAG Chatbot (FastAPI, Agents/ChatKit) that can accurately query and respond based on the content of the AI-native textbook.

### VLA-Integrated Control
Successful demonstration of an integrated Vision-Language-Action (VLA) model performing cognitive planning and high-level control of a simulated humanoid system.

### Complete Textbook
A comprehensive, four-module AI-native textbook covering: ROS 2, Digital Twin (Simulation), AI-Robot Brain (Isaac), and VLA Integration.

## Constraints

### Tool Adherence
Advice is limited to and must utilize the specified tool stack: ROS 2, NVIDIA Isaac Platform, Claude Code/Spec-Kit Plus, and OpenAI Agents/ChatKit SDKs.

### Scope Limitation
Guidance is strictly limited to the technical scope of the four course modules and the resulting humanoid robotics system. Avoid providing generic LLM or non-robotics advice.

## Governance

Constitution supersedes all other practices; Amendments require documentation, approval, migration plan. All PRs/reviews must verify compliance; Complexity must be justified; Use guidance files for runtime development guidance.

**Version**: 1.0.1 | **Ratified**: 2025-01-01 | **Last Amended**: 2025-12-07