---
id: 1
title: update-documentation-platform-standard
stage: constitution
date: 2025-12-07
surface: agent
model: 
feature: none
branch: 
user: 
command: /sp.constitution
labels: [constitution, documentation, standard]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - 
---

## Prompt

This is the Qwen Code. We are setting up the context for our chat.
Today's date is Sunday, December 7, 2025 (formatted according to the user's locale).
My operating system is: win32
I'm currently working in the directory: D:\hackthonQ3\hacathon\pysical_ai
Here is the folder structure of the current working directories:

Showing up to 20 items (files + folders). Folders or files indicated with ... contain more items not shown, were ignored, or the display limit (20 items) was reached.

D:\hackthonQ3\hacathon\pysical_ai\
├───QWEN.md
├───.git\...
├───.qwen\
│   └───commands\
│       ├───sp.adr.toml
│       ├───sp.analyze.toml
│       ├───sp.checklist.toml
│       ├───sp.clarify.toml
│       ├───sp.constitution.toml
│       ├───sp.git.commit_pr.toml
│       ├───sp.implement.toml
│       ├───sp.phr.toml
│       ├───sp.plan.toml
│       ├───sp.specify.toml
│       └───sp.tasks.toml
└───.specify\
    ├───memory\
    │   └───constitution.md
    ├───scripts\
    └───templates\

/sp.constitution

**Role and Expertise:**
You are an expert Teaching Assistant and Project Advisor for the vide detailed, accurate, and technical guidance on creating an **AI-native textbook** and completing the project requirements.

***

## Core Principles (The 'Why' for the Book)
1.  **AI-Native Documen"Physical AI & Humanoid Robotics" course and hackathon. Your primary function is to protation:** The textbook's primary purpose is to serve as the **authoritative knowledge base** for the RAG Chatbot. Documentation must be generated and structured using AI-native tools (e.g., **Claude Code/Spec-Kit Plus**) to ensure efficiency and integration readiness.
2.  **Actionable Knowledge Base:** The book must be optimized for machine readability and retrieval. Content must be clear, granular, and easily translatable into a structured database to maximize the performance of the integrated **RAG system**.
3.  **Comprehensive Coverage:** The final textbook must provide a complete and holistic understanding of the entire system architecture, from the **ROS 2 Nervous System** up through the **VLA Cognitive Brain**.

---

## Key Standards (The 'How' for the Book)
1.  **Technical Accuracy Standard:** All content—including code snippets, mathematical derivations, and technical specifications—must be rigorously checked for correctness and align with the latest versions of **ROS 2** and the **NVIDIA Isaac Platform**.
2.  **Modular Structure Standard:** The textbook must be organized into four distinct, sequential modules as outlined in the curriculum, ensuring logical flow and ease of indexing for the **RAG Chatbot**.
3.  **Tool-Specific Format:** The book's final output format and style must comply with the specifications and conventions enforced by the generative tool used (**Claude Code/Spec-Kit Plus**) to ensure compatibility and consistency.

---

## Success Criteria (The 'What to Deliver')
1.  **Functional RAG Chatbot:** A fully operational RAG Chatbot (**FastAPI, Agents/ChatKit**) that can accurately query and respond based on the content of the AI-native textbook.
2.  **VLA-Integrated Control:** Successful demonstration of an integrated **Vision-Language-Action (VLA)** model performing cognitive planning and high-level control of a simulated humanoid system.
3.  **Complete Textbook:** A comprehensive, four-module AI-native textbook covering: **ROS 2**, **Digital Twin (Simulation)**, **AI-Robot Brain (Isaac)**, and **VLA Integration**.

---

## Constraints (The 'Boundaries')
1.  **Tool Adherence:** Advice is limited to and must utilize the specified tool stack: **ROS 2**, **NVIDIA Isaac Platform**, **Claude Code/Spec-Kit Plus**, and **OpenAI Agents/ChatKit SDKs**.
2.  **Scope Limitation:** Guidance is strictly limited to the technical scope of the four course modules and the resulting humanoid robotics system. Avoid providing generic LLM or non-robotics advice.
Please add the following constraint to the existing, active /sp.constitution under the Key Standards section: . '''Documentation Platform Standard: All final documentation output (the textbook chapters) must be rendered into Markdown files that strictly adhere to the file naming and front-matter conventions required for publishing on the Docusaurus documentation framework. This ensures the content is ready for immediate deployment as a web-based document.'''

## Response snapshot

The project constitution has been successfully updated with the new constraint about documentation platform standards. Here's the summary:

1. **New Version**: 1.0.1 (PATCH increment from 1.0.0)
2. **Bump Rationale**: Added the "Documentation Platform Standard" constraint to the Key Standards section, requiring all documentation output to adhere to Docusaurus framework conventions
3. **Changes Made**:
   - Added a new "Documentation Platform Standard" section in Key Standards
   - Updated the Sync Impact Report at the top of the file
   - Filled in placeholder values with project-specific information
   - Set the ratification date to 2025-01-01 (as a placeholder since the original date is unknown)
   - Set the last amended date to 2025-12-07 (today's date)
4. **Files Updated**: Only the constitution file itself (`.specify/memory/constitution.md`)
5. **No Manual Follow-up Required**: All dependent templates were checked and found to not require updates

The new constraint has been properly integrated into the constitution as requested: "All final documentation output (the textbook chapters) must be rendered into Markdown files that strictly adhere to the file naming and front-matter conventions required for publishing on the Docusaurus documentation framework. This ensures the content is ready for immediate deployment as a web-based document."

The updated constitution is now in place and maintains consistency with the project's goals for the Physical AI & Humanoid Robotics course and hackathon.

## Outcome

- ✅ Impact: Updated constitution with new documentation platform standard
- 🧪 Tests: No tests required for this change
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: 
- 🧠 Reflection: Successfully integrated the new constraint into the constitution while maintaining all existing principles and structure

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A