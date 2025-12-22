---
id: 002
title: "Docusaurus UI Theme Plan"
stage: plan
date_iso: "2025-12-21"
surface: "agent"
model: "Qwen"
feature: "docusaurus-ui-theme"
branch: "002-docusaurus-ui-theme"
user: "user"
command: "/sp.plan"
labels: ["ui", "docusaurus", "theme", "frontend", "planning"]
links:
  spec: "specs/002-docusaurus-ui-theme/spec.md"
  plan: "specs/002-docusaurus-ui-theme/plan.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/002-docusaurus-ui-theme/plan.md"
  - "specs/002-docusaurus-ui-theme/research.md"
  - "specs/002-docusaurus-ui-theme/data-model.md"
  - "specs/002-docusaurus-ui-theme/quickstart.md"
  - "specs/002-docusaurus-ui-theme/contracts/component-contracts.md"
tests: []
---

# Docusaurus UI Theme Plan

## User Prompt

```
/sp.plan /sp.plan
Project: Physical AI & Humanoid Robotics — Docusaurus UI Theme

Plan ID: ui-theme-physical-ai-docusaurus-plan-v1
Derived From: /sp.specify ui-theme-physical-ai-docusaurus-v1
Phase: Phase-1
Status: Execution-Ready
Framework: Docusaurus

1. Plan Objective

Execute the approved UI theme inside a Docusaurus project with visual and behavioral parity to the provided static HTML/CSS/JS implementation.

This plan defines:

File creation order

Exact responsibilities per step

Dependency sequencing

Verification checkpoints

2. Execution Principles (Hard Rules)

No visual redesign

No additional sections

No new animations

No framework beyond Docusaurus defaults

No refactors unless required for React compatibility

One-way execution (no iteration during Phase-1)

3. High-Level Execution Phases
PhaseGoal
P1Project scaffold verification
P2Global theme foundation
P3Homepage structure
P4Interactive behavior
P5Responsive validation
P6Acceptance validation
4. Phase Breakdown
Phase P1 — Docusaurus Baseline (Dependency: none)

Objective

Ensure a clean Docusaurus project baseline exists.

Steps

Confirm Docusaurus project initializes without errors

Verify default homepage renders

Verify custom.css is loaded

Deliverables

Running local dev server

No runtime warnings

Phase P2 — Global Theme Foundation

Dependency: P1 complete

Objective

Establish global design system parity.

Steps

Add Google Fonts (Inter, Orbitron) globally

Define CSS variables in src/css/custom.css

Override base typography styles

Set global background gradient

Define shared animation keyframes

Deliverables

Global styles visually match static theme

No component-specific CSS yet

Phase P3 — Homepage Structure

Dependency: P2 complete

Objective

Recreate page layout in Docusaurus architecture.

Steps

Implement src/pages/index.jsx

Wrap content with @theme/Layout

Add Hero section JSX

Add Features section JSX

Add Learning Objectives section JSX

Mount footer (temporary placeholder if needed)

Deliverables

Static layout visible

Correct content hierarchy

No animations yet

Phase P4 — Navigation & Footer Overrides

Dependency: P3 complete

Objective

Match header and footer behavior.

Steps

Override Navbar via src/theme/Navbar

Apply fixed positioning and blur styles

Implement scroll-based scrolled class

Override Footer via src/theme/Footer

Map footer columns and links

Deliverables

Fixed navbar with scroll response

Footer matches static design

Phase P5 — Interactive Behavior

Dependency: P4 complete

Objective

Port JavaScript behavior into React-safe logic.

Steps

Integrate particles.js (client-side only)

Implement IntersectionObserver via useEffect

Apply visible class on reveal

Implement CTA pulse timer

Enable smooth scrolling offset logic

Deliverables

All animations functional

No hydration or SSR errors

Phase P6 — Responsive Validation

Dependency: P5 complete

Objective

Ensure breakpoint parity.

Steps

Validate ≥992px layout

Validate ≤992px hero SVG removal

Validate ≤768px mobile navigation

Validate typography scaling

Validate touch interactions

Deliverables

Responsive behavior matches spec

No layout collapse

Phase P7 — Acceptance Validation

Dependency: P6 complete

Objective

Confirm compliance with /sp.specify.

Checklist

 No console errors

 Particle background works

 Navbar scroll behavior correct

 Mobile nav toggles correctly

 Feature/objective reveal animations trigger once

 Visual parity confirmed

Deliverable

Phase-1 acceptance approval

5. Dependency Graph (Strict Order)
P1 → P2 → P3 → P4 → P5 → P6 → P7


No phase may be skipped or reordered.

6. Roles & Responsibility

Owner (You)

Approves phase completion

Enforces freeze discipline

Executor

Implements exactly as planned

Does not interpret or enhance

7. Change Control

Any deviation requires:

Spec revision

Plan revision

Phase authority approval

No exceptions.

8. Plan Freeze Declaration

This /sp.plan is frozen for Phase-1 execution.

Execution may begin immediately.
```

## Assistant Response

Successfully executed the implementation planning workflow for the Physical AI & Humanoid Robotics Docusaurus UI Theme. The following artifacts were created:

1. **plan.md** - Comprehensive implementation plan with technical context, constitution check, and project structure
2. **research.md** - Research summary with technical decisions and best practices
3. **data-model.md** - Data model defining key entities and components
4. **quickstart.md** - Quickstart guide with setup and development workflow
5. **contracts/** - Directory containing component interface contracts
6. **Agent context updated** - Updated Qwen Code context file with new technology stack

All constitutional requirements were verified and passed. The implementation plan follows the Docusaurus project structure and meets all specified requirements for visual and behavioral parity with the original static theme. The plan is now ready for Phase 1 execution.