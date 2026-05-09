# PROJECT_CONTEXT.md

*Companion to PROJECT_STATE.md. The technical document describes WHAT is being built. This document describes WHO is building it, WHY it matters, and HOW to collaborate effectively.*

**Last updated:** May 7, 2026 — initial creation

---

## Who is involved

**Hudson** is the only contributor. 23, mechanical engineering student at ASU Online, robotics is a possible career direction. Solo project — no peer reviewers, no other developers, no client. Hudson plays Engineering Lead, makes architectural decisions, reviews PRs, and writes specs. Claude (chat venue) is the Project Manager / planner / reviewer; Claude Code is the executor.

The project is on Hudson's GitHub (`RocketDevSpace/stewart-platform-controller`). Adam Williams (Hudson's part-time employer at the prosthetics company) is not involved with this project — this is independent personal work.

## What this project is

A desktop control system for a hand-built 6-DOF Stewart platform. The hardware is a real physical rig in Hudson's workspace: a moving platform with six servo-driven legs, an Arduino managing the servos, and a webcam for ball tracking. The software does manual pose control, scripted motion routines, and closed-loop ball balancing.

The project started as a Codex-generated codebase that needed serious architectural cleanup. The current phase is a multi-milestone refactor (M1–M4 done, M5–M6 pending) restructuring it into clean layered modules. After the refactor lands, the project pivots to feature work — most notably a second-camera setup that unlocks 3D ball tracking, ball catching, and ball bouncing.

## The deeper mission

Two levels matter here.

**Surface level:** A working ball-balancing platform that does cool things. Tangible, satisfying, demonstrably real. The kind of thing that's nice to show people and that produces good demo footage.

**Underlying level:** A learning vehicle for robotics, system architecture, and disciplined software development. Hudson has solid engineering fundamentals from his ME degree and rocketry / SpaceX background, but software architecture is a domain where he's still building reps. This project is partly about producing a real working artifact, but it's just as much about developing the judgment to direct AI tools well on substantial software projects — judgment that compounds across future projects in robotics, prosthetics, and anything else that involves running real systems.

That second level is why the project gets the full infrastructure treatment (Tier C+ per PROJECT_BOOTSTRAP.md). A project that was *only* about shipping a balancing rig wouldn't need this much process scaffolding. The process scaffolding is part of what's being learned.

## What I'm actually optimizing for

Not raw speed-to-completion. The refactor isn't urgent in any external-deadline sense — it's about getting the architecture *right* and learning what right looks like.

- **Quality of the resulting architecture** comes first.
- **Hudson's understanding of the architecture** comes a close second. Not "Claude built me a thing" but "Hudson directed the build and can explain why every layer is the way it is."
- **Working hardware** is the third axis, gating on M6 completion at the earliest.
- **Speed** is somewhere fourth. Sessions can be efficient or slow depending on how much Hudson wants to dig in on a given decision; that's fine.

The implication: when Claude is choosing between "ship this fast" and "make Hudson genuinely understand this," default toward the second unless Hudson signals otherwise (the "just do it" override is the explicit signal).

## How I'll work on this

Hudson works on this project in side-project time, which is roughly 3–4 hours a week unless something gets prioritized. Not a daily commit cadence. Sessions tend to land when Hudson has a clear block of time and energy — often evenings, sometimes weekends. Don't expect or assume continuous progress.

The collaboration uses the **chat-as-planner / Code-as-executor split** that's in HUDSON_CONTEXT.md. Pattern:

1. Chat venue reads PROJECT_STATE, PROJECT_CONTEXT, CLAUDE.md, SPEC, CHANGELOG (all in project knowledge for chat) and any relevant code via GitHub MCP read tools.
2. Chat venue produces a milestone plan, gets Hudson's approval.
3. Hudson pastes the plan into Claude Code along with CLAUDE.md context.
4. Claude Code executes step-by-step, commits per step, opens one PR at the end.
5. Chat venue reviews the PR via GitHub MCP. Claude Code replies to each review thread with commit hash of the fix.
6. Hudson merges manually after PM (chat venue) approves.

**Write-pattern rule (load-bearing):** Code venue does all GitHub writes. Chat venue stays read-only against GitHub except for short edits to small docs. The GitHub MCP write flow in chat is too slow and token-expensive for full-file rewrites; this rule keeps chat sessions efficient.

## How I want Claude to work on this project

HUDSON_CONTEXT.md covers the general working preferences. Project-specific overrides and emphases:

- **Default-check is especially important here** because Hudson is using this project to build software-architecture judgment. Silent decisions deny him learning. Even on borderline calls, lean toward checking. The "just do it" override is available when he wants execution speed instead.
- **Expertise scaffolding is essential.** Surface "what good looks like" alongside whatever you're proposing. Hudson is calibrating his own judgment off the explanations.
- **Hold position when pushed back on, especially on architecture.** If Hudson disagrees with an architectural recommendation, make him articulate why before adjusting. Sycophantic capitulation is worse than friction here.
- **Read the actual code before commenting on it.** Don't pattern-match what a typical PyQt5 / OpenCV project looks like and assume this one matches. The Codex-original codebase has quirks (forbidden import prefixes, ball_controller in the wrong directory, 25KB GUI monolith) that are not what a fresh well-architected codebase would have.

## What good looks like in this domain

This is a software-architecture domain where Hudson is still building reps. Things to recognize and surface:

- **Layer separation as a real pattern, not a slogan.** GUI files don't call IK. Control files don't import Qt. Each layer talks to the layer directly below it, not the one two layers down. When this gets violated, it's worth saying so clearly and explaining the cost.
- **Single-call-site discipline.** When the same logic shows up in two places, it's not "convenient duplication" — it's a future bug waiting to drift.
- **Data contracts as a separate thing from code.** The dataclasses in `core/platform_state.py` are the contract; code that uses them must conform. Defining new shared shapes inline in code is an anti-pattern.
- **Testable pure logic.** Routines and controllers that don't touch Qt or hardware can be unit-tested. The decision to keep them Qt-free is what makes that possible.
- **CI as a discipline, not a checkbox.** A PR with failing CI shouldn't merge. The exclude list in `setup.cfg` shrinks each milestone — that's how you know the refactor is making progress.
- **Refactor without functional change** as a discipline. M1–M4 explicitly avoided behavior changes. That separation of "restructure" from "add features" is what makes the refactor reviewable. Mixing them is how refactors fail.

## Known collaboration patterns that work / don't

*Will accumulate as we learn. Initial entries from prior project work:*

- **The chat-as-planner / Code-as-executor split works** — the back-and-forth (paste test output → get prompt → paste prompt → run script → paste output) is moderately friction-y but the role separation is genuine value.
- **AI_WORKFLOW.md in its prior form** was over-formalized for a solo project — too much ceremony around a workflow that doesn't actually have a separate human PM. The replacement (CLAUDE.md + PROJECT_CONTEXT + PROJECT_STATE + retained SPEC and CHANGELOG) keeps the discipline without the over-roleplay.

## What NOT to do

- Don't propose architectural changes without a clear rationale. Hudson values reasoning more than recommendations.
- Don't assume the architecture is in target state when reading the code. Refactor is in flight; the gap between target and current is real and the doc system tries to make this explicit.
- Don't propose Mac / Linux support, web UI, cloud integration, or other scope expansions outside the listed future features. The project is intentionally bounded.
- Don't churn on cosmetic things (formatting, variable names, docstring style) when the linter handles them. Save Claude's tokens and Hudson's attention for substantive decisions.
- Don't engage with the project as if Hudson has years of software-architecture reps. Scaffolding is genuinely helpful here, not condescending.

## The standard

The bar isn't "code that works." The bar is **code Hudson can defend the architecture of, where every layer's existence is justifiable, and where the test suite is real enough to catch regressions.**

Specific things that would feel like wins:
- Refactor lands, M5 and M6 both shipped, gui_layout.py fully retired.
- M1–M6 CHANGELOG entries read as a coherent architectural story, not a sequence of unrelated changes.
- Test coverage is real on every clean module — no "I'll add tests later."
- A second person could read CLAUDE.md and PROJECT_STATE.md and understand where the project is and how to contribute.
- The second-camera scoping conversation, when it happens, is shaped by what the architecture already supports rather than fighting against it.

Specific things that would feel like failures:
- Layer violations sneaking into "clean" modules because nobody pushed back.
- A refactored module that's "done" but actually has a subtle behavior difference from the original.
- Tests that pass but don't really exercise the logic.
- Hudson finishing the refactor without being able to articulate why the architecture is the way it is.

## How this document is maintained

Hudson maintains this document. PROJECT_STATE.md is updated more frequently — at the end of substantive sessions, when decisions land, when scope shifts. PROJECT_CONTEXT.md changes only when the project's deeper direction or collaboration patterns shift.

This doc lives in the repo root and in the Claude.ai project knowledge for the planning venue.
