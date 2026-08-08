# Source Page Template

Use this template for one substantive source file.  Keep the heading names so
pages remain scannable in Obsidian; remove only a subsection that is genuinely
not applicable.

## 1. Introduction

State the source file, one-sentence responsibility, its owner/caller, and the
boundary it must not cross.  Link the primary owner and any relevant interface.

## 2. Code structure

For each class, state the constructor inputs and what it stores or depends on.
Then list every public function and relevant private helper as:

| Definition | Inputs | Output | Algorithm or purpose |
| --- | --- | --- | --- |
| `Class.method(...)` | typed/physical inputs | returned value or side effect | one sentence |

List module-level functions in the same table.  If a function contains an
algorithm, state its implemented method or equation, not an aspiration.

## 3. Special data and cross-references

Describe non-obvious state, caches, frames, messages, and arrays.  For each,
give its elements and one sentence of physical or operational meaning.  Link
the definition page, for example [[vehicle-types|SensorData]] or
[[io-base|IOBase]].  State coordinate-frame, timestamp, ownership, or
freshness assumptions here when applicable.

## 4. Position in the project

Identify the direct caller, direct collaborators, and the source of authority.
Explain why this module exists at that boundary and which adjacent concern it
does *not* own.

## 5. Use and verification

Show the smallest supported invocation or configuration snippet.  Link or name
the focused unit tests and any integration/hardware test, including required
external preconditions.  State the expected safety behavior on failure.
