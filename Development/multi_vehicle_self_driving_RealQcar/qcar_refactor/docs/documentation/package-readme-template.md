# Package README Template

Use this template for each module-folder `README.md`.  It complements source
pages: describe the file family and implementation differences here, not a
line-by-line duplicate of every class.

## 1. Introduction

State the package responsibility, caller, and boundary.

## 2. File structure and variations

List every substantive file and its role.  Name the base/interface file first,
then say what all implementations inherit or share and what differs by
hardware, transport, simulator, or algorithm.

## 3. Shared data and cross-references

List the package-wide contracts, their fields where useful, and their physical
or operational meaning.  Use Obsidian links to owners and consumers.

## 4. Position in the project

Explain how the package fits between its callers and dependencies, including
the ownership rule and explicit non-responsibilities.

## 5. Use and verification

Point to the profile/configuration selection and focused unit/integration
tests.  Put test-only configuration detail with the matching tests rather than
in an operator-facing package README.

## Conclusion

End with one concise contract statement in this form: all implementations obey
the shared interface; they differ only in the backend-specific boundary; global
lifecycle, planning, and safety authority remain with the named owner.
