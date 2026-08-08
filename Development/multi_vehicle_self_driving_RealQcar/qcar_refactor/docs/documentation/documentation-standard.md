# Documentation Standard

Every substantive source page uses the five sections in the
[source-page template](source-page-template.md): introduction, code structure,
special data and cross-references, position in the project, and use and
verification.  Every package `README.md` uses the complementary
[package README template](package-readme-template.md), where code structure is
replaced by file structure and implementation variations.

Every source page lists its defined classes and functions (including relevant
private helpers), constructor dependencies, inputs, outputs, and a one-sentence
algorithm or purpose.  It describes observable behavior, not private
implementation trivia.  Package indexes may group definitions only when they
are import-only shims.

## Notation

- `x, y, theta, v, a` are position in metres, heading in radians, speed in
  metres/second, and acceleration in metres/second².
- `delta` is steering angle; `L` is wheelbase; `dt` is a positive monotonic
  control period.
- `hat{x}` denotes an estimate, and `z` a sensor measurement.
- All transport freshness durations use local monotonic time. Wall-clock time
  is audit data only.

## Ownership rule

`core` owns domain lifecycle and safe actuation decisions. `utils` owns one
bounded service behind an injected interface. A page must name its caller and
must not imply that a utility directly changes the global vehicle state.

## Obsidian links

Use links to show real ownership, data-contract, or call relationships. At a
minimum, a page links its interface/base class, its exchanged data contract,
and its direct lifecycle owner when those are outside the file. Write the
Obsidian link directly in the prose (do not wrap it in inline-code backticks)
and prefer a readable alias such as [[io-base|IOBase]]. Do not create a link
for an incidental name.

## Algorithms and mechanisms

Algorithm pages give the implemented equation and identify simplifications.
Mechanism pages use a small flow or state diagram and state where validation,
queueing, and safe-zero behavior occur. A formula is documentation of the
current implementation, not a claim of physical-model accuracy.
