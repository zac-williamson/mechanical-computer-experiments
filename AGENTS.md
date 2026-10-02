# Active design entry points

There are exactly two active designs:

1. `modular-actuator-carriage/` — current reusable actuator, carriage, clutch and locking-bolt module. Read its README.md, AGENTS.md and Source/README.md first. Its supported rebuild, bundled inputs and validation scripts are in Source/.
2. `planar-register/` — preserved latched 1-bit register: the level-sensitive write-enabled latch. Read its README.md and AGENTS.md. Do not confuse it with the archived clocked-register variants.

The next objective is a small collection of reusable modules for the 1-bit register. Extend this collection deliberately; do not introduce competing versions of the same module without user direction.

## Archive boundary

`archive/` is inactive historical material. Do not search it for current design entry points, edit it, restore it or use it as a starting point unless the user explicitly requests that history. Statements such as “current”, “sole current” or “replaces” inside archived documents describe their historical revision, not the active repository.

The frozen build inputs under `planar-register/work/` and the historical source archive under `modular-actuator-carriage/Source/History/` are dependencies/reference material, not additional active designs. The supported module entry point is Source/rebuild.py; historical scripts can retain obsolete local paths.

Preserve unrelated uncommitted edits, including archived work. Never include them in a commit merely because their directories moved.

## Design constraints

Honor each active design's axis and printability rules. In particular, retain visible camera-aware X/Y/Z indicators; evaluate every changed part in its actual print orientation; protect bearing and sliding faces from support material; and distinguish geometric checks from physical validation. Run geometry jobs serially using the documented CPU-throttled runner.
