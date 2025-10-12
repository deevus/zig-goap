# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**zig-goap** is a Zig library implementing Goal-Oriented Action Planning (GOAP) using A* pathfinding. GOAP is an AI planning technique where agents determine a sequence of actions to transition from a current world state to a desired goal state.

## Build Commands

```bash
# Build the library
zig build

# Run the example executable
zig build run

# Run all tests
zig build test
```

## Architecture

### Core Components

The library consists of three main modules:

1. **ActionPlanner** (`src/action_planner.zig`):
   - Stores action definitions with their preconditions, postconditions, and costs
   - WorldState is represented as `std.StringHashMap(bool)` - a map of atom names to boolean values
   - Actions are defined by:
     - Preconditions: WorldState conditions that must be met before the action can execute
     - Postconditions: WorldState changes that result from executing the action
     - Cost: u32 value used in pathfinding (lower is better)
   - `get_possible_state_transitions()` returns all valid actions from a given state
   - Uses an internal ArenaAllocator for efficient memory management of conditions

2. **A\* Pathfinding** (`src/astar.zig`):
   - Generic A* implementation that operates on WorldStates
   - Uses a priority queue to explore paths with lowest cost first
   - VTable pattern for customizable distance/goal/neighbor functions
   - `plan_with_astar()` is the high-level function that finds action sequences
   - Returns a `Result` union: `.done` (with path and actions), `.neighbors` (intermediate), or `.no_path`
   - Tracks visited states to avoid cycles

3. **Module Exports** (`src/root.zig`):
   - Library entry point that re-exports public API
   - Exports: Astar, AstarOptions, Result, ActionPlanner, ActionError, WorldState, PossibleTransitions

### Memory Management

- ActionPlanner uses an ArenaAllocator for all condition maps to simplify cleanup
- A* pathfinding uses its own ArenaAllocator for internal state tracking
- Results returned from `plan_with_astar()` own their memory via DoneResult's arena
- Always call `deinit()` on ActionPlanner and Results to prevent leaks

### Typical Usage Pattern

1. Create an ActionPlanner and define actions with `setPrecondition()`, `setPostcondition()`, `setCost()`
2. Create a current WorldState and set initial atom values
3. Create a goal WorldState with desired atom values
4. Call `plan_with_astar()` to find the action sequence
5. Check if result is `.done` and extract the `actions` list
6. Clean up all resources with `deinit()`

See `src/astar.zig:370-420` and `src/main.zig` for complete examples.

### Example Domain (from tests)

The tests demonstrate a tactical combat scenario with actions like:
- "scout": Requires armed with gun → makes enemy visible
- "approach": Requires enemy visible → get near enemy
- "load": Requires armed with gun → weapon loaded
- "aim": Requires enemy visible + weapon loaded → enemy lined up
- "shoot": Requires enemy lined up → enemy not alive

The planner finds optimal action sequences based on the current state and goal.

## Tools

### Zigdoc

Usage: zigdoc [options] <symbol>

Show documentation for Zig standard library symbols and imported modules.

zigdoc can access any module imported in your build.zig file, making it easy
to view documentation for third-party dependencies alongside the standard library.

Examples:
  zigdoc std.ArrayList
  zigdoc std.mem.Allocator
  zigdoc std.http.Server
  zigdoc vaxis.Window
  zigdoc zeit.timezone.Posix

Options:
  -h, --help        Show this help message
  --dump-imports    Dump module imports from build.zig as JSON
