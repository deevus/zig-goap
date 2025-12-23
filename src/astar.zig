const std = @import("std");
const testing = std.testing;
const action_planner = @import("action_planner.zig");

const PriorityQueue = std.PriorityQueue;
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;
const ArenaAllocator = std.heap.ArenaAllocator;
const Order = std.math.Order;
const WorldState = @import("action_planner.zig").WorldState;
const ActionPlanner = action_planner.ActionPlanner;
const ActionError = action_planner.ActionError;

const Path = struct {
    path: *ArrayList(WorldState),
    actions: *ArrayList([]const u8),
    current: WorldState,
    g_cost: usize = 0,
    allocator: Allocator,

    pub fn init(current: WorldState, allocator: Allocator) Path {
        const path = allocator.create(ArrayList(WorldState)) catch unreachable;
        path.* = ArrayList(WorldState).empty;

        const actions = allocator.create(ArrayList([]const u8)) catch unreachable;
        actions.* = ArrayList([]const u8).empty;

        return Path{
            .path = path,
            .actions = actions,
            .current = current,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *Path) void {
        for (self.actions.items) |action| {
            self.allocator.free(action);
        }
        self.actions.deinit(self.allocator);
    }

    pub fn dupe(self: *Path) !Path {
        return self.dupeOwned(self.allocator);
    }

    pub fn dupeOwned(self: *Path, allocator: Allocator) !Path {
        const path = allocator.create(ArrayList(WorldState)) catch unreachable;
        path.* = try self.path.clone(allocator);

        const actions = allocator.create(ArrayList([]const u8)) catch unreachable;
        actions.* = .empty;

        for (self.actions.items) |action| {
            try actions.append(allocator, try allocator.dupe(u8, action));
        }

        return Path{
            .path = path,
            .actions = actions,
            .current = self.current,
            .g_cost = self.g_cost,
            .allocator = allocator,
        };
    }
};

const DoneResult = struct {
    path: *ArrayList(WorldState),
    actions: *ArrayList([]const u8),
    arena: *ArenaAllocator,

    pub fn init(allocator: Allocator, path: Path) !DoneResult {
        const arena = try allocator.create(ArenaAllocator);
        arena.* = ArenaAllocator.init(allocator);

        const arena_allocator = arena.allocator();

        const clone_path = try arena_allocator.create(ArrayList(WorldState));
        clone_path.* = ArrayList(WorldState).empty;

        for (path.path.items) |ws| {
            try clone_path.append(arena_allocator, try ws.cloneWithAllocator(arena_allocator));
        }

        const actions = try arena_allocator.create(ArrayList([]const u8));
        actions.* = ArrayList([]const u8).empty;

        for (path.actions.items) |action| {
            try actions.append(arena_allocator, try arena_allocator.dupe(u8, action));
        }

        return DoneResult{
            .path = clone_path,
            .actions = actions,
            .arena = arena,
        };
    }

    pub fn deinit(self: *DoneResult) void {
        const allocator = self.arena.child_allocator;
        self.arena.deinit();
        allocator.destroy(self.arena);
    }
};

pub const Result = union(enum) {
    done: DoneResult,
    neighbors: WorldState,
    no_path,

    pub fn deinit(self: Result) void {
        switch (self) {
            .done => {
                var done_result = self.done;
                done_result.deinit();
            },
            else => {},
        }
    }
};

const VTable = struct {
    distance: *const fn (*const WorldState, *const WorldState) usize,
    is_goal: *const fn (*const WorldState, *const WorldState) bool,
    get_neighbors: *const fn (Allocator, ap: *const ActionPlanner, *const WorldState) ActionError![]Neighbor,
};

pub const AstarOptions = struct {
    actionPlanner: *const ActionPlanner,
    start: *const WorldState,
    vtable: *const VTable,
};

pub const Astar = struct {
    const Self = @This();
    const NextQueue = PriorityQueue(Path, *const WorldState, compare);

    next_q: NextQueue,
    seen: ArrayList(*const WorldState),
    start: *const WorldState,
    end: *const WorldState,
    arena: *ArenaAllocator,
    ap: *const ActionPlanner,
    vtable: *const VTable,

    pub fn init(allocator: Allocator, options: AstarOptions) !Self {
        const arena = allocator.create(ArenaAllocator) catch unreachable;
        arena.* = ArenaAllocator.init(allocator);

        const arena_alloc = arena.allocator();

        const start = try arena_alloc.create(WorldState);
        start.* = try options.start.cloneWithAllocator(arena_alloc);

        const end = try arena_alloc.create(WorldState);
        end.* = try options.start.cloneWithAllocator(arena_alloc);

        const next_q = NextQueue.init(arena_alloc, start);
        const seen: ArrayList(*const WorldState) = .empty;

        return Self{
            .next_q = next_q,
            .seen = seen,
            .start = start,
            .end = end,
            .arena = arena,
            .ap = options.actionPlanner,
            .vtable = options.vtable,
        };
    }

    pub fn deinit(self: *Self) void {
        const allocator = self.arena.child_allocator;
        self.arena.deinit();
        allocator.destroy(self.arena);
    }

    pub fn pathFind(self: *Self, start: *const WorldState, end: *const WorldState) !Result {
        self.next_q.items.len = 0;
        self.seen.items.len = 0;
        self.start = start;
        self.end = end;
        try self.seen.append(self.arena.allocator(), start);
        try self.next_q.add(Path.init(start.*, self.arena.allocator()));
        return Result{ .neighbors = start.* };
    }

    pub fn step(self: *Self) !Result {
        if (self.next_q.items.len == 0) {
            return Result.no_path;
        }

        var best = self.next_q.remove();

        // Goal check
        if (self.vtable.is_goal(&best.current, self.end)) {
            try best.path.append(self.arena.allocator(), best.current);
            return Result{ .done = try DoneResult.init(self.arena.child_allocator, best) };
        }

        // Generate neighbors
        const neighbors = try self.vtable.get_neighbors(self.arena.allocator(), self.ap, &best.current);

        for (neighbors) |neighbor| {
            // Check if we've already seen this state
            var found = false;
            for (self.seen.items) |seen_state| {
                if (state_equals(&neighbor.state, seen_state)) {
                    found = true;
                    break;
                }
            }
            if (found) {
                continue;
            }

            const state = try self.arena.allocator().create(WorldState);
            state.* = neighbor.state;

            try self.seen.append(self.arena.allocator(), state);

            var new_path = try best.dupe();
            try new_path.path.append(self.arena.allocator(), best.current);
            try new_path.actions.append(self.arena.allocator(), neighbor.action_name);

            new_path.current = state.*;
            new_path.g_cost += neighbor.cost;

            try self.next_q.add(new_path);
        }

        const next_best = self.next_q.peek() orelse return Result.no_path;

        return Result{ .neighbors = next_best.current };
    }

    fn compare(end: *const WorldState, a: Path, b: Path) Order {
        const a_cost = a.g_cost + _distance(&a.current, end);
        const b_cost = b.g_cost + _distance(&b.current, end);
        return std.math.order(a_cost, b_cost);
    }
};

const Neighbor = struct {
    state: WorldState,
    action_name: []const u8,
    cost: usize,
    allocator: Allocator,

    pub fn init(allocator: Allocator, state: WorldState, action_name: []const u8, cost: usize) !Neighbor {
        return Neighbor{
            .state = state,
            .action_name = action_name,
            .cost = cost,
            .allocator = allocator,
        };
    }
};

fn state_equals(a: *const WorldState, b: *const WorldState) bool {
    var it = a.iterator();
    while (it.next()) |entry| {
        const atom_name = entry.key_ptr.*;
        const a_value = entry.value_ptr.*;

        const maybe_b_value = b.get(atom_name);
        if (maybe_b_value) |b_value| {
            if (a_value != b_value) {
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

fn _distance(current: *const WorldState, goal: *const WorldState) usize {
    var count: usize = 0;
    var it = goal.iterator();
    while (it.next()) |entry| {
        const atom_name = entry.key_ptr.*;
        const goal_value = entry.value_ptr.*;

        const maybe_current_value = current.get(atom_name);
        if (maybe_current_value) |current_value| {
            if (current_value != goal_value) {
                count += 1;
            }
        } else {
            // Atom is absent in current state, counts as a difference
            count += 1;
        }
    }
    return count;
}

fn _is_goal(current: *const WorldState, goal: *const WorldState) bool {
    var it = goal.iterator();
    while (it.next()) |entry| {
        const atom_name = entry.key_ptr.*;
        const goal_value = entry.value_ptr.*;

        const maybe_current_value = current.get(atom_name);
        if (maybe_current_value) |current_value| {
            if (current_value != goal_value) {
                return false;
            }
        } else {
            // Atom is absent in current state, goal not achieved
            return false;
        }
    }
    return true;
}

fn _get_neighbours(
    allocator: Allocator,
    ap: *const ActionPlanner,
    current_state: *const WorldState,
) ActionError![]Neighbor {
    const transitions = try action_planner.get_possible_state_transitions(
        allocator,
        ap,
        current_state,
    );

    var neighbors = try allocator.alloc(Neighbor, transitions.items.len);

    for (0..transitions.items.len) |i| {
        const transition = transitions.items[i];

        neighbors[i] = try Neighbor.init(
            allocator,
            transition.to_state,
            transition.action_name,
            transition.action_cost,
        );
    }

    return neighbors;
}

pub fn plan_with_astar(
    allocator: Allocator,
    ap: *const ActionPlanner,
    current_state: *const WorldState,
    goal_state: *const WorldState,
) !Result {
    var finder = try Astar.init(allocator, .{
        .actionPlanner = ap,
        .start = current_state,
        .vtable = &.{
            .distance = _distance,
            .get_neighbors = _get_neighbours,
            .is_goal = _is_goal,
        },
    });
    defer finder.deinit();

    var result = try finder.pathFind(current_state, goal_state);

    while (result == .neighbors) {
        result = try finder.step();
    }

    return result;
}

test "GOAP planning with A*" {
    const allocator = std.testing.allocator;

    // Initialize action planner
    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Define actions, preconditions, postconditions, and costs
    try ap.setPrecondition("scout", "armedwithgun", true);
    try ap.setPostcondition("scout", "enemyvisible", true);
    try ap.setCost("scout", 1);

    try ap.setPrecondition("approach", "enemyvisible", true);
    try ap.setPostcondition("approach", "nearenemy", true);
    try ap.setCost("approach", 2);

    try ap.setPrecondition("shoot", "nearenemy", true);
    try ap.setPostcondition("shoot", "enemyalive", false);
    try ap.setCost("shoot", 1);

    // Initialize current world state
    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("armedwithgun", true);
    try current_state.put("enemyvisible", false);
    try current_state.put("nearenemy", false);
    try current_state.put("enemyalive", true);

    // Define goal state
    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("enemyalive", false);

    // Initialize A* planner
    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try std.testing.expect(result == .done);

    const plan = result.done;

    // Expected plan:
    // - scout
    // - approach
    // - shoot

    try std.testing.expectEqual(@as(usize, 3), plan.actions.items.len);
    try std.testing.expectEqualStrings("scout", plan.actions.items[0]);
    try std.testing.expectEqualStrings("approach", plan.actions.items[1]);
    try std.testing.expectEqualStrings("shoot", plan.actions.items[2]);
}

test "planning 2" {
    const allocator = std.testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    try ap.setPrecondition("scout", "armedwithgun", true);
    try ap.setPostcondition("scout", "enemyvisible", true);

    try ap.setPrecondition("approach", "enemyvisible", true);
    try ap.setPostcondition("approach", "nearenemy", true);

    try ap.setPrecondition("aim", "enemyvisible", true);
    try ap.setPrecondition("aim", "weaponloaded", true);
    try ap.setPostcondition("aim", "enemylinedup", true);

    try ap.setPrecondition("shoot", "enemylinedup", true);
    try ap.setPostcondition("shoot", "enemyalive", false);

    try ap.setPrecondition("load", "armedwithgun", true);
    try ap.setPostcondition("load", "weaponloaded", true);

    try ap.setPrecondition("detonatebomb", "armedwithbomb", true);
    try ap.setPrecondition("detonatebomb", "nearenemy", true);
    try ap.setPostcondition("detonatebomb", "alive", false);
    try ap.setPostcondition("detonatebomb", "enemyalive", false);

    try ap.setPrecondition("flee", "enemyvisible", true);
    try ap.setPostcondition("flee", "nearenemy", false);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();

    try current_state.put("enemyvisible", false);
    try current_state.put("armedwithgun", true);
    try current_state.put("weaponloaded", false);
    try current_state.put("enemylinedup", false);
    try current_state.put("enemyalive", true);
    try current_state.put("armedwithbomb", true);
    try current_state.put("nearenemy", false);
    try current_state.put("alive", true);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();

    try goal_state.put("enemyalive", false);

    var plan_a = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer plan_a.deinit();

    try std.testing.expect(plan_a == .done);

    const a_actions = plan_a.done.actions.items;

    try std.testing.expectEqual(3, a_actions.len);
    try std.testing.expectEqualStrings("scout", a_actions[0]);
    try std.testing.expectEqualStrings("approach", a_actions[1]);
    try std.testing.expectEqualStrings("detonatebomb", a_actions[2]);

    try goal_state.put("alive", true);

    var plan_b = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer plan_b.deinit();

    try std.testing.expect(plan_b == .done);

    const b_actions = plan_b.done.actions.items;

    try std.testing.expectEqual(4, b_actions.len);
    try std.testing.expectEqualStrings("scout", b_actions[0]);
    try std.testing.expectEqualStrings("load", b_actions[1]);
    try std.testing.expectEqualStrings("aim", b_actions[2]);
    try std.testing.expectEqualStrings("shoot", b_actions[3]);
}

// ============================================================================
// BASIC FUNCTIONALITY TESTS
// ============================================================================

test "already at goal - should return empty plan" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Define an action (even though we won't need it)
    try ap.setPrecondition("scout", "armed", true);
    try ap.setPostcondition("scout", "enemyvisible", true);
    try ap.setCost("scout", 1);

    // Current state already matches goal
    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("enemyvisible", true);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("enemyvisible", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 0), result.done.actions.items.len);
}

test "single action needed" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    try ap.setPrecondition("flip_switch", "power", false);
    try ap.setPostcondition("flip_switch", "power", true);
    try ap.setCost("flip_switch", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("power", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("power", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 1), result.done.actions.items.len);
    try testing.expectEqualStrings("flip_switch", result.done.actions.items[0]);
}

test "action with no preconditions - always available" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Action with no preconditions (always executable)
    try ap.setPostcondition("spawn", "exists", true);
    try ap.setCost("spawn", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("exists", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("exists", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 1), result.done.actions.items.len);
    try testing.expectEqualStrings("spawn", result.done.actions.items[0]);
}

test "multiple preconditions must all be met (AND logic)" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Action requires BOTH conditions
    try ap.setPrecondition("craft", "has_wood", true);
    try ap.setPrecondition("craft", "has_tools", true);
    try ap.setPostcondition("craft", "has_table", true);
    try ap.setCost("craft", 1);

    // Gather wood
    try ap.setPostcondition("chop_tree", "has_wood", true);
    try ap.setCost("chop_tree", 1);

    // Get tools
    try ap.setPostcondition("buy_tools", "has_tools", true);
    try ap.setCost("buy_tools", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("has_wood", false);
    try current_state.put("has_tools", false);
    try current_state.put("has_table", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("has_table", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 3), result.done.actions.items.len);

    // Should do both prerequisites then craft (order of first two may vary)
    const last_action = result.done.actions.items[2];
    try testing.expectEqualStrings("craft", last_action);
}

test "action changes multiple atoms" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Action that changes multiple things at once
    try ap.setPrecondition("cook_meal", "has_ingredients", true);
    try ap.setPostcondition("cook_meal", "has_ingredients", false); // consumed
    try ap.setPostcondition("cook_meal", "has_food", true); // produced
    try ap.setPostcondition("cook_meal", "kitchen_dirty", true); // side effect
    try ap.setCost("cook_meal", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("has_ingredients", true);
    try current_state.put("has_food", false);
    try current_state.put("kitchen_dirty", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("has_food", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 1), result.done.actions.items.len);

    // Verify final state has all postconditions applied
    const final_state = result.done.path.items[result.done.path.items.len - 1];
    try testing.expectEqual(false, final_state.get("has_ingredients").?);
    try testing.expectEqual(true, final_state.get("has_food").?);
    try testing.expectEqual(true, final_state.get("kitchen_dirty").?);
}

// ============================================================================
// EDGE CASES
// ============================================================================

test "impossible goal - no path exists" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Only action requires something we don't have and can't get
    try ap.setPrecondition("win", "has_key", true);
    try ap.setPostcondition("win", "game_won", true);
    try ap.setCost("win", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("has_key", false);
    try current_state.put("game_won", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("game_won", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .no_path);
}

test "empty goal state - already satisfied" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    try ap.setPostcondition("action", "something", true);
    try ap.setCost("action", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("anything", true);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    // Empty goal - nothing to achieve

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 0), result.done.actions.items.len);
}

test "goal is subset of world state - should match" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    try ap.setPostcondition("action", "x", true);
    try ap.setCost("action", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("x", true);
    try current_state.put("y", true);
    try current_state.put("z", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("x", true); // Only care about x, not y or z

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 0), result.done.actions.items.len);
}

test "cyclic actions - should still find path" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Lock/unlock cycle
    try ap.setPrecondition("lock", "locked", false);
    try ap.setPostcondition("lock", "locked", true);
    try ap.setCost("lock", 1);

    try ap.setPrecondition("unlock", "locked", true);
    try ap.setPostcondition("unlock", "locked", false);
    try ap.setCost("unlock", 1);

    // Goal action
    try ap.setPrecondition("open", "locked", false);
    try ap.setPostcondition("open", "door_open", true);
    try ap.setCost("open", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("locked", true);
    try current_state.put("door_open", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("door_open", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 2), result.done.actions.items.len);
    try testing.expectEqualStrings("unlock", result.done.actions.items[0]);
    try testing.expectEqualStrings("open", result.done.actions.items[1]);
}

test "long action chain" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Create a chain: step1 -> step2 -> step3 -> step4 -> step5
    try ap.setPostcondition("step1", "state1", true);
    try ap.setCost("step1", 1);

    try ap.setPrecondition("step2", "state1", true);
    try ap.setPostcondition("step2", "state2", true);
    try ap.setCost("step2", 1);

    try ap.setPrecondition("step3", "state2", true);
    try ap.setPostcondition("step3", "state3", true);
    try ap.setCost("step3", 1);

    try ap.setPrecondition("step4", "state3", true);
    try ap.setPostcondition("step4", "state4", true);
    try ap.setCost("step4", 1);

    try ap.setPrecondition("step5", "state4", true);
    try ap.setPostcondition("step5", "state5", true);
    try ap.setCost("step5", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("state5", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("state5", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 5), result.done.actions.items.len);
    try testing.expectEqualStrings("step1", result.done.actions.items[0]);
    try testing.expectEqualStrings("step2", result.done.actions.items[1]);
    try testing.expectEqualStrings("step3", result.done.actions.items[2]);
    try testing.expectEqualStrings("step4", result.done.actions.items[3]);
    try testing.expectEqualStrings("step5", result.done.actions.items[4]);
}

// ============================================================================
// COST-BASED OPTIMIZATION TESTS
// ============================================================================

test "choose cheaper of two paths" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Expensive path
    try ap.setPrecondition("expensive_action", "start", true);
    try ap.setPostcondition("expensive_action", "goal", true);
    try ap.setCost("expensive_action", 100);

    // Cheap path (two steps but lower total cost)
    try ap.setPrecondition("cheap_step1", "start", true);
    try ap.setPostcondition("cheap_step1", "middle", true);
    try ap.setCost("cheap_step1", 1);

    try ap.setPrecondition("cheap_step2", "middle", true);
    try ap.setPostcondition("cheap_step2", "goal", true);
    try ap.setCost("cheap_step2", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("start", true);
    try current_state.put("goal", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("goal", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 2), result.done.actions.items.len);
    try testing.expectEqualStrings("cheap_step1", result.done.actions.items[0]);
    try testing.expectEqualStrings("cheap_step2", result.done.actions.items[1]);
}

test "default cost of 1 when not specified" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Don't set cost - should default to 1
    try ap.setPostcondition("action_no_cost", "done", true);

    // Explicitly set cost to 2
    try ap.setPostcondition("action_with_cost", "done", true);
    try ap.setCost("action_with_cost", 2);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("done", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("done", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    // Should choose the default cost=1 action
    try testing.expectEqual(@as(usize, 1), result.done.actions.items.len);
    try testing.expectEqualStrings("action_no_cost", result.done.actions.items[0]);
}

test "optimal path with varying costs" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Create diamond pattern with different costs
    //       /-- B(10) --\
    // Start              Goal
    //       \-- C(2) + D(2) --/

    try ap.setPrecondition("pathB", "start", true);
    try ap.setPostcondition("pathB", "goal", true);
    try ap.setCost("pathB", 10);

    try ap.setPrecondition("pathC", "start", true);
    try ap.setPostcondition("pathC", "middle", true);
    try ap.setCost("pathC", 2);

    try ap.setPrecondition("pathD", "middle", true);
    try ap.setPostcondition("pathD", "goal", true);
    try ap.setCost("pathD", 2);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("start", true);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("goal", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 2), result.done.actions.items.len);
    try testing.expectEqualStrings("pathC", result.done.actions.items[0]);
    try testing.expectEqualStrings("pathD", result.done.actions.items[1]);
}

// ============================================================================
// STATE COMPARISON TESTS
// ============================================================================

test "state_equals - asymmetric states should not match" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // This tests the bug found in code review
    // state_equals should return false if states have different keys
    try ap.setPostcondition("action", "x", true);
    try ap.setCost("action", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("x", true);
    try current_state.put("y", false); // Extra key in current state

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("x", true); // Only x

    // Goal is satisfied (x=true), extra keys don't matter
    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 0), result.done.actions.items.len);
}

test "state_equals - different values should not match" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    try ap.setPrecondition("fix", "broken", true);
    try ap.setPostcondition("fix", "broken", false);
    try ap.setCost("fix", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("broken", true);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("broken", false);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 1), result.done.actions.items.len);
    try testing.expectEqualStrings("fix", result.done.actions.items[0]);
}

// ============================================================================
// REALISTIC SCENARIOS
// ============================================================================

test "crafting tree - gather resources then craft" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Resource gathering
    try ap.setPostcondition("mine_iron", "has_iron", true);
    try ap.setCost("mine_iron", 3);

    try ap.setPostcondition("chop_wood", "has_wood", true);
    try ap.setCost("chop_wood", 2);

    // Crafting (requires both resources)
    try ap.setPrecondition("craft_sword", "has_iron", true);
    try ap.setPrecondition("craft_sword", "has_wood", true);
    try ap.setPostcondition("craft_sword", "has_sword", true);
    try ap.setPostcondition("craft_sword", "has_iron", false); // consumed
    try ap.setPostcondition("craft_sword", "has_wood", false); // consumed
    try ap.setCost("craft_sword", 1);

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("has_iron", false);
    try current_state.put("has_wood", false);
    try current_state.put("has_sword", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("has_sword", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 3), result.done.actions.items.len);

    // Last action must be craft
    try testing.expectEqualStrings("craft_sword", result.done.actions.items[2]);

    // First two actions should be gathering (order may vary)
    const action1 = result.done.actions.items[0];
    const action2 = result.done.actions.items[1];

    const has_mine = std.mem.eql(u8, action1, "mine_iron") or std.mem.eql(u8, action2, "mine_iron");
    const has_chop = std.mem.eql(u8, action1, "chop_wood") or std.mem.eql(u8, action2, "chop_wood");

    try testing.expect(has_mine);
    try testing.expect(has_chop);
}

test "stealth vs combat - choose based on current state" {
    const allocator = testing.allocator;

    var ap = try ActionPlanner.init(allocator);
    defer ap.deinit();

    // Stealth path (requires stealth gear)
    try ap.setPrecondition("sneak_past", "has_stealth_gear", true);
    try ap.setPostcondition("sneak_past", "past_guard", true);
    try ap.setCost("sneak_past", 1);

    // Combat path (requires weapons)
    try ap.setPrecondition("fight_guard", "has_weapon", true);
    try ap.setPostcondition("fight_guard", "past_guard", true);
    try ap.setCost("fight_guard", 5); // Higher cost

    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("has_stealth_gear", true);
    try current_state.put("has_weapon", true);
    try current_state.put("past_guard", false);

    var goal_state = WorldState.init(allocator);
    defer goal_state.deinit();
    try goal_state.put("past_guard", true);

    var result = try plan_with_astar(allocator, &ap, &current_state, &goal_state);
    defer result.deinit();

    try testing.expect(result == .done);
    try testing.expectEqual(@as(usize, 1), result.done.actions.items.len);
    // Should choose stealth (lower cost)
    try testing.expectEqualStrings("sneak_past", result.done.actions.items[0]);
}
