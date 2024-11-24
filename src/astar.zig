const std = @import("std");
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
        path.* = ArrayList(WorldState).init(allocator);

        const actions = allocator.create(ArrayList([]const u8)) catch unreachable;
        actions.* = ArrayList([]const u8).init(allocator);

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
        self.actions.deinit();
    }

    pub fn dupe(self: *Path) !Path {
        return self.dupeOwned(self.allocator);
    }

    pub fn dupeOwned(self: *Path, allocator: Allocator) !Path {
        const path = allocator.create(ArrayList(WorldState)) catch unreachable;
        path.* = try self.path.clone();

        const actions = allocator.create(ArrayList([]const u8)) catch unreachable;
        actions.* = ArrayList([]const u8).init(allocator);

        for (self.actions.items) |action| {
            try actions.append(try allocator.dupe(u8, action));
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
        clone_path.* = ArrayList(WorldState).init(arena_allocator);

        for (path.path.items) |ws| {
            try clone_path.append(try ws.cloneWithAllocator(arena_allocator));
        }

        const actions = try arena_allocator.create(ArrayList([]const u8));
        actions.* = ArrayList([]const u8).init(arena_allocator);

        for (path.actions.items) |action| {
            try actions.append(try arena_allocator.dupe(u8, action));
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
        const seen = ArrayList(*const WorldState).init(arena_alloc);

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
        try self.seen.append(start);
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
            try best.path.append(best.current);
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

            try self.seen.append(state);

            var new_path = try best.dupe();
            try new_path.path.append(best.current);
            try new_path.actions.append(neighbor.action_name);

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
