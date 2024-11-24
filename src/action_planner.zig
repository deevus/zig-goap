const std = @import("std");

const Allocator = std.mem.Allocator;
const ArenaAllocator = std.heap.ArenaAllocator;

pub const WorldState = std.StringHashMap(bool);

pub const ActionPlanner = struct {
    allocator: Allocator,
    arena: *ArenaAllocator,
    costs: std.StringHashMap(u32),
    preconditions: std.StringHashMap(*WorldState),
    postconditions: std.StringHashMap(*WorldState),

    pub fn init(allocator: Allocator) !ActionPlanner {
        const arena = try allocator.create(ArenaAllocator);
        arena.* = ArenaAllocator.init(allocator);

        return ActionPlanner{
            .allocator = allocator,
            .arena = arena,
            .costs = std.StringHashMap(u32).init(allocator),
            .preconditions = std.StringHashMap(*WorldState).init(allocator),
            .postconditions = std.StringHashMap(*WorldState).init(allocator),
        };
    }

    pub fn deinit(self: *ActionPlanner) void {
        self.arena.deinit();
        self.costs.deinit();
        self.preconditions.deinit();
        self.postconditions.deinit();
        self.allocator.destroy(self.arena);
    }

    pub fn setPrecondition(self: *ActionPlanner, comptime action_name: []const u8, comptime atom_name: []const u8, value: bool) !void {
        try set_condition(self.arena.allocator(), &self.preconditions, action_name, atom_name, value);
    }

    pub fn setPostcondition(self: *ActionPlanner, comptime action_name: []const u8, comptime atom_name: []const u8, value: bool) !void {
        try set_condition(self.arena.allocator(), &self.postconditions, action_name, atom_name, value);
    }

    pub fn setCost(self: *ActionPlanner, comptime action_name: []const u8, cost: u32) !void {
        try self.costs.put(action_name, cost);
    }
};

inline fn set_condition(allocator: Allocator, map: *std.StringHashMap(*WorldState), comptime action_name: []const u8, comptime atom_name: []const u8, value: bool) !void {
    if (!map.contains(action_name)) {
        const state_map = try allocator.create(WorldState);
        state_map.* = WorldState.init(allocator);
        try map.put(action_name, state_map);
    }

    var state_map = map.get(action_name).?;
    try state_map.put(atom_name, value);
}

pub const ActionError = error{
    ActionNotFound,
} || Allocator.Error;

pub fn do_action(action_planner: *const ActionPlanner, world_state: *WorldState, action_name: []const u8) ActionError!void {
    const action_postconditions = action_planner.postconditions.get(action_name) orelse return error.ActionNotFound;

    var it = action_postconditions.iterator();
    while (it.next()) |entry| {
        const res = try world_state.getOrPut(entry.key_ptr.*);
        res.value_ptr.* = entry.value_ptr.*;
    }
}

const PossibleTransition = struct {
    action_name: []const u8,
    action_cost: u32,
    to_state: WorldState,
    allocator: Allocator,

    pub fn init(allocator: Allocator, action_name: []const u8, cost: u32, to_state: WorldState) !PossibleTransition {
        return PossibleTransition{
            .action_name = action_name,
            .action_cost = cost,
            .to_state = to_state,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *@This()) void {
        self.to_state.deinit();
    }
};

pub const PossibleTransitions = struct {
    items: []PossibleTransition,
    allocator: Allocator,

    pub fn init(allocator: Allocator, items: []PossibleTransition) !PossibleTransitions {
        return .{
            .allocator = allocator,
            .items = items,
        };
    }

    pub fn deinit(self: *PossibleTransitions) void {
        for (0..self.items.len) |i| {
            self.items[i].deinit();
        }

        self.allocator.free(self.items);
    }
};

pub fn get_possible_state_transitions(
    allocator: Allocator,
    action_planner: *const ActionPlanner,
    fr: *const WorldState,
) ActionError!PossibleTransitions {
    var transitions = std.ArrayList(PossibleTransition).init(allocator);
    defer transitions.deinit();

    var it = action_planner.preconditions.iterator();
    while (it.next()) |entry| {
        const action_name = entry.key_ptr.*;
        const preconditions: *WorldState = entry.value_ptr.*;

        if (are_preconditions_met(preconditions, fr)) {
            // Get the action cost
            const action_cost = action_planner.costs.get(action_name) orelse 1;

            // Apply the action to get the resulting state
            var to_state = try fr.cloneWithAllocator(allocator);
            try do_action(action_planner, &to_state, action_name);

            // Store the possible transition
            try transitions.append(try PossibleTransition.init(allocator, action_name, action_cost, to_state));
        }
    }

    return try PossibleTransitions.init(allocator, try transitions.toOwnedSlice());
}

fn are_preconditions_met(
    preconditions: *const WorldState,
    fr: *const WorldState,
) bool {
    var it = preconditions.iterator();
    while (it.next()) |entry| {
        const key = entry.key_ptr.*;
        const required_value = entry.value_ptr.*;
        const current_value = fr.get(key) orelse false;
        if (current_value != required_value) {
            return false;
        }
    }
    return true;
}

const testing = std.testing;

test "set precondition" {
    var action_planner = try ActionPlanner.init(testing.allocator);
    defer action_planner.deinit();

    try action_planner.setPrecondition("scout", "armedwithgun", true);

    const precondition = action_planner.preconditions.get("scout").?;
    const armed_with_gun = precondition.get("armedwithgun") orelse false;

    try testing.expect(armed_with_gun);
}

test "do_action applies action postconditions to world state" {
    var action_planner = try ActionPlanner.init(testing.allocator);
    defer action_planner.deinit();

    // Set up action postconditions
    try action_planner.setPostcondition("scout", "enemyvisible", true);

    // Initialize world state
    var world_state = WorldState.init(testing.allocator);
    defer world_state.deinit();
    try world_state.put("enemyvisible", false);

    // Apply the "scout" action
    try do_action(&action_planner, &world_state, "scout");

    // Verify that "enemyvisible" is now true
    const enemy_visible = world_state.get("enemyvisible") orelse false;
    try std.testing.expect(enemy_visible);
}

test "get_possible_state_transitions returns correct transitions" {
    const allocator = std.testing.allocator;

    var action_planner = try ActionPlanner.init(allocator);
    defer action_planner.deinit();

    // Set up actions, preconditions, postconditions, and costs
    try action_planner.setPrecondition("scout", "armedwithgun", true);
    try action_planner.setPostcondition("scout", "enemyvisible", true);
    try action_planner.setCost("scout", 1);

    try action_planner.setPrecondition("approach", "enemyvisible", true);
    try action_planner.setPostcondition("approach", "nearenemy", true);
    try action_planner.setCost("approach", 2);

    // Initialize current world state
    var current_state = WorldState.init(allocator);
    defer current_state.deinit();
    try current_state.put("armedwithgun", true);
    try current_state.put("enemyvisible", false);

    // Get possible transitions
    var transitions = try get_possible_state_transitions(
        allocator,
        &action_planner,
        &current_state,
    );
    defer transitions.deinit();

    // Verify the transitions
    try std.testing.expectEqual(1, transitions.items.len);
    const transition = transitions.items[0];
    try std.testing.expectEqualStrings("scout", transition.action_name);
    try std.testing.expectEqual(1, transition.action_cost);

    const enemy_visible = transition.to_state.get("enemyvisible") orelse false;
    try std.testing.expect(enemy_visible);
}
