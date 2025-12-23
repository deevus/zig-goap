const std = @import("std");
const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;
const ActionPlanner = action_planner.ActionPlanner;
const ActionError = action_planner.ActionError;
const Neighbor = @import("Neighbor.zig");

distance: *const fn (*const WorldState, *const WorldState) usize,
is_goal: *const fn (*const WorldState, *const WorldState) bool,
get_neighbors: *const fn (std.mem.Allocator, ap: *const ActionPlanner, *const WorldState) ActionError![]Neighbor,
