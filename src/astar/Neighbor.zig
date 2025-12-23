const Self = @This();

state: WorldState,
action_name: []const u8,
cost: usize,

pub fn init(state: WorldState, action_name: []const u8, cost: usize) Self {
    return Self{
        .state = state,
        .action_name = action_name,
        .cost = cost,
    };
}

const std = @import("std");
const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;
const Allocator = std.mem.Allocator;
