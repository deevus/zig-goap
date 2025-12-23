const std = @import("std");
const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;
const Allocator = std.mem.Allocator;

state: WorldState,
action_name: []const u8,
cost: usize,
allocator: Allocator,

pub fn init(allocator: Allocator, state: WorldState, action_name: []const u8, cost: usize) !@This() {
    return @This(){
        .state = state,
        .action_name = action_name,
        .cost = cost,
        .allocator = allocator,
    };
}
