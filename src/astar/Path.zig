const std = @import("std");
const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;

path: *ArrayList(WorldState),
actions: *ArrayList([]const u8),
current: WorldState,
g_cost: usize = 0,
allocator: Allocator,

pub fn init(current: WorldState, allocator: Allocator) @This() {
    const path = allocator.create(ArrayList(WorldState)) catch unreachable;
    path.* = ArrayList(WorldState).empty;

    const actions = allocator.create(ArrayList([]const u8)) catch unreachable;
    actions.* = ArrayList([]const u8).empty;

    return @This(){
        .path = path,
        .actions = actions,
        .current = current,
        .allocator = allocator,
    };
}

pub fn deinit(self: *@This()) void {
    for (self.actions.items) |action| {
        self.allocator.free(action);
    }
    self.actions.deinit(self.allocator);
}

pub fn dupe(self: *@This()) !@This() {
    return self.dupeOwned(self.allocator);
}

pub fn dupeOwned(self: *@This(), allocator: Allocator) !@This() {
    const path = allocator.create(ArrayList(WorldState)) catch unreachable;
    path.* = try self.path.clone(allocator);

    const actions = allocator.create(ArrayList([]const u8)) catch unreachable;
    actions.* = .empty;

    for (self.actions.items) |action| {
        try actions.append(allocator, try allocator.dupe(u8, action));
    }

    return @This(){
        .path = path,
        .actions = actions,
        .current = self.current,
        .g_cost = self.g_cost,
        .allocator = allocator,
    };
}
