const std = @import("std");
const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;

const Path = @This();

path: ArrayList(WorldState),
actions: ArrayList([]const u8),
current: WorldState,
g_cost: usize = 0,
allocator: Allocator,

pub fn init(current: WorldState, allocator: Allocator) Path {
    return Path{
        .path = .empty,
        .actions = .empty,
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
    var clone: Path = .init(self.current, self.allocator);
    clone.path = try self.path.clone(allocator);

    for (self.actions.items) |action| {
        try clone.actions.append(allocator, try allocator.dupe(u8, action));
    }

    return clone;
}
