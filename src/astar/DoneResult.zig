const std = @import("std");
const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;
const Path = @import("Path.zig");
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;
const ArenaAllocator = std.heap.ArenaAllocator;

path: *ArrayList(WorldState),
actions: *ArrayList([]const u8),
arena: *ArenaAllocator,

pub fn init(allocator: Allocator, path: Path) !@This() {
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

    return @This(){
        .path = clone_path,
        .actions = actions,
        .arena = arena,
    };
}

pub fn deinit(self: *@This()) void {
    const allocator = self.arena.child_allocator;
    self.arena.deinit();
    allocator.destroy(self.arena);
}
