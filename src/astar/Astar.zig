const Self = @This();
const NextQueue = PriorityQueue(Path, *const WorldState, compare);

next_q: NextQueue,
seen: ArrayList(*const WorldState),
start: *const WorldState,
end: *const WorldState,
arena: *ArenaAllocator,
ap: *const ActionPlanner,
vtable: *const VTable,

pub fn init(allocator: Allocator, options: Options) !Self {
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
        .ap = options.action_planner,
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
    try self.next_q.add(Path.init(start.*, self.arena.allocator()));
    return Result{ .neighbors = start.* };
}

pub fn step(self: *Self) !Result {
    if (self.next_q.items.len == 0) {
        return Result.no_path;
    }

    var best = self.next_q.remove();

    // Check if we've already expanded this state
    for (self.seen.items) |seen_state| {
        if (utils.stateEquals(&best.current, seen_state)) {
            const next_best = self.next_q.peek() orelse return Result.no_path;
            return Result{ .neighbors = next_best.current };
        }
    }

    // Mark this state as expanded
    const current_state = try self.arena.allocator().create(WorldState);
    current_state.* = best.current;
    try self.seen.append(self.arena.allocator(), current_state);

    // Goal check
    if (self.vtable.is_goal(&best.current, self.end)) {
        try best.path.append(self.arena.allocator(), best.current);
        return Result{ .done = try DoneResult.init(self.arena.child_allocator, best) };
    }

    // Generate neighbors
    const neighbors = try self.vtable.get_neighbors(self.arena.allocator(), self.ap, &best.current);

    for (neighbors) |neighbor| {
        const state = try self.arena.allocator().create(WorldState);
        state.* = neighbor.state;

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
    const a_cost = a.g_cost + utils.distance(&a.current, end);
    const b_cost = b.g_cost + utils.distance(&b.current, end);
    return std.math.order(a_cost, b_cost);
}

const std = @import("std");
const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;
const ActionPlanner = action_planner.ActionPlanner;
const Path = @import("Path.zig");
const Options = @import("Options.zig");
const VTable = @import("VTable.zig");
const DoneResult = @import("DoneResult.zig");
const Result = @import("result.zig").Result;
const utils = @import("utils.zig");
const PriorityQueue = std.PriorityQueue;
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;
const ArenaAllocator = std.heap.ArenaAllocator;
const Order = std.math.Order;
