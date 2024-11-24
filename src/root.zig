const std = @import("std");
const testing = std.testing;
const astar = @import("astar.zig");

export fn add(a: i32, b: i32) i32 {
    return a + b;
}

test {
    testing.refAllDecls(@This());
}
