pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();

    var action_planner = try ActionPlanner.init(allocator);
    defer action_planner.deinit();

    try action_planner.setPrecondition("scout", "armedwithgun", true);
    try action_planner.setPostcondition("scout", "enemyvisible", true);

    try action_planner.setPrecondition("approach", "enemyvisible", true);
    try action_planner.setPostcondition("approach", "nearenemy", true);

    try action_planner.setPrecondition("aim", "enemyvisible", true);
    try action_planner.setPrecondition("aim", "weaponloaded", true);
    try action_planner.setPostcondition("aim", "enemylinedup", true);

    try action_planner.setPrecondition("shoot", "enemylinedup", true);
    try action_planner.setPostcondition("shoot", "enemyalive", false);

    try action_planner.setPrecondition("load", "armedwithgun", true);
    try action_planner.setPostcondition("load", "weaponloaded", true);

    try action_planner.setPrecondition("detonatebomb", "armedwithbomb", true);
    try action_planner.setPrecondition("detonatebomb", "nearenemy", true);
    try action_planner.setPostcondition("detonatebomb", "alive", false);
    try action_planner.setPostcondition("detonatebomb", "enemyalive", false);

    try action_planner.setPrecondition("flee", "enemyvisible", true);
    try action_planner.setPostcondition("flee", "nearenemy", false);

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
}

fn heuristic(current: *const WorldState, goal: *const WorldState) usize {
    var count: usize = 0;
    var it = goal.iterator();
    while (it.next()) |entry| {
        const atom_name = entry.key;
        const goal_value = entry.value;
        const current_value = current.get(atom_name) orelse return error.CurrentStateIncomplete;

        if (current_value != goal_value) {
            count += 1;
        }
    }
    return count;
}

fn is_goal(current: *const WorldState, goal: *const WorldState) bool {
    var it = goal.iterator();
    while (it.next()) |entry| {
        const atom_name = entry.key;
        const goal_value = entry.value;
        const current_value = current.get(atom_name) orelse return error.CurrentStateIncomplete;

        if (current_value != goal_value) {
            return false;
        }
    }
    return true;
}

test {
    std.testing.refAllDecls(@This());
}

const std = @import("std");
const astar = @import("astar.zig");
const ActionPlanner = @import("action_planner.zig").ActionPlanner;
const WorldState = @import("action_planner.zig").WorldState;
