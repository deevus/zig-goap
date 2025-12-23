const action_planner = @import("../action_planner.zig");
const ActionPlanner = action_planner.ActionPlanner;
const WorldState = action_planner.WorldState;
const VTable = @import("VTable.zig");

actionPlanner: *const ActionPlanner,
start: *const WorldState,
vtable: *const VTable,
