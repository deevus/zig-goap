action_planner: *const ActionPlanner,
start: *const WorldState,
vtable: *const VTable,

const ap = @import("../action_planner.zig");
const ActionPlanner = ap.ActionPlanner;
const WorldState = ap.WorldState;
const VTable = @import("VTable.zig");
