const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;

/// Checks if two WorldStates are exactly equal (same keys and values).
pub fn stateEquals(a: *const WorldState, b: *const WorldState) bool {
    var it = a.iterator();
    while (it.next()) |entry| {
        const atom_name = entry.key_ptr.*;
        const a_value = entry.value_ptr.*;

        const maybe_b_value = b.get(atom_name);
        if (maybe_b_value) |b_value| {
            if (a_value != b_value) {
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

/// Calculates the heuristic distance between current and goal states.
/// Returns the count of atoms that differ between the two states.
pub fn distance(current: *const WorldState, goal: *const WorldState) usize {
    var count: usize = 0;
    var it = goal.iterator();
    while (it.next()) |entry| {
        const atom_name = entry.key_ptr.*;
        const goal_value = entry.value_ptr.*;

        const maybe_current_value = current.get(atom_name);
        if (maybe_current_value) |current_value| {
            if (current_value != goal_value) {
                count += 1;
            }
        } else {
            // Atom is absent in current state, counts as a difference
            count += 1;
        }
    }
    return count;
}
