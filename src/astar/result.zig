pub const Result = union(enum) {
    done: DoneResult,
    neighbors: WorldState,
    no_path,

    pub fn deinit(self: Result) void {
        switch (self) {
            .done => {
                var done_result = self.done;
                done_result.deinit();
            },
            else => {},
        }
    }
};

const action_planner = @import("../action_planner.zig");
const WorldState = action_planner.WorldState;
const DoneResult = @import("DoneResult.zig");
