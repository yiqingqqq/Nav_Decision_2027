# RMUL2027 semantic navigation contract

`RMUL2027_semantic_map.yaml` is the authority for field geometry and route
topology. Its transform is named `map_T_field` and always means
`p_map = map_T_field * p_field`. BT XML contains semantic identifiers, never
competition coordinates.

The currently measured tunnel centerline is field `y=-3.60` (map `y=0.60`).
The older implementation-plan table says field `y=-3.40` (map `y=0.40`),
while the verified controller code uses `-3.60`. This change preserves the
verified controller value; the discrepancy must be resolved by a field survey.

The center target is the confirmed Gazebo/world point `(0,0)`. With the
configured transform it resolves to map `(5,-3)`. Supply remains world
`(5,-3)`, which resolves to map `(0,0)`, so the two semantic regions remain
separate.

Normal edges are executed with NavigateToPose. The tunnel edge alone owns an
ordered centerline and is executed directly with FollowPath and
TunnelController. A committed failure projects the live robot pose onto that
ordered line and walks its prefix backwards; it never invokes a planner or
Spin in the tunnel.

Runtime ownership is deliberately singular: the root `GetCurrentPose` is the
only TF pose producer. Region classification and retreat projection consume
that shared `PoseStamped`. `LockedSemanticTask` resolves and selects a route
once when a new task becomes active; the route is unlocked only when the task
branch is halted by a task transition.
