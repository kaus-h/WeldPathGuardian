# Failure Modes

## Invalid Points

NaN and infinite coordinates are rejected in `seam_perception` before smoothing or planning.

## Low Confidence

Points below the configured confidence threshold are rejected. If too few points remain, the perception output is marked invalid and the planner returns `InsufficientPoints`.

## Stale Sensor Input

The sensor can publish deliberately old timestamps in the `stale_messages` scenario. Perception compares the message stamp to node time and rejects stale observations.

## Missing Segment

The `missing_segment` scenario creates large gaps in the seam. The planner checks neighboring point distance and rejects paths with `ExcessiveGap`.

## Sudden Offset

The `sudden_offset` scenario shifts part of the seam. Perception and planning still produce a path if geometry remains valid, while monitor metrics expose the changed output.

## Complete Dropout

The `complete_dropout` scenario publishes empty/invalid observations. The pipeline transitions into invalid perception and planning failure instead of executing unsafe waypoints.

## Low Confidence Recovery

The `low_confidence_recovery` scenario temporarily lowers confidence below the perception threshold. The executor enters `PAUSED`, then resumes when valid observations return and reports recovery time.

## Cancellation

`weld_executor` exposes a cancellable `ExecuteWeld` action. Cancellation is represented with an atomic flag and is checked between waypoint advances.

## External Fault During Action Execution

If `/weld/plan` publishes an invalid plan while an `ExecuteWeld` action is active, the executor transitions to `FAULTED` or `PAUSED` depending on the fault. Accepted action goals always terminate with a result: external faults abort the action with the current fault code, cancellations return a canceled result, and successful completion returns `None`.
