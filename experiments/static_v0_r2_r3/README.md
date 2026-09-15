# Static V0 R2/R3 cohorts

Exact approved routes use the unchanged GT controller. `validate_routes.py` runs
three fresh-world repetitions each and stops on failed completion, tolerance,
tracking, acceleration, or conservative 0.20 m-envelope collision screening.
No contact sensor is added. A 600 s wall supervision timeout accommodates longer
routes; all controller limits and 90 s simulation deadlines remain unchanged.

After validation, `end_to_end.py` records and verifies each canonical bag, attempts
exactly ten SLAM runs, aggregates pose/resources, generates the noise-free reference
twice, and scores maps. Output paths must be new. Failures are retained, not retried.
Run from clean Jazzy + TurtleBot3 + evaluation overlays with Burger selected.

Adapters reuse frozen R1 source and save their exact emitted code with outputs.
Only dataset paths/IDs, measured counts/durations, and expected hashes change.
Trajectory alignment, association, RPE, SLAM parameters, resource windows, grid
construction and map scoring functions are unchanged. Replay uses the validated
endpoint-GID clock guard and accepted count/timestamp checks, not the superseded
raw-CDR/reserialization byte-hash comparison. Reference scans outside recorded GT
support are omitted, never extrapolated; actual association counts are reported.
`check_adapters.py` compiles emitted recorder/validator/replay/SLAM scripts without
running them. Full source and data hashes are recorded with each cohort.
