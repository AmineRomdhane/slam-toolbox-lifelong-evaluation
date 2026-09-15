# V0/R1 offline map evaluation

Scores runs 004–013 against the frozen observed reference using existing trajectory
SE(2) alignments. No fitting, ROS processes, or SLAM reruns.

Run `python3 score.py <new-results-directory>`; test with
`python3 -m unittest -v test_score`. Outputs include per-run confusion metrics,
resampled maps, per-cell evidence, directed boundary distances and aggregate
statistics. Exact scoring conventions and ambiguities are recorded in report.json.
