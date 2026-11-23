# stage3-test-learner

Test controller for the KFUPM F1TENTH Stage 3 platform (organizer test, not a team).

## Official run

- Package: `kfupm_test_learner`
- Entry point: `ros2 run kfupm_test_learner controller`
- Starts moving: within 2 s

Practice: drives reactively and records one lap into F1TENTH_DATA_DIR. Official run: follows a racing line built from that lap; reactive if nothing was saved.
