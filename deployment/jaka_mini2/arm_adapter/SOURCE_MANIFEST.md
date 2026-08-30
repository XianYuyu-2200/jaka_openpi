# Migrated JAKA source manifest

The files below were initially copied from
`~/codex/codex-jaka_mini_2/jaka_dual_readonly` on 2026-08-29. The hashes record
that original source snapshot; integration files may subsequently be improved
in this repository. The source repository was left untouched. Compiled
binaries, logs, caches and legacy revisions were deliberately excluded.

| Destination | Original source SHA-256 |
| --- | --- |
| `include/dry_run_core.hpp` | `c41590c8433f2cb5224ea12fef42afc9692cc62eaa9b41dc8a3f62502423e891` |
| `include/joint_sample_ipc.hpp` | `99ad58bb756fe991ebb66bbfdd776041e42cb5e10a7f800cc95d72a67e3de1bd` |
| `include/latest_packet_mailbox.hpp` | `f7369baa78369c35b47b48df40c86575bf3bb14fa34b046773ad92ac0ab6db9f` |
| `include/six_joint_mapping.hpp` | `13bf52d78ccbb8653027239f58e8db15f2ee0a41e6f433b58d07d1f4cd60cdc3` |
| `include/waypoint_recorder_core.hpp` | `cbdd802f2ad75ad078b8d6d84186ba735948fd29d95f3be90b529f3b7fbc369f` |
| `src/operator_joint_publisher.cpp` | `1056eb30b6ab469073606ec8ec86557e9ec21910451ee81bde6cb65d71d68ad3` |
| `src/six_joint_follow_test.cpp` | `4b77b1e42f2d0ea6426635787b2e8a3ed377c2683426cf2a2367851c49de1d27` |
| `src/waypoint_recorder.cpp` | `4c3cd07a551a2d5fa46c66148dc20e4b48142f0482a28008d974fb170a2e19aa` |
| `tests/dry_run_core_test.cpp` | `b159bf143862e2741a9eec32fed5181d5e094f8b3156e9018ce60b3256c7efb7` |
| `tests/waypoint_recorder_core_test.cpp` | `6d4609a2a7d5d73919436c1cbde7263217d6dd213b11727ffc354859690d9565` |
| `../config/coffee_waypoints.yaml` | `dfa737689d43d5599e77329e48f6ccefd9cfcfcfeeb584f97c8ed0694972cc15` |

The JAKA headers and `libjakaAPI.so` are not vendored. Hardware programs only
build when `BUILD_JAKA_HARDWARE_TOOLS=ON` and `JAKA_SDK_ROOT` is provided.
