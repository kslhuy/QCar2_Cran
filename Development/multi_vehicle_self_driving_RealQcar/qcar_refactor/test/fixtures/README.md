# Test Fixtures

This directory contains deterministic, committed test inputs. Fixtures are
read by tests and must never be overwritten by a test run.

Generated evidence belongs under `test/artifacts/<category>/<platform>/<test_name>/<run_id>/` and is ignored by Git. The QCar IO recording in `qcar/` is an EKF replay input, not a generated artifact.
