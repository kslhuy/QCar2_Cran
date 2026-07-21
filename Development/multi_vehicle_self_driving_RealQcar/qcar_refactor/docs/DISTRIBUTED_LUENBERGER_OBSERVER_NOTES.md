# Distributed Luenberger Observer Notes

## Scope

The distributed Luenberger observer is a future fleet-local estimator, not a
dependency of the initial fleet controller. The baseline follower controller
uses only the local ego estimate and a validated fresh predecessor snapshot.

## Legacy Reference

Use `refs/qcar_origin/Observer/ShengyaObs/distributed_luenberger_estimator.py`
as the implementation reference. Its related recorder and plotting utilities
are in the same directory. That code estimates longitudinal fleet-relative
state; it must not be copied into the refactor without a separate interface,
sampling, and validation review.

## Refactor Boundary

When introduced, the observer belongs in `utils/fleet/` and has these inputs:

- the local `VehicleStateEstimate` from `utils/control/observer/`;
- validated `FleetPeerSnapshot` values from the fleet peer store;
- the formation topology and current membership revision;
- an explicit local update rate and model parameters.

It produces a distinct `FleetStateEstimate`. It does not own UDP sockets, IO,
controllers, or shared state across vehicle processes. Its output must be
published with a distinct fleet message type and explicit validity metadata.

## Preconditions

1. Keep the current predecessor-following baseline independent of the observer.
2. Reproduce the legacy longitudinal model offline with sampled updates.
3. Test packet loss, delay, out-of-order packets, membership revision changes,
   noise, and bounded observer gains.
4. Validate the longitudinal estimator before designing a separate 2D kinematic
   extension.
5. Treat its output as advisory until its fault and stale-data response is
   demonstrated; do not fuse it with the ego estimate without a defined model.
