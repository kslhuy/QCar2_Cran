# V2V Utilities

## 1. Introduction

`utils/v2v` provides generic peer datagram transport for fleet services.

## 2. File structure and variations

[V2V base](v2v-base.md) defines base/null behavior; [V2V UDP](v2v-udp.md)
adds socket, receive-thread, peer-resolution, and rate-limit mechanics.

## 3. Shared data and cross-references

All implementations publish and drain [[vehicle-types|V2VMessage]] records
with freshness diagnostics; payload interpretation belongs to [[fleet-manager|FleetManager]].

## 4. Position in the project

The package sits below fleet and outside global lifecycle/actuation authority.

## 5. Use and verification

Select a V2V profile through configuration and use `test/unit_test_v2v.py`.

## Conclusion

All V2V implementations share a generic message contract and differ only by
transport; fleet policy and vehicle safety remain above this boundary.

