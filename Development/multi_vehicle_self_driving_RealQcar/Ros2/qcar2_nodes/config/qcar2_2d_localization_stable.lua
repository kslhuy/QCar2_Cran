include "qcar2_2d.lua"

-- Stability-first localization profile.
--
-- On this QCar2 Humble setup, the pure-localization trimmer path has been seen
-- to abort at runtime with `free(): invalid pointer` after several minutes.
-- Keeping the loaded map frozen is still handled by the launch file via
-- `-load_frozen_state true`, so this config avoids the trimmer/extra
-- localization-only overrides and instead uses the base 2D config with
-- slightly stricter matching thresholds.
POSE_GRAPH.constraint_builder.min_score = 0.7
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75

return options
