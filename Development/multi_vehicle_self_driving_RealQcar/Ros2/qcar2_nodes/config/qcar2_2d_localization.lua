include "qcar2_2d.lua"

-- Keep only a small rolling set of live submaps during localization.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

-- Lower optimization frequency for smoother runtime localization updates.
POSE_GRAPH.optimize_every_n_nodes = 25

-- Slightly stricter matching thresholds reduce accidental relocalization jumps.
POSE_GRAPH.constraint_builder.min_score = 0.7
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75

return options
