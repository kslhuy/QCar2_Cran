include "qcar2_2d.lua"

-- AMCL owns saved-map localization (map -> odom) in this mode.
-- Cartographer is kept only as a live scan-matching odometry source
-- for the local odom -> base_link transform.
options.map_frame = "odom"
options.odom_frame = "odom"
options.published_frame = "base_link"
options.provide_odom_frame = false

-- We only need local odometry here, not global SLAM loop closures.
POSE_GRAPH.optimize_every_n_nodes = 0

return options
