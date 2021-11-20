-- Map Dimensions --
dist_res = 0.1 -- 50 cm

-- Observation Model --
sigma_observation = 0.2
range_max = 5.0

-- Map Resize --
map_length_dist = 50
row_num = 2*(map_length_dist)/dist_res + 1
dilation_factor = 1

-- Global Planner --
global_planner_steps = 5000
global_planner_curvature_num = 20
global_planner_step_length = 0.1
global_planner_per_step_num = 20;
goal_bias = 0.05