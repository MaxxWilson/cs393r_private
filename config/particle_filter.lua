map = "maps/GDC1.txt"
init_x = 14.7
init_y = 14.24
init_r = 0

-- Vehicle Constants --
laser_offset = 0.2
range_min = 0.02
range_max = 9.5

-- Initial cloud distribution --
init_x_sigma = 0.3 -- 99% of the time, within a meter
init_y_sigma = 0.3 -- 99% of the time, within a meter
init_r_sigma = 0.3  -- 99% of the time, its within 35 deg

-- Motion Model Params --
k1 = 2.0   -- x error from translation         -- 95% of translations are within 15% margin of error
k2 = 2.0   -- x error from rotation            -- This is effectively zero given the small angle approx
k3 = 2.0   -- rotation error from translation  -- at 1m, 99% of values within 7 deg
k4 = 3.0    -- rotation error from rotation     -- 95% of translations are within 15% margin of error

min_update_dist = 0.03
min_update_angle = 0.05

-- Limited by computation --
num_particles = 200 -- Increase until computation runs out
resize_factor = 5          -- # num_points / resize_factor = num_rays

sigma_observation = 0.1    -- Prof recommends 0.15-0.2 based on sensor specs
gamma = 0.04 -- 0.01                -- TODO Experimental tuning

-- Limits maximum weight error --
-- Increasing these makes it harsher on short/long errors for scan
dist_short = 0.23   -- 1 std from sensor 68.2%
dist_long = 0.28     -- 2 std from sensor 95%

resample_frequency = 10     -- TODO Experimental tuning

-- Cost Map --
dist_update_thresh = 0.3
angle_update_thresh = 0.2 -- 15°

-- CSM Search --

csm_sigma_observation = 0.1
csm_gamma = 0.05

low_theta_res = 0.08 -- 0.05 -- 50 cm
low_dist_res = 0.25 -- 0.05 -- 50 cm

dist_res = 0.05 -- 0.05 -- 50 cm
theta_res = 0.04 -- 0.02 -- ~5°
csm_eval_range_max =  8.5

theta_search_range = 0.3
dist_search_range = 0.3

map_length_dist = dist_update_thresh + range_max + laser_offset + 4*sigma_observation + 0.5
row_num = 2*(map_length_dist)/dist_res + 1

min_map_prob = -60
csm_resize = 1

localization_mode = "ekf" -- "odom", "lidar", "ekf", "ekf_pf"