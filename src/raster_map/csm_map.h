//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    csm_map.h
\brief   Implements XY Map for Correlative Scan Matching
\author  Melissa Cruz, Yuhong Kan, Maxx Wilson
*/
//========================================================================

#include "shared/math/line2d.h"
#include "vector_map/vector_map.h"

#include <vector>
#include <utility>

#include "raster_map/xy_raster_map.h"

#ifndef CSM_MAP_H
#define CSM_MAP_H

namespace csm_map{

class CSMMap : public xy_raster_map::XYRasterMap {
    public:
        CSMMap();

        /**
         * @brief Construct a new CSMMap object with initial size and grid values
         * 
         * @param map_dimension length/width of map in meters
         * @param dist_res dimension of map grid squares in meters
         * @param init_grid_val initial grid values as double
         * @param default_range_max maximum range to consider scan points
         * @param sigma_observation standard deviation of laser scan range
         */
        CSMMap(double map_dimension, double dist_res, double init_grid_val,  double default_range_max, double sigma_observation);

        /**
         * @brief Clears scan map and regenerates given a new laser scan
         * 
         * @param cloud laser scan as cartesian points
         */
        void GenerateMapFromNewScan(const std::vector<Eigen::Vector2f> &cloud);

        /**
         * @brief Set the Log Likelihood at given position
         * 
         * @param x x distance in meters
         * @param y y distance in meters
         * @param likelihood log likelihood as a double
         */
        void SetLogLikelihoodAtPosition(const double x, const double y, const double likelihood);

        /**
         * @brief Get the Log Likelihood at given position
         * 
         * @param x x distance in meters
         * @param y y distance in meters
         * @return double log likelihood
         */
        double GetLogLikelihoodAtPosition(const double x, const double y) const;

        /**
         * @brief Get the Likelihood at given position normalzied by the max value in image
         * 
         * @param x x distance in meters
         * @param y y distance in meters
         * @param likelihood normalized likelihood as a double
         */
        double GetNormalizedLikelihoodAtPosition(const double x, const double y) const;

        /**
         * @brief Get the Likelihood at given position as standard gaussian probability (area under curve of 1)
         * 
         * @param x x distance in meters
         * @param y y distance in meters
         * @param likelihood standard likelihood as a double
         */
        double GetStandardLikelihoodAtPosition(const double x, const double y) const;

        /**
         * @brief Converts log likelihood to standard gaussian probability sample
         * 
         * @param log_likelihood 
         * @return double standard likelihood
         */
        double ConvertLogToStandard(const double log_likelihood) const;

        /**
         * @brief updates image object, drawing the current values in the probability map
         */
        void DrawCSMImage();

        /**
         * @brief Draws points on the image as 3x3 squares
         * 
         * @param cloud vector of cartesian points
         */
        void DrawScanCloudOnImage(const std::vector<Eigen::Vector2f> &cloud, const double range_max, bool alt_color);

        /**
         * @brief Returns the maximum range observed in the current rasterized scan
         * 
         * @return maximum range observed in scan as double in meters
         */
        double GetMaxRangeInScan();

    private:
        double default_range_max;
        double sigma_observation;
        double max_range_in_scan = 0.0;

};

}

#endif   // CSM_MAP_H