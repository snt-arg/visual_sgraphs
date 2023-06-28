/**
 * This file is added to ORB-SLAM3 to augment semantic data.
 *
 * Copyright (C) 2022 A. Tourani, H. Bavle, J. L. Sanchez-Lopez, and H. Voos - SnT University of Luxembourg.
 *
 */

#ifndef WALL_H
#define WALL_H

#include <Eigen/Eigen>

using namespace std;

namespace ORB_SLAM3
{
    struct Wall
    {
        // The identifier of each wall
        int id;
        // The identifier of each wall in the graph
        int graphId;
        // The plane equation of the wall
        vector<double> plane_equation;
        // the markers lying on the wall
        vector<int> marker_ids;
        // set of map points lying on the wall
        // vector<MapPoint> mapPoints;
        // the node in the graph
        // g2o::VertexPlane *planeNode;
        // color of visualization
        vector<double> color;
    };
}

#endif