#ifndef NORMAL_ESTIMATOR_H
#define NORMAL_ESTIMATOR_H

/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*	https://3d.bk.tudelft.nl/liangliang/
*
*	This file is part of the GEO1004 assignment code framework.
*/

namespace easy3d {
    class PointCloud;
}


class NormalEstimator
{
public:
    NormalEstimator();
    ~NormalEstimator();

    // Computes the point cloud normals. After execution, the point normals
    // (in type 'vec3') are stored in the "v:normal" property of the point cloud.
    // TParameters:
    //   - cloud: the input point cloud.
    //   - k: the number of neighers (with default value 16)unsigned int
    void apply(easy3d::PointCloud* cloud, unsigned int k = 16);

private:
    // This class has only one public function apply(). Of course, you can
    // add some private functions to be used internally. But from the user's
    // point of view, the apply() function is the only one he/she needs. So
    // please keep user interface as clean as possible (i.e., not add any
    // public functions).

};


#endif
