#ifndef MY_VIEWER_H
#define MY_VIEWER_H
/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*	https://3d.bk.tudelft.nl/liangliang/
*
*	This file is part of the GEO1004 assignment code framework.
*/

#include <easy3d/viewer.h>


class MyViewer : public easy3d::Viewer
{
public:
    MyViewer();
    ~MyViewer();

    // Returns the pointer of the loaded point cloud.
    // In case the model doesn't exist (e.g., not loaded to the viewer),
    // it will return NULL (== 0).
    easy3d::PointCloud* point_cloud();

    // Create a drawable and upload the necessary data to GPU to
    // visualize the normals.
    // The input to this function is the point cloud.
    void creat_normal_drawable(easy3d::PointCloud* cloud);

private:
    bool key_press_event(int key, int modifiers);

};

#endif
