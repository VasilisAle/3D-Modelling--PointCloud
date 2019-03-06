/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*	https://3d.bk.tudelft.nl/liangliang/
*
*	This file is part of the GEO1004 assignment code framework.
*/

#include "my_viewer.h"

#include <3rd_party/glew/include/GL/glew.h>		// for OpenGL functions
#include <3rd_party/glfw/include/GLFW/glfw3.h>	// for glfw functions

#include <easy3d/drawable.h>
#include <easy3d/point_cloud.h>

#include "normal_estimator.h"


MyViewer::MyViewer()
    : easy3d::Viewer("A2_PointCloud")
{
    std::cout <<
                 "-------------------------------------------------------------------\n"
                 "  E:               Estimation point cloud normals                  \n"
                 "  N:               Toggle the visualization of the normals" << std::endl;
}


MyViewer::~MyViewer()
{
}


bool MyViewer::key_press_event(int key, int modifiers)
{
    if (key == GLFW_KEY_E) {
        easy3d::PointCloud* cloud = point_cloud();
        NormalEstimator estimator;
        estimator.apply(cloud);

        easy3d::PointCloud::VertexProperty<easy3d::vec3> normals = cloud->get_vertex_property<easy3d::vec3>("v:normal");
        if (normals) { // normal estimation may fail, make sure the normals exist
            // -------------------------------------------------------
            // now we have the normal information, we can have a better
            // visualization of the point cloud. But we first need to
            // upload the normal information to GPU.
            easy3d::PointsDrawable* drawable = cloud->points_drawable("points");
            drawable->update_normal_buffer(normals.vector());

            auto colors = cloud->get_vertex_property< easy3d::vec3>("v:color");
            if (colors)// if colors exist
                drawable->update_color_buffer(colors.vector());

            drawable->set_per_vertex_color(colors); // set to true if has color property
            drawable->set_default_color( easy3d::vec3(0.4f, 0.8f, 0.8f));
            drawable->set_point_size(6.0f);
            // -------------------------------------------------------
            // this is for visualizing the normal vectors
            creat_normal_drawable(cloud);
        }

        return true;
    }
    else if (key == GLFW_KEY_N) {
        easy3d::PointCloud* cloud = point_cloud();
        if (cloud) {
            easy3d::LinesDrawable* lines = cloud->lines_drawable("normals");
            if (lines) {
                bool visible = lines->is_visible();
                lines->set_visible(!visible);
                update();
            }
            else
                creat_normal_drawable(cloud);
        }
        else
            std::cerr << "Warning: point cloud does not exist" << std::endl;
        return true;
    }

    else
        return Viewer::key_press_event(key, modifiers);
}


// Returns the pointer of the loaded point cloud.
// In case the model doesn't exist (e.g., not loaded to the viewer),
// it will return NULL (== 0).
easy3d::PointCloud* MyViewer::point_cloud()
{
    easy3d::PointCloud* cloud = dynamic_cast<easy3d::PointCloud*>(current_model());
    return cloud;
}


// create a drawable for visualizing the normals.
void MyViewer::creat_normal_drawable(easy3d::PointCloud* cloud)
{
    // it is always good to check if the input is valid
    if (cloud == nullptr)
    {
        std::cerr << "Warning: point cloud does not exist" << std::endl;
        return;
    }

    // the point normals (in type 'vec3') are stored in the "v:normal" property.
    easy3d::PointCloud::VertexProperty<easy3d::vec3> normals = cloud->get_vertex_property<easy3d::vec3>("v:normal");

    if (!normals)
    {
        std::cerr << "Warning: normal information does not exist" << std::endl;
        return;
    }

    // create a drawable and upload the necessary data to GPU to visualize the normals.
    // Note: a user may press the same button (i.e., running this function multiple times,
    //       so a good software implementation should check if the drawable already exists.
    //          - if no, create the drawable
    //          - if yes, no need to create the drawable, but just update the data transfered to
    //            the GPU to update the visualization.
    // each drawable should have a unique name. Here we use "normals"



   /* easy3d::LinesDrawable* lines = cloud->lines_drawable("normals");
    if (!lines)
    {// this is equal to "lines == nullptr"
        std::cout << "creating drawable...[not implemented yet]" << std::endl;
        lines = cloud->add_lines_drawable("normals");
    }*/

    //-----------------------------------------------------------------
    // TODO: prepare the normal data (each normal is a line segment
    //       represented by its two end points). You can store them in
    //       a array of "std::vector". Check here for its usage:
    //       https://en.cppreference.com/w/cpp/container/vector
    easy3d::PointCloud::VertexProperty<easy3d::vec3> points = cloud->get_vertex_property<easy3d::vec3>("v:point");

    if (normals)
    {
        //length of the normals
        float length =0.09f;

        // Every consecutive two points represent a normal vector.
        std::vector<easy3d::vec3> normal_points;

        //iterate through the vertices of the pointcloud::cloud
        for (auto v : cloud->vertices())
        {
            const easy3d::vec3& s = points[v];
            easy3d::vec3 n = normals[v];

            n.normalize();

            const easy3d::vec3& t = points[v] + n * length;

            normal_points.push_back(s);
            normal_points.push_back(t);
        }

        // Create a drawable for rendering the normal vectors.
        easy3d::LinesDrawable* normals_drawable = cloud->add_lines_drawable("normals");

        if (!normals_drawable)
        {// this is equal to "lines == nullptr"
            std::cout << "creating drawable normals..." << std::endl;
            normals_drawable = cloud->add_lines_drawable("normals");
        }

        // Upload the data to the GPU.
        normals_drawable->update_vertex_buffer(normal_points);
        // We will draw the normal vectors in green color
        normals_drawable->set_per_vertex_color(false);
        normals_drawable->set_default_color(easy3d::vec3(0.0f, 0.0f, 0.0f));//white colour
    }
    update();
}
