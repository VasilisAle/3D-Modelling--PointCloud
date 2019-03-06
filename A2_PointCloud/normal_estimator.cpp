/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*	https://3d.bk.tudelft.nl/liangliang/
*
*	This file is part of the GEO1004 assignment code framework.
*/

#include "normal_estimator.h"
#include <easy3d/point_cloud.h>
#include <easy3d/kdtree.h>
#include <eigen_solver.h>


// PLEASE READ THIS!!!
//
// Well, here is another lesson on C++, the 'namespace’:
//       https://en.cppreference.com/w/cpp/language/namespace
//
// In the linear algebra assignment, you have a three dimentional vector called 'vec3'.
// You might have already noticed that there is another three dimentional vector called exactly the
// same name 'vec3' in Easy3D. In such a case, if you simply type 'vec3 v;', the compiler may get
// confused.
//
// Below shows how to allow them co-exist without confusion:
// easy3d::vec3 v(0, 0, 0);   // use the vec3 defined in the namespace easy3d
// ::vec3  v(0, 0, 0);        // use the vec3 defined in your linear algebra library (in which there is no namespace defined, we call it 'global' namespace).
//
// namespace is very useful for large projects, in which many teams may work simutaneousely on
// different modules. namespace provides the mechanism for preventing name conflicts.


// by default, use types defined in the namespace 'easy3d'
using namespace easy3d;


NormalEstimator::NormalEstimator()
{

}


NormalEstimator::~NormalEstimator()
{
    // release memory if necessary
}


// Computes the point cloud normals
void NormalEstimator::apply(easy3d::PointCloud *cloud, unsigned int k)
{
    // it is alway good to check if the input is valid
    if (cloud == nullptr)
    {
        std::cerr << "Warning: point cloud does not exist" << std::endl;
        return;
    }

    //-----------------------------------------------------------------
    std::cout << "computing point cloud normals" << std::endl;
    // to estimate the normal of each point, we need to collect it neighbors.
    // Here we use a high-perfomance KD-tree implementation. For an introduction
    // of kd-tree, see here: https://en.wikipedia.org/wiki/K-d_tree

    // create an instance of KdTree
    KdTree tree;
    tree.begin();
    // build kd-tree using the point cloud data
    tree.add_point_cloud(cloud);
    tree.end();

    // the point coordinates (in type 'vec3') are stored in the "v:point" property.
    easy3d::PointCloud::VertexProperty<easy3d::vec3> points = cloud->get_vertex_property<easy3d::vec3>("v:point");

    // the point normals (in type 'vec3') are stored in the "v:normal" property.
    easy3d::PointCloud::VertexProperty<easy3d::vec3> normals = cloud->get_vertex_property<easy3d::vec3>("v:normal");
    if (!normals)
        normals = cloud->add_vertex_property<easy3d::vec3>("v:normal");

    // IMPROTANT: the differences between 'get_vertex_property' and 'add_vertex_property'?
    //            please read the related comments in the corresponding file headers.

    for (auto v : cloud->vertices()) {
        // This is how to access the xyz coordinates (of type vec3) of a vertex 'v'.
        const easy3d::vec3& p = points[v];

        // The indices of the neighbors of v (NOTE: the result include v itself).
        std::vector<int> neighbor_indices;
        tree.find_closest_K_points(p, k, neighbor_indices);
        if (neighbor_indices.size() < k)
            return; // in extreme cases, a point cloud can have less than K points

        // now let's collect the xyz coordinates of v's neighbors
        // you can use easy3d::PointCloud::Vertex(index) to convert a vertex's index into the vertex itself, then
        // you can use it to access the vertex properties.
        std::vector<easy3d::vec3> neighbor_coordinates;
        for (std::size_t i = 0; i<neighbor_indices.size(); ++i) {
            int index = neighbor_indices[i];
            easy3d::PointCloud::Vertex vtx(index);
            const easy3d::vec3& q = points[vtx];
            neighbor_coordinates.push_back(q);
        }

        // now you have collected the xyz coordinates of the K-nearest neighbors of vertex 'v'.
        // TODO: use your own eigen solver (implemented in Assignment_1) to estimate the normal of 'v'.
        //       you can write all you code here, of course.
        //       but for simplicity, I would suggest you write a function that takes the K-nearest neighbors of 'v'
        //       as input and oupts the normal.

        ::vec3 centroid;
        for (std::size_t i=0; i<neighbor_coordinates.size(); i++)
        {
            easy3d::vec3 n =  neighbor_coordinates[i];

            centroid(0) = centroid(0) + n[0];
            centroid(1) = centroid(1) + n[1];
            centroid(2) = centroid(2) + n[2];
        }
        centroid/=k;

        ::mat3 covariance;
        for (std::size_t i=0; i<neighbor_coordinates.size(); i++)
        {
             easy3d::vec3 n =  neighbor_coordinates[i];
             ::vec3 p;

             p(0) = n[0] - centroid(0);
             p(1) = n[1] - centroid(1);
             p(2) = n[2] - centroid(2);

             for (int i=0; i<3; i++)
             {
                 for (int j=0; j<3; j++)
                 {
                     covariance (i,j) = covariance(i,j) + p(i)*p(j);
                 }
             }
        }
        covariance/=k;

        //call eigensolver function
        EigenSolver solver;
        solver.solve(covariance);

        float value;
        float next = FLT_MAX;
        easy3d::vec3 evector;

        ::vec3 vc;
        //float curvature[3];
        for (int i=0; i<3; i++)
        {      
            value = solver.get_eigen_value(i);
            //curvature[i]= solver.get_eigen_value(i);
            if (value < next)
            {
                next = value;
                vc = solver.get_eigen_vector(i);

                evector[0]= vc(0);
                evector[1]= vc(1);
                evector[2]= vc(2);
            }

        }
        //int n = sizeof(curvature)/sizeof(curvature[0]);
        //std::sort(curvature, curvature+n);
        //float t = curvature[0]/(curvature[0]+curvature[1]+curvature[2]);

        normals[v] = evector;

    }
}
