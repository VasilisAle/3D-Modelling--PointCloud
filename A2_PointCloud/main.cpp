/*
*	Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
*	https://3d.bk.tudelft.nl/liangliang/
*
*	This file is part of the GEO1004 assignment code framework.
*/

#include "my_viewer.h"


int main(int /*argc*/, char** /*argv*/) {
    // Create the viewer (inherited from the default easy3d::Viewer).
	// Note: a viewer must be created before creating any drawables. 
    MyViewer viewer;

	// Run the viewer
    viewer.run();


    return EXIT_SUCCESS;
}

