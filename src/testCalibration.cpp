/***************************************************************************
 *   testCalibratrion.cpp   - description
 *
 *   This program is part of the Etiseo project.
 *
 *   See http://www.etiseo.net  http://www.silogic.fr   
 *
 *   (C) Silogic - Etiseo Consortium
 ***************************************************************************/

 
#include <iostream>
#include <sstream>
#include <fstream>
#include "cameraModel.h"
#include "xmlUtil.h"
#include "common.h"

int test_calibration(const char *camera_xml)
{
	shared_ptr<Etiseo::CameraModel> cam = make_shared<Etiseo::CameraModel>();
	
	Etiseo::UtilXml::Init();
	
	std::cout << "******** CameraModel TEST  ********" << std::endl;
	
	std::ifstream is;
	std::cout << "Reading calibration from camera.xml ..." << std::endl;
	is.open(camera_xml);
	cam->fromXml(is);
	std::cout << " done ..." << std::endl;
	
	std::cout << " Camera position : ( " << cam->cposx() << " , " << cam->cposy()
	    << " , " << cam->cposz() << " )" << std::endl;
	
	while ( true ) {
	
		std::string operation;
		std::cin.clear();
		std::cout << " Type of operation: (1)world->img (2)img->world (3)quit" << std::endl;
		std::cin >> operation;
		if (operation == "1") {
			double x,y,z,u,v;
			std::cin.clear();
			std::cout << " X: "; std::cout.flush();
			std::cin >> operation;
			x = atof(operation.c_str());
			std::cout << " Y: "; std::cout.flush();
			std::cin >> operation;
			y = atof(operation.c_str());
			std::cout << " Z: "; std::cout.flush();
			std::cin >> operation;
			z = atof(operation.c_str());
			
			cam->worldToImage(x, y, z, u, v);
			
			std::cout << " u: " << u << " v: " << v << std::endl;
		
		} else if (operation == "2") {
			double x,y,z,u,v;
			std::cin.clear();
			std::cout << " u: "; std::cout.flush();
			std::cin >> operation;
			u = atof(operation.c_str());
			std::cout << " v: "; std::cout.flush();
			std::cin >> operation;
			v = atof(operation.c_str());
			std::cout << " Z: "; std::cout.flush();
			std::cin >> operation;
			z = atof(operation.c_str());
			
			cam->imageToWorld(u, v, z, x, y);

			std::cout << " X: " << x << " Y: " << y << " Z: " << z << std::endl;
		
		
		} else if (operation == "3")
			break;
	}
	
 
 Etiseo::UtilXml::Cleanup();
 
 std::cout << "******** CameraModel TEST  ********" << std::endl;
 
 return 0;
}
  
