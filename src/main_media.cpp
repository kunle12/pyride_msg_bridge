//
//  pyride_media_bridge.cpp
//  PyRIDE
//
//  Created by Xun Wang on 12/06/19.
//  Copyright (c) 2019 Xun Wang. All rights reserved.
//

#include <ros/ros.h>

#include "pyride_media_bridge.h"

using namespace pyride;

int main( int argc, char * argv[] )
{
  ros::init( argc, argv, "pyride_media_bridge" );

  PyRIDEMediaBridge s_server;
  
  s_server.init();

  s_server.continueProcessing();
  
  s_server.fini();
  
  ros::shutdown();
  return 0;
}
