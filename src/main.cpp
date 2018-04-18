//
//  pyride_msg_bridge.cpp
//  PyRIDE
//
//  Created by Xun Wang on 18/04/17.
//  Copyright (c) 2017 Xun Wang. All rights reserved.
//

#include <ros/ros.h>

#include "pyride_msg_bridge.h"

using namespace pyride;

int main( int argc, char * argv[] )
{
  ros::init( argc, argv, "pyride_msg_bridge" );

  PyRIDEMsgBridge s_server;
  
  s_server.init();

  s_server.continueProcessing();
  
  s_server.fini();
  
  ros::shutdown();
  return 0;
}
