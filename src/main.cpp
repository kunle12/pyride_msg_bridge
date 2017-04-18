//
//  pyride_msg_bridge.cpp
//  PyRIDE
//
//  Created by Xun Wang on 18/04/17.
//  Copyright (c) 2017 Xun Wang. All rights reserved.
//

#include <ros/ros.h>
#include <signal.h>

#include "pyride_msg_bridge.h"

using namespace pyride;
static PyRIDEMsgBridge * s_server = NULL;

void stopProcess( int sig )
{
  if (s_server)
    s_server->stopProcess();
}

int main( int argc, char * argv[] )
{
  ros::init( argc, argv, "pyride_msg_bridge" );

  s_server = new PyRIDEMsgBridge();

  signal( SIGINT, ::stopProcess );
  
  s_server->init();

  s_server->continueProcessing();
  
  s_server->fini();
  
  delete s_server;

  ros::shutdown();
  return 0;
}
