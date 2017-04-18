//
//  pyride_msg_bridge.h
//  PyRIDE
//
//  Created by Xun Wang on 18/04/17.
//  Copyright (c) 2017 Xun Wang. All rights reserved.
//

#ifndef PYRIDE_MSG_BRIDGE_H_
#define PYRIDE_MSG_BRIDGE_H_

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <pyride_common_msgs/NodeStatus.h>

#include "PyConnectWrapper.h"
#include "PyConnectNetComm.h"

using namespace std;
using namespace ros;
using namespace pyconnect;

namespace pyride {

#define PYCONNECT_MODULE_NAME PyRIDEMsgBridge

class PyRIDEMsgBridge : public OObject, FDSetOwner
{
public:
  PyRIDEMsgBridge();
  virtual ~PyRIDEMsgBridge();

  void init();
  void fini();

  void stopProcess();

  void sendNodeMessage( const std::string & node, const std::string & command, const int priority );

  void continueProcessing();

  PYCONNECT_NETCOMM_DECLARE;
  PYCONNECT_WRAPPER_DECLARE;

  PYCONNECT_METHOD_ACCESS_VOID_RETURN( sendNodeMessage, ARGTYPE( string ), ARGTYPE( string ), ARGTYPE( int ) );

  std::string NodeStatusUpdate;

  PYCONNECT_RO_ATTRIBUTE( NodeStatusUpdate );

private:
  NodeHandle priNode_;

  Publisher nodePub_;
  Subscriber nodeSub_;

  bool isRunning_;
  int maxFD_;
  fd_set masterFDSet_;

  void nodeStatusCB( const pyride_common_msgs::NodeStatusConstPtr & msg );
};

} // namespace pyride

#endif /* PYRIDE_MSG_BRIDGE_H_ */
