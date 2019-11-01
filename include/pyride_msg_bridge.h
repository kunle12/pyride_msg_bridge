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
#include <signal.h>

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <pyride_common_msgs/NodeStatus.h>
#include <pyride_common_msgs/ObjectEnrolmentAction.h>

#include "PyConnectWrapper.h"
#include "PyConnectNetComm.h"

using namespace std;
using namespace ros;
using namespace pyconnect;

namespace pyride {

#define PYCONNECT_MODULE_NAME PyRIDEMsgBridge

typedef actionlib::SimpleActionClient<pyride_common_msgs::ObjectEnrolmentAction> ObjectEnrolmentClient;

class PyRIDEMsgBridge : public OObject, FDSetOwner
{
public:
  PyRIDEMsgBridge();
  virtual ~PyRIDEMsgBridge();

  void init();
  void fini();

  void stopProcess();

  void sendMessageToNode( const std::string & node, const std::string & command );
  void sendMessageToNodeWithPriority( const std::string & node, const std::string & command, const int priority );
  bool enrolHumanFace( const std::string & face_name, const int required_samples );
  bool renameFace( const std::string & old_name, const std::string & new_name );

  void continueProcessing();

private:
  NodeHandle priNode_;

  Publisher nodePub_;
  Subscriber nodeSub_;

  ObjectEnrolmentClient * faceEnrolmentClient_;
  ServiceClient renameFaceClient_;

  std::string NodeStatus;
  bool EnrolmentStatus;

  volatile sig_atomic_t isRunning_;
  int maxFD_;
  fd_set masterFDSet_;

  void nodeStatusCB( const pyride_common_msgs::NodeStatusConstPtr & msg );
  void doneFaceEnrolmentAction( const actionlib::SimpleClientGoalState & state,
                                const pyride_common_msgs::ObjectEnrolmentResultConstPtr & result );


public:
  PYCONNECT_NETCOMM_DECLARE;
  PYCONNECT_WRAPPER_DECLARE;

  PYCONNECT_MODULE_DESCRIPTION( "A ROS bridge to connect (Non-ROS) PyRIDE with ROS ecosystem." );
  PYCONNECT_METHOD( sendMessageToNode, "send message to a ROS node through pyride_common_msgs" );
  PYCONNECT_METHOD( sendMessageToNodeWithPriority, "send message with priority to a ROS node through pyride_common_msgs" );
  PYCONNECT_METHOD( enrolHumanFace, "register a human face with an associated name." );
  PYCONNECT_METHOD( renameFace, "update human face with an new name." );

  PYCONNECT_RO_ATTRIBUTE( NodeStatus, "Node status update message" );
  PYCONNECT_RO_ATTRIBUTE( EnrolmentStatus, "Face enrolment status update message" );
};

} // namespace pyride

#endif /* PYRIDE_MSG_BRIDGE_H_ */
