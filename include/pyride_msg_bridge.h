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

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <pyride_common_msgs/NodeStatus.h>

#include "PyConnectWrapper.h"
#include "PyConnectNetComm.h"
#include "ImageDataReceiver.h"

using namespace std;
using namespace ros;
using namespace pyconnect;

namespace pyride {

#define PYCONNECT_MODULE_NAME PyRIDEMsgBridge

class PyRIDEMsgBridge : public OObject
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
  int VideoPort;
  bool IsImageStreaming;

  PYCONNECT_RO_ATTRIBUTE( NodeStatusUpdate );
  PYCONNECT_RO_ATTRIBUTE( VideoPort );
  PYCONNECT_RO_ATTRIBUTE( IsImageStreaming );

private:
  NodeHandle priNode_;

  Publisher nodePub_;
  Subscriber nodeSub_;

  image_transport::ImageTransport imgTrans_;
  image_transport::Publisher imgPub_;

  boost::thread * image_grab_thread_;
  boost::thread * pyconnect_thread_;

  pyride_remote::ImageDataReceiver * imageReceiver_;

  int imageWidth_;
  int imageHeight_;
  long imgcnt_;

  bool isRunning_;
  int srvRequests_;

  void startImageStream();
  void stopImageStream();

  void doImageGrabbing();
  void processPyConnectMessage();

  void nodeStatusCB( const pyride_common_msgs::NodeStatusConstPtr & msg );
};

} // namespace pyride

#endif /* PYRIDE_MSG_BRIDGE_H_ */
