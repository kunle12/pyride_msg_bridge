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
#ifndef NO_MEDIA
#include <image_transport/image_transport.h>
#endif
#include <pyride_common_msgs/NodeStatus.h>

#include "PyConnectWrapper.h"
#include "PyConnectNetComm.h"
#ifndef NO_MEDIA
#include "ImageDataReceiver.h"
#include "AudioDataReceiver.h"
#endif

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

  void sendNodeMessage( const std::string & node, const std::string & command );

  void continueProcessing();

private:
  NodeHandle priNode_;

  Publisher nodePub_;
  Subscriber nodeSub_;

  std::string NodeStatus;

#ifndef NO_MEDIA
  int VideoPort;
  bool IsImageStreaming;
  int AudioPort;
  bool IsAudioStreaming;

  image_transport::ImageTransport imgTrans_;
  image_transport::Publisher imgPub_;

  Publisher audioPub_;

  boost::thread * image_grab_thread_;
  boost::thread * audio_grab_thread_;
#endif
  boost::thread * pyconnect_thread_;

#ifndef NO_MEDIA
  pyride_remote::ImageDataReceiver * imageReceiver_;
  pyride_remote::AudioDataReceiver * audioReceiver_;
#endif

  bool isRunning_;

#ifndef NO_MEDIA
  int imageWidth_;
  int imageHeight_;
  int sampleRate_;
  long imgcnt_;

  int imgRequests_;
  int audRequests_;

  void startImageStream();
  void stopImageStream();

  void startAudioStream();
  void stopAudioStream();

  void doImageGrabbing();
  void doAudioGrabbing();
#endif

  void processPyConnectMessage();

  void nodeStatusCB( const pyride_common_msgs::NodeStatusConstPtr & msg );

public:
  PYCONNECT_NETCOMM_DECLARE;
  PYCONNECT_WRAPPER_DECLARE;

  PYCONNECT_MODULE_DESCRIPTION( "A ROS bridge to connect (Non-ROS) PyRIDE with ROS ecosystem." );
  PYCONNECT_METHOD( sendNodeMessage, "send message to a ROS node through pyride_common_msgs" );

  PYCONNECT_RO_ATTRIBUTE( NodeStatus, "Node status update message" );
#ifndef NO_MEDIA
  PYCONNECT_RO_ATTRIBUTE( VideoPort, "UDP port for receiving image stream." );
  PYCONNECT_RO_ATTRIBUTE( IsImageStreaming, "image streaming flag." );
  PYCONNECT_RO_ATTRIBUTE( AudioPort, "UDP port for receiving audio data stream." );
  PYCONNECT_RO_ATTRIBUTE( IsAudioStreaming, "audio data streaming flag." );
#endif
};

} // namespace pyride

#endif /* PYRIDE_MSG_BRIDGE_H_ */
