//
//  pyride_media_bridge.h
//  PyRIDE
//
//  Created by Xun Wang on 11/06/19.
//  Copyright (c) 2019 Xun Wang. All rights reserved.
//

#ifndef PYRIDE_MEDIA_BRIDGE_H_
#define PYRIDE_MEDIA_BRIDGE_H_

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "PyConnectWrapper.h"
#include "PyConnectNetComm.h"
#include "ImageDataReceiver.h"
#include "AudioDataReceiver.h"

using namespace std;
using namespace ros;
using namespace pyconnect;

namespace pyride {

#define PYCONNECT_MODULE_NAME PyRIDEMediaBridge

class PyRIDEMediaBridge : public OObject
{
public:
  PyRIDEMediaBridge();
  virtual ~PyRIDEMediaBridge();

  void init();
  void fini();

  void continueProcessing();

private:
  NodeHandle priNode_;

  image_transport::ImageTransport imgTrans_;
  image_transport::Publisher imgPub_;

  Publisher audioPub_;

  boost::thread * image_grab_thread_;
  boost::thread * audio_grab_thread_;

  boost::thread * pyconnect_thread_;

  pyride_remote::ImageDataReceiver * imageReceiver_;
  pyride_remote::AudioDataReceiver * audioReceiver_;

  bool isRunning_;

  int VideoPort;
  bool IsImageStreaming;
  int AudioPort;
  bool IsAudioStreaming;

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

  void processPyConnectMessage();
public:
  PYCONNECT_NETCOMM_DECLARE;
  PYCONNECT_WRAPPER_DECLARE;

  PYCONNECT_MODULE_DESCRIPTION( "A ROS media bridge to connect (Non-ROS) PyRIDE with ROS ecosystem." );
  PYCONNECT_RO_ATTRIBUTE( VideoPort, "UDP port for receiving image stream." );
  PYCONNECT_RO_ATTRIBUTE( IsImageStreaming, "image streaming flag." );
  PYCONNECT_RO_ATTRIBUTE( AudioPort, "UDP port for receiving audio data stream." );
  PYCONNECT_RO_ATTRIBUTE( IsAudioStreaming, "audio data streaming flag." );
};

} // namespace pyride

#endif /* PYRIDE_MEDIA_BRIDGE_H_ */
