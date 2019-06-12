//
//  pyride_msg_bridge.cpp
//  PyRIDE
//
//  Created by Xun Wang on 11/06/19.
//  Copyright (c) 2019 Xun Wang. All rights reserved.
//

#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <audio_common_msgs/AudioData.h>

#include "pyride_media_bridge.h"

namespace pyride {

using namespace pyride_remote;

static const int kVideoStreamPort = 43096;
static const int kAudioStreamPort = 42096;
static const int kImageWidth = 640;
static const int kImageHeight = 480;
static const int kAudioSampleRate = 48000;
static const int kAudioFrameSize = 256;
static const int kAudioPacketBytes = 46;

PyRIDEMediaBridge::PyRIDEMediaBridge() :
  VideoPort( kVideoStreamPort ),
  IsImageStreaming( false ),
  AudioPort( kAudioStreamPort ),
  IsAudioStreaming( false ),
  imgTrans_( priNode_ ),
  imgRequests_( 0 ),
  audRequests_( 0 ),
  imgcnt_( 0L ),
  isRunning_( false )
{
  PYCONNECT_DECLARE_MODULE( PyRIDEMediaBridge, "A ROS media bridge to connect (Non-ROS) PyRIDE with ROS ecosystem." );
  PYCONNECT_RO_ATTRIBUTE_DECLARE( VideoPort, int, "UDP port for receiving image stream." );
  PYCONNECT_RO_ATTRIBUTE_DECLARE( IsImageStreaming, bool, "image streaming flag." );
  PYCONNECT_RO_ATTRIBUTE_DECLARE( AudioPort, int, "UDP port for receiving audio data stream." );
  PYCONNECT_RO_ATTRIBUTE_DECLARE( IsAudioStreaming, bool, "audio data streaming flag." );
}

PyRIDEMediaBridge::~PyRIDEMediaBridge()
{

}

void PyRIDEMediaBridge::init()
{
  priNode_.param( "image_data_port", VideoPort, kVideoStreamPort );
  priNode_.param( "audio_data_port", AudioPort, kAudioStreamPort );
  priNode_.param( "image_data_width", imageWidth_, kImageWidth );
  priNode_.param( "image_data_height", imageHeight_, kImageHeight );
  priNode_.param( "audio_sample_rate", sampleRate_, kAudioSampleRate );

  imgPub_ = imgTrans_.advertise( "/pyride/image", 1,
      boost::bind( &PyRIDEMediaBridge::startImageStream, this ),
      boost::bind( &PyRIDEMediaBridge::stopImageStream, this) );
  audioPub_ = priNode_.advertise<audio_common_msgs::AudioData>( "pyride/audio", 1,
      boost::bind( &PyRIDEMediaBridge::startAudioStream, this ),
      boost::bind( &PyRIDEMediaBridge::stopAudioStream, this) );
  ROS_INFO( "Video and Audio streaming capabilities are enabled." );

  PYCONNECT_NETCOMM_INIT;
  PYCONNECT_NETCOMM_ENABLE_NET;
  PYCONNECT_MODULE_INIT;

  pyconnect_thread_ = new boost::thread( &PyRIDEMediaBridge::processPyConnectMessage, this );

  isRunning_ = true;
}

void PyRIDEMediaBridge::fini()
{
  isRunning_ = false; // not really necessary
  imgRequests_ = 1; // reset requests
  this->stopImageStream();
  audRequests_ = 1;
  this->stopAudioStream();

  PYCONNECT_MODULE_FINI;
  PYCONNECT_NETCOMM_FINI;
  //pyconnect_thread_->join();
  delete pyconnect_thread_;
  pyconnect_thread_ = NULL;
}

void PyRIDEMediaBridge::startImageStream()
{
  if (!isRunning_) {
    ROS_ERROR( "PyRIDE message bridge is not running!\n" );
    return;
  }
  imgRequests_ ++;
  if (imgRequests_ > 1)
    return;

  IsImageStreaming = true;

  PYCONNECT_ATTRIBUTE_UPDATE( IsImageStreaming );

  imageReceiver_ = new ImageDataReceiver( VideoPort, imageWidth_, imageHeight_ );
  image_grab_thread_ = new boost::thread( &PyRIDEMediaBridge::doImageGrabbing, this );
  ROS_INFO( "Start image streaming service." );
}

void PyRIDEMediaBridge::doImageGrabbing()
{
  while (IsImageStreaming) {
    ImageDataPtr data = imageReceiver_->grabVideoStreamData();
    if (data) {
      try {
        std_msgs::Header header; // empty header
        header.seq = imgcnt_++; // user defined counter
        header.stamp = ros::Time::now();
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage( header, sensor_msgs::image_encodings::BGR8, *data ) );
        imgPub_.publish( cv_ptr->toImageMsg() );
      }
      catch (cv_bridge::Exception & e) {
        ROS_ERROR( "Unable to convert image message to mat." );
        continue;
      }
    }
    else {
      usleep( 1000 );
    }
  }
}

void PyRIDEMediaBridge::stopImageStream()
{
  imgRequests_--;
  if (imgRequests_ > 0 || !IsImageStreaming)
    return;

  IsImageStreaming = false;

  PYCONNECT_ATTRIBUTE_UPDATE( IsImageStreaming );

  if (image_grab_thread_) {
    image_grab_thread_->join();
    delete image_grab_thread_;
    image_grab_thread_ = NULL;
  }
  delete imageReceiver_;
  imageReceiver_ = NULL;
  ROS_INFO( "Stop image streaming service." );
}

void PyRIDEMediaBridge::startAudioStream()
{
  if (!isRunning_) {
    ROS_ERROR( "PyRIDE message bridge is not running!\n" );
    return;
  }
  audRequests_ ++;
  if (audRequests_ > 1)
    return;

  IsAudioStreaming = true;

  PYCONNECT_ATTRIBUTE_UPDATE( IsAudioStreaming );

  audioReceiver_ = new AudioDataReceiver( AudioPort, sampleRate_, kAudioFrameSize, kAudioPacketBytes );
  audio_grab_thread_ = new boost::thread( &PyRIDEMediaBridge::doAudioGrabbing, this );
  ROS_INFO( "Start audio data streaming service." );
}

void PyRIDEMediaBridge::doAudioGrabbing()
{
  unsigned char * audioData = new unsigned char[128 * kAudioFrameSize * sizeof(short)];

  while (IsAudioStreaming) {
    int datasize = audioReceiver_->grabAudioStreamData( (short *)audioData );
    if (datasize == 0) {
      continue;
    }
    audio_common_msgs::AudioData msg;

    // double ts = double(now.tv_sec) + (double(now.tv_usec) / 1000000.0);
    msg.data.resize( datasize );
    memcpy( &msg.data[0], audioData, datasize );

    audioPub_.publish( msg );
  }
  delete [] audioData;

}

void PyRIDEMediaBridge::stopAudioStream()
{
  audRequests_--;
  if (audRequests_ > 0 || !IsAudioStreaming)
    return;

  IsAudioStreaming = false;

  PYCONNECT_ATTRIBUTE_UPDATE( IsAudioStreaming );

  if (audio_grab_thread_) {
    audio_grab_thread_->join();
    delete audio_grab_thread_;
    audio_grab_thread_ = NULL;
  }
  delete audioReceiver_;
  audioReceiver_ = NULL;
  ROS_INFO( "Stop audio data streaming service." );
}

void PyRIDEMediaBridge::processPyConnectMessage()
{
  PYCONNECT_NETCOMM_PROCESS_DATA;
}

void PyRIDEMediaBridge::continueProcessing()
{
  ros::spin();
}

} // namespace pyride
