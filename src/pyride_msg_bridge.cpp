//
//  pyride_msg_bridge.cpp
//  PyRIDE
//
//  Created by Xun Wang on 18/04/17.
//  Copyright (c) 2017 Xun Wang. All rights reserved.
//

#ifndef NO_MEDIA
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <audio_common_msgs/AudioData.h>
#endif

#include <pyride_common_msgs/NodeMessage.h>
#include "pyride_msg_bridge.h"

namespace pyride {

#ifndef NO_MEDIA
using namespace pyride_remote;

static const int kVideoStreamPort = 43096;
static const int kAudioStreamPort = 42096;
static const int kImageWidth = 640;
static const int kImageHeight = 480;
static const int kAudioSampleRate = 48000;
static const int kAudioFrameSize = 256;
static const int kAudioPacketBytes = 46;
#endif

PyRIDEMsgBridge::PyRIDEMsgBridge() :
#ifndef NO_MEDIA
  VideoPort( kVideoStreamPort ),
  IsImageStreaming( false ),
  AudioPort( kAudioStreamPort ),
  IsAudioStreaming( false ),
  imgTrans_( priNode_ ),
  imgRequests_( 0 ),
  audRequests_( 0 ),
  imgcnt_( 0L ),
#endif
  isRunning_( false )
{
  EXPORT_PYCONNECT_MODULE;
  EXPORT_PYCONNECT_RO_ATTRIBUTE( NodeStatus );
#ifndef NO_MEDIA
  EXPORT_PYCONNECT_RO_ATTRIBUTE( VideoPort );
  EXPORT_PYCONNECT_RO_ATTRIBUTE( IsImageStreaming );
  EXPORT_PYCONNECT_RO_ATTRIBUTE( AudioPort );
  EXPORT_PYCONNECT_RO_ATTRIBUTE( IsAudioStreaming );
#endif

  EXPORT_PYCONNECT_METHOD( sendNodeMessage );
}

PyRIDEMsgBridge::~PyRIDEMsgBridge()
{

}

void PyRIDEMsgBridge::init()
{
#ifndef NO_MEDIA
  priNode_.param( "image_data_port", VideoPort, kVideoStreamPort );
  priNode_.param( "audio_data_port", AudioPort, kAudioStreamPort );
  priNode_.param( "image_data_width", imageWidth_, kImageWidth );
  priNode_.param( "image_data_height", imageHeight_, kImageHeight );
  priNode_.param( "audio_sample_rate", sampleRate_, kAudioSampleRate );

  imgPub_ = imgTrans_.advertise( "/pyride/image", 1,
      boost::bind( &PyRIDEMsgBridge::startImageStream, this ),
      boost::bind( &PyRIDEMsgBridge::stopImageStream, this) );
  audioPub_ = priNode_.advertise<audio_common_msgs::AudioData>( "pyride/audio", 1,
      boost::bind( &PyRIDEMsgBridge::startAudioStream, this ),
      boost::bind( &PyRIDEMsgBridge::stopAudioStream, this) );
  ROS_INFO( "Video and Audio streaming capabilities are enabled." );
#endif

  nodePub_ = priNode_.advertise<pyride_common_msgs::NodeMessage>( "pyride/node_message", 5 );
  nodeSub_ = priNode_.subscribe( "pyride/node_status", 10, &PyRIDEMsgBridge::nodeStatusCB, this );

  PYCONNECT_NETCOMM_INIT;
  PYCONNECT_NETCOMM_ENABLE_NET;
  PYCONNECT_MODULE_INIT;

  pyconnect_thread_ = new boost::thread( &PyRIDEMsgBridge::processPyConnectMessage, this );

  isRunning_ = true;
}

void PyRIDEMsgBridge::fini()
{
  isRunning_ = false; // not really necessary
#ifndef NO_MEDIA
  imgRequests_ = 1; // reset requests
  this->stopImageStream();
  audRequests_ = 1;
  this->stopAudioStream();
#endif

  nodeSub_.shutdown();
  PYCONNECT_MODULE_FINI;
  PYCONNECT_NETCOMM_FINI;
  //pyconnect_thread_->join();
  delete pyconnect_thread_;
  pyconnect_thread_ = NULL;
}

/*! \typedef onNodeStatusUpdate( data )
 *  \memberof PyRIDEMsgBridge.
 *  \brief Callback function invoked when an external ROS node dispatch status update message
 *   to PyRIDE on pyride_reem/node_status topic.
 *  \param dictionary data. node status message in the format of {'node', 'timestamp', 'priority', 'message' }.
 *  \return None.
 */
void PyRIDEMsgBridge::nodeStatusCB( const pyride_common_msgs::NodeStatusConstPtr & msg )
{
  stringstream ss;

  // put in a json string format
  ss << "{\"node\": \"" << msg->node_id << "\", \"timestamp\": " << ((double)msg->header.stamp.sec + (double)msg->header.stamp.nsec / 1E9);
  if (msg->status_text.length() > 2 && (msg->status_text[0] =='{' || msg->status_text[0] =='[') ) {
    ss << ", \"message\": " << msg->status_text << ", \"priority\": " << (int)msg->priority << "}";
  }
  else {
    ss << ", \"message\": \"" << msg->status_text << "\", \"priority\": " << (int)msg->priority << "}";
  }

  NodeStatus = ss.str();
  PYCONNECT_ATTRIBUTE_UPDATE( NodeStatus );
}

void PyRIDEMsgBridge::sendNodeMessage( const std::string & node, const std::string & command )
{
  pyride_common_msgs::NodeMessage msg;
  msg.header.stamp = ros::Time::now();
  msg.node_id = node;
  msg.priority = 2;
  msg.command = command;

  nodePub_.publish( msg );
}

#ifndef NO_MEDIA
void PyRIDEMsgBridge::startImageStream()
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
  image_grab_thread_ = new boost::thread( &PyRIDEMsgBridge::doImageGrabbing, this );
  ROS_INFO( "Start image streaming service." );
}

void PyRIDEMsgBridge::doImageGrabbing()
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

void PyRIDEMsgBridge::stopImageStream()
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

void PyRIDEMsgBridge::startAudioStream()
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
  audio_grab_thread_ = new boost::thread( &PyRIDEMsgBridge::doAudioGrabbing, this );
  ROS_INFO( "Start audio data streaming service." );
}

void PyRIDEMsgBridge::doAudioGrabbing()
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

void PyRIDEMsgBridge::stopAudioStream()
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
#endif

void PyRIDEMsgBridge::processPyConnectMessage()
{
  PYCONNECT_NETCOMM_PROCESS_DATA;
}

void PyRIDEMsgBridge::continueProcessing()
{
  ros::spin();
}

} // namespace pyride
