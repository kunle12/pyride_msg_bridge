//
//  pyride_msg_bridge.cpp
//  PyRIDE
//
//  Created by Xun Wang on 18/04/17.
//  Copyright (c) 2017 Xun Wang. All rights reserved.
//

#include <pyride_common_msgs/RenameObject.h>
#include <pyride_common_msgs/NodeMessage.h>
#include "pyride_msg_bridge.h"

namespace pyride {

PyRIDEMsgBridge::PyRIDEMsgBridge() :
  isRunning_( 0 ),
  faceEnrolmentClient_( NULL ),
  EnrolmentStatus( false )
{
  EXPORT_PYCONNECT_MODULE;
  EXPORT_PYCONNECT_RO_ATTRIBUTE( NodeStatus );
  EXPORT_PYCONNECT_RO_ATTRIBUTE( EnrolmentStatus );
  EXPORT_PYCONNECT_METHOD( sendMessageToNode );
  EXPORT_PYCONNECT_METHOD( sendMessageToNodeWithPriority );
  EXPORT_PYCONNECT_METHOD( enrolFace );
  EXPORT_PYCONNECT_METHOD( renameFace );

  FD_ZERO( &masterFDSet_ );
}

PyRIDEMsgBridge::~PyRIDEMsgBridge()
{

}

void PyRIDEMsgBridge::init()
{
  PYCONNECT_NETCOMM_INIT;
  PYCONNECT_NETCOMM_ENABLE_NET;
  PYCONNECT_MODULE_INIT;

  nodePub_ = priNode_.advertise<pyride_common_msgs::NodeMessage>( "node_message", 1 );
  nodeSub_ = priNode_.subscribe( "node_status", 10, &PyRIDEMsgBridge::nodeStatusCB, this );

  faceEnrolmentClient_ = new ObjectEnrolmentClient( "/face_server/face_enrolment", true );
  renameFaceClient_ = priNode_.serviceClient<pyride_common_msgs::RenameObject>( "/face_server/rename_face" );
  isRunning_ = true;
}

void PyRIDEMsgBridge::fini()
{
  isRunning_ = false; // not really necessary

  if (faceEnrolmentClient_) {
    delete faceEnrolmentClient_;
    faceEnrolmentClient_ = NULL;
  }

  nodeSub_.shutdown();
  PYCONNECT_MODULE_FINI;
  PYCONNECT_NETCOMM_FINI;
}

/*! \typedef onNodeStatus( data )
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

void PyRIDEMsgBridge::sendMessageToNode( const std::string & node, const std::string & command )
{
  this->sendMessageToNodeWithPriority( node, command, 0 );
}

void PyRIDEMsgBridge::sendMessageToNodeWithPriority( const std::string & node, const std::string & command, const int priority )
{
  pyride_common_msgs::NodeMessage msg;
  msg.header.stamp = ros::Time::now();
  msg.node_id = node;
  msg.priority = priority;
  msg.command = command;

  nodePub_.publish( msg );
}

bool PyRIDEMsgBridge::enrolFace( const std::string & face_name, const int required_samples )
{
  if (face_name.length() == 0 || required_samples <= 0) // really just check for negative sample.
    return false;

  if (!faceEnrolmentClient_->isServerConnected()) {
    ROS_INFO( "face recognition server is not connected, trying to connect..." );
    int trials = 0;
    while (!faceEnrolmentClient_->waitForServer( ros::Duration( 1.0 ) ) && trials < 2) {
      ROS_INFO( "Waiting for the face recognition server server to come up." );
      trials++;
    }
    if (!faceEnrolmentClient_->isServerConnected()) {
      ROS_ERROR( "face recognition server is down." );
      return false;
    }
  }

  pyride_common_msgs::ObjectEnrolmentGoal goal;

  goal.name = face_name;
  goal.instances = required_samples;
  goal.timeout = required_samples * 1.5;

  faceEnrolmentClient_->sendGoal( goal,
                      boost::bind( &PyRIDEMsgBridge::doneFaceEnrolmentAction, this, _1, _2 ),
                      ObjectEnrolmentClient::SimpleActiveCallback(),
                      ObjectEnrolmentClient::SimpleFeedbackCallback() );
  return true;
}

void PyRIDEMsgBridge::doneFaceEnrolmentAction( const actionlib::SimpleClientGoalState & state,
                        const pyride_common_msgs::ObjectEnrolmentResultConstPtr & result )
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    EnrolmentStatus = true;
  }
  else {
    EnrolmentStatus = false;
  }

  PYCONNECT_ATTRIBUTE_UPDATE( EnrolmentStatus );
  // internally reset to false anyway
  EnrolmentStatus = false;

  ROS_INFO( "On face enrolment finished in state [%s]", state.toString().c_str());
}

bool PyRIDEMsgBridge::renameFace( const std::string & old_name, const std::string & new_name )
{
  if (!renameFaceClient_.exists()) {
    return false;
  }
  pyride_common_msgs::RenameObject srvMsg;

  srvMsg.request.old_name = old_name;
  srvMsg.request.new_name = new_name;
  if (renameFaceClient_.call( srvMsg )) {
    return srvMsg.response.success;
  }
  return false;
}

void PyRIDEMsgBridge::stopProcess()
{
  isRunning_ = 0;
}

void PyRIDEMsgBridge::continueProcessing()
{
  fd_set readyFDSet;
  isRunning_ = 1;

  while (isRunning_) {
    struct timeval timeout, timeStamp;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms

    FD_ZERO( &readyFDSet );
    memcpy( &readyFDSet, &masterFDSet_, sizeof( masterFDSet_ ) );
    int maxFD = maxFD_;
    select( maxFD+1, &readyFDSet, NULL, NULL, &timeout ); // non-blocking select

    if (isRunning_) {
      PYCONNECT_NETCOMM_PROCESS_DATA( &readyFDSet );
    }
    ros::spinOnce();
  }
}

void PyRIDEMsgBridge::setFD( const SOCKET_T & fd )
{
  FD_SET( fd, &masterFDSet_ );
  maxFD_ = max( fd, maxFD_ );
}

void PyRIDEMsgBridge::clearFD( const SOCKET_T & fd )
{
  FD_CLR( fd, &masterFDSet_ );
  maxFD_ = max( fd, maxFD_ );
}

} // namespace pyride
