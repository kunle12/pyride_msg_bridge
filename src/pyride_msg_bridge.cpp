//
//  pyride_msg_bridge.cpp
//  PyRIDE
//
//  Created by Xun Wang on 18/04/17.
//  Copyright (c) 2017 Xun Wang. All rights reserved.
//

#include <pyride_common_msgs/NodeMessage.h>
#include "pyride_msg_bridge.h"

namespace pyride {

PyRIDEMsgBridge::PyRIDEMsgBridge() :
  isRunning_( false )
{
  PYCONNECT_DECLARE_MODULE( PyRIDEMsgBridge, "A ROS bridge to connect (Non-ROS) PyRIDE with ROS ecosystem." );
  PYCONNECT_RO_ATTRIBUTE_DECLARE( NodeStatusUpdate, std::string, "Node status update message" );

  PYCONNECT_METHOD_DECLARE( sendNodeMessage, void, "send message to a ROS node through pyride_common_msgs", ARG( node, std::string, "node name" ) \
      ARG( command, std::string, "command message to the node" ) ARG( priority, int, "message priority" ) );

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
  isRunning_ = true;
}

void PyRIDEMsgBridge::fini()
{
  isRunning_ = false; // not really necessary
  nodeSub_.shutdown();
  PYCONNECT_MODULE_FINI;
  PYCONNECT_NETCOMM_FINI;
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
  ss << ", \"message\": \"" << msg->status_text << "\", \"priority\": " << msg->priority << "}";

  NodeStatusUpdate = ss.str();
  PYCONNECT_ATTRIBUTE_UPDATE( NodeStatusUpdate );
}

void PyRIDEMsgBridge::sendNodeMessage( const std::string & node, const std::string & command, const int priority )
{
  pyride_common_msgs::NodeMessage msg;
  msg.header.stamp = ros::Time::now();
  msg.node_id = node;
  msg.priority = priority;
  msg.command = command;

  nodePub_.publish( msg );
}

void PyRIDEMsgBridge::stopProcess()
{
  isRunning_ = false;
}

void PyRIDEMsgBridge::continueProcessing()
{
  fd_set readyFDSet;

  while (isRunning_) {
    ros::spinOnce();
    struct timeval timeout, timeStamp;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms

    FD_ZERO( &readyFDSet );
    memcpy( &readyFDSet, &masterFDSet_, sizeof( masterFDSet_ ) );
    int maxFD = maxFD_;
    select( maxFD+1, &readyFDSet, NULL, NULL, &timeout ); // non-blocking select

    PYCONNECT_NETCOMM_PROCESS_DATA( &readyFDSet );
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
