// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.cpp
 * @brief Service implementation code of MobileRobot.idl
 *
 */

#include "MobileRobotSVC_impl.h"

/*
 * Example implementational code for IDL interface RTC::OGMapper
 */
RTC_OGMapperSVC_impl::RTC_OGMapperSVC_impl()
{
  // Please add extra constructor code here.
}


RTC_OGMapperSVC_impl::~RTC_OGMapperSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE RTC_OGMapperSVC_impl::initializeMap(const RTC::OGMapConfig& config, const RTC::Pose2D& initialPose)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_OGMapperSVC_impl::initializeMap(const RTC::OGMapConfig& config, const RTC::Pose2D& initialPose)>"
#endif
  return 0;
}

RTC::RETURN_VALUE RTC_OGMapperSVC_impl::startMapping()
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_OGMapperSVC_impl::startMapping()>"
#endif
  return 0;
}

RTC::RETURN_VALUE RTC_OGMapperSVC_impl::stopMapping()
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_OGMapperSVC_impl::stopMapping()>"
#endif
  return 0;
}

RTC::RETURN_VALUE RTC_OGMapperSVC_impl::suspendMapping()
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_OGMapperSVC_impl::suspendMapping()>"
#endif
  return 0;
}

RTC::RETURN_VALUE RTC_OGMapperSVC_impl::resumeMapping()
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_OGMapperSVC_impl::resumeMapping()>"
#endif
  return 0;
}

RTC::RETURN_VALUE RTC_OGMapperSVC_impl::getState(RTC::MAPPER_STATE& state)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_OGMapperSVC_impl::getState(RTC::MAPPER_STATE& state)>"
#endif
  return 0;
}

RTC::RETURN_VALUE RTC_OGMapperSVC_impl::requestCurrentBuiltMap(RTC::OGMap_out map)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_OGMapperSVC_impl::requestCurrentBuiltMap(RTC::OGMap_out map)>"
#endif
  return 0;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface RTC::OGMapServer
 */
RTC_OGMapServerSVC_impl::RTC_OGMapServerSVC_impl()
{
  // Please add extra constructor code here.
}


RTC_OGMapServerSVC_impl::~RTC_OGMapServerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE RTC_OGMapServerSVC_impl::requestCurrentBuiltMap(RTC::OGMap_out map)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_OGMapServerSVC_impl::requestCurrentBuiltMap(RTC::OGMap_out map)>"
#endif
  return 0;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface RTC::PathPlanner
 */
RTC_PathPlannerSVC_impl::RTC_PathPlannerSVC_impl()
{
  // Please add extra constructor code here.
}


RTC_PathPlannerSVC_impl::~RTC_PathPlannerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE RTC_PathPlannerSVC_impl::planPath(const RTC::PathPlanParameter& param, RTC::Path2D_out outPath)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_PathPlannerSVC_impl::planPath(const RTC::PathPlanParameter& param, RTC::Path2D_out outPath)>"
#endif
  return 0;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface RTC::PathFollower
 */
RTC_PathFollowerSVC_impl::RTC_PathFollowerSVC_impl()
{
  // Please add extra constructor code here.
}


RTC_PathFollowerSVC_impl::~RTC_PathFollowerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::followPath(const RTC::Path2D& path)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::followPath(const RTC::Path2D& path)>"
#endif
  return 0;
}

RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::getState(RTC::FOLLOWER_STATE& state)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::getState(RTC::FOLLOWER_STATE& state)>"
#endif
  return 0;
}

RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::followPathNonBlock(const RTC::Path2D& path)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE RTC_PathFollowerSVC_impl::followPathNonBlock(const RTC::Path2D& path)>"
#endif
  return 0;
}



// End of example implementational code



