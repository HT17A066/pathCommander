// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.h
 * @brief Service implementation header of MobileRobot.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"
#include "InterfaceDataTypesSkel.h"

#include "MobileRobotSkel.h"

#ifndef MOBILEROBOTSVC_IMPL_H
#define MOBILEROBOTSVC_IMPL_H
 
/*!
 * @class RTC_OGMapperSVC_impl
 * Example class implementing IDL interface RTC::OGMapper
 */
class RTC_OGMapperSVC_impl
 : public virtual POA_RTC::OGMapper,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RTC_OGMapperSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RTC_OGMapperSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RTC_OGMapperSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE initializeMap(const RTC::OGMapConfig& config, const RTC::Pose2D& initialPose);
   RTC::RETURN_VALUE startMapping();
   RTC::RETURN_VALUE stopMapping();
   RTC::RETURN_VALUE suspendMapping();
   RTC::RETURN_VALUE resumeMapping();
   RTC::RETURN_VALUE getState(RTC::MAPPER_STATE& state);
   RTC::RETURN_VALUE requestCurrentBuiltMap(RTC::OGMap_out map);

};

/*!
 * @class RTC_OGMapServerSVC_impl
 * Example class implementing IDL interface RTC::OGMapServer
 */
class RTC_OGMapServerSVC_impl
 : public virtual POA_RTC::OGMapServer,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RTC_OGMapServerSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RTC_OGMapServerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RTC_OGMapServerSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE requestCurrentBuiltMap(RTC::OGMap_out map);

};

/*!
 * @class RTC_PathPlannerSVC_impl
 * Example class implementing IDL interface RTC::PathPlanner
 */
class RTC_PathPlannerSVC_impl
 : public virtual POA_RTC::PathPlanner,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RTC_PathPlannerSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RTC_PathPlannerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RTC_PathPlannerSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE planPath(const RTC::PathPlanParameter& param, RTC::Path2D_out outPath);

};

/*!
 * @class RTC_PathFollowerSVC_impl
 * Example class implementing IDL interface RTC::PathFollower
 */
class RTC_PathFollowerSVC_impl
 : public virtual POA_RTC::PathFollower,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RTC_PathFollowerSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RTC_PathFollowerSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RTC_PathFollowerSVC_impl();

   // attributes and operations
   RTC::RETURN_VALUE followPath(const RTC::Path2D& path);
   RTC::RETURN_VALUE getState(RTC::FOLLOWER_STATE& state);
   RTC::RETURN_VALUE followPathNonBlock(const RTC::Path2D& path);

};



#endif // MOBILEROBOTSVC_IMPL_H


