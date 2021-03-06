﻿// -*- C++ -*-
/*!
 * @file  pathCommander_arm.h
 * @brief ${rtcParam.description}
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PATHCOMMANDER_ARM_H
#define PATHCOMMANDER_ARM_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "MobileRobotStub.h"
#include "ExtendedDataTypesStub.h"
#include "BasicDataTypeStub.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <string>
#include <vector>

using namespace RTC;

/*!
 * @class pathCommander_arm
 * @brief ${rtcParam.description}
 *
 */
class pathCommander_arm
  : public RTC::DataFlowComponentBase {
public:
 /*!
  * @brief constructor
  * @param manager Maneger Object
  */
  pathCommander_arm(RTC::Manager* manager);
  /*!
   * @brief destructor
   */
  ~pathCommander_arm();

  // <rtc-template block="public_attribute">

  // </rtc-template>

  // <rtc-template block="public_operation">

  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry()
   *
   * @return RTC::ReturnCode_t
   *
   *
   */
  virtual RTC::ReturnCode_t onInitialize();

 /***
  *
  * The finalize action (on ALIVE->END transition)
  * formaer rtc_exiting_entry()
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onFinalize();

 /***
  *
  * The startup action when ExecutionContext startup
  * former rtc_starting_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

 /***
  *
  * The shutdown action when ExecutionContext stop
  * former rtc_stopping_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

 /***
  *
  * The activated action (Active state entry action)
  * former rtc_active_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

 /***
  *
  * The deactivated action (Active state exit action)
  * former rtc_active_exit()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

 /***
  *
  * The execution action that is invoked periodically
  * former rtc_active_do()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 /***
  *
  * The aborting action when main logic error occurred.
  * former rtc_aborting_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

 /***
  *
  * The error action in ERROR state
  * former rtc_error_do()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

 /***
  *
  * The reset action that is invoked resetting
  * This is same but different the former rtc_init_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
  virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

 /***
  *
  * The state update action that is invoked after onExecute() action
  * no corresponding operation exists in OpenRTm-aist-0.2.0
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

 /***
  *
  * The action that is invoked when execution context's rate is changed
  * no corresponding operation exists in OpenRTm-aist-0.2.0
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


protected:
 // <rtc-template block="protected_attribute">

 // </rtc-template>

 // <rtc-template block="protected_operation">

 // </rtc-template>

 // Configuration variable declaration
 // <rtc-template block="config_declare">
 /*!
  *
  * - Name:  filename
  * - DefaultValue: path.txt
  */
  std::string m_filename;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedPose2D m_currentPose;
  /*!
   */
  InPort<RTC::TimedPose2D> m_currentPoseIn;
  RTC::TimedString m_commandString;
  /*!
   */
  InPort<RTC::TimedString> m_commandStringIn;
  RTC::TimedString m_cameraxy;
  /*!
   */
  InPort<RTC::TimedString> m_cameraxyIn;

  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedString m_statusString;
  /*!
   */
  OutPort<RTC::TimedString> m_statusStringOut;
  RTC::TimedShort m_localizationMode;
  /*!
   */
  OutPort<RTC::TimedShort> m_localizationModeOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_pathFollowerPort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  /*!
   */
  RTC::CorbaConsumer<RTC::PathFollower> m_FollowerMotion;

  // </rtc-template>

private:
 // <rtc-template block="private_attribute">


  std::vector <double> ave_x;
  std::vector <double> ave_y;

 // </rtc-template>

 // <rtc-template block="private_operation">

 // </rtc-template>

  RTC::Waypoint2D m_waypointDefault;
  RTC::FOLLOWER_STATE m_statePrev;
  std::vector<RTC::Path2D> m_paths;
  int readPath();
  int InPath();
  int Number = 4;
  struct Point2d {
    double x;
    double y;
    double r;
    Point2d() { x = 0; y = 0; r = 0; }
    Point2d(double x_, double y_, double r_) { x = x_; y = y_; r = r_; }
  };
  void setPath(RTC::Path2D &path, const std::vector<Point2d> &vp);
  bool m_rtcError;  //OpenRTM-aist-1.2.0のバグ回避
};


extern "C"
{
  DLL_EXPORT void pathCommander_armInit(RTC::Manager* manager);
};

#endif // PATHCOMMANDER_ARM_H
