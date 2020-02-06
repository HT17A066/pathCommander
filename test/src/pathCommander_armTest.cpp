// -*- C++ -*-
/*!
 * @file  pathCommander_armTest.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#include "pathCommander_armTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* pathcommander_arm_spec[] =
  {
    "implementation_id", "pathCommander_armTest",
    "type_name",         "pathCommander_armTest",
    "description",       "${rtcParam.description}",
    "version",           "1.0.0",
    "vendor",            "MasutaniLab",
    "category",          "Mobile Robot",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.filename", "path.txt",

    // Widget
    "conf.__widget__.filename", "text",
    // Constraints

    "conf.__type__.filename", "string",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
pathCommander_armTest::pathCommander_armTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_currentPoseIn("currentPose", m_currentPose),
    m_commandStringIn("commandString", m_commandString),
    m_cameraxyIn("cameraxy", m_cameraxy),
    m_statusStringOut("statusString", m_statusString),
    m_localizationModeOut("localizationMode", m_localizationMode),
    m_pathFollowerPort("pathFollower")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
pathCommander_armTest::~pathCommander_armTest()
{
}



RTC::ReturnCode_t pathCommander_armTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("statusString", m_statusStringIn);
  addInPort("localizationMode", m_localizationModeIn);
  
  // Set OutPort buffer
  addOutPort("currentPose", m_currentPoseOut);
  addOutPort("commandString", m_commandStringOut);
  addOutPort("cameraxy", m_cameraxyOut);
  
  // Set service provider to Ports
  m_pathFollowerPort.registerProvider("PathFollower", "RTC::PathFollower", m_FollowerMotion);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_pathFollowerPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("filename", m_filename, "path.txt");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t pathCommander_armTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t pathCommander_armTest::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t pathCommander_armTest::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t pathCommander_armTest::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t pathCommander_armTest::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t pathCommander_armTest::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t pathCommander_armTest::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t pathCommander_armTest::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t pathCommander_armTest::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t pathCommander_armTest::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t pathCommander_armTest::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void pathCommander_armTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(pathcommander_arm_spec);
    manager->registerFactory(profile,
                             RTC::Create<pathCommander_armTest>,
                             RTC::Delete<pathCommander_armTest>);
  }
  
};


