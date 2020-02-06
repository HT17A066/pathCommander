// -*- C++ -*-
/*!
 * @file  pathCommander_arm.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#define _USE_MATH_DEFINES
#include "pathCommander_arm.h"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
using namespace std;

#define ServicePortCheck(x) \
  (!CORBA::is_nil(x._ptr()) && !x->_non_existent())

// Module specification
// <rtc-template block="module_spec">
static const char* pathcommander_arm_spec[] =
{
  "implementation_id", "pathCommander_arm",
  "type_name",         "pathCommander_arm",
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
pathCommander_arm::pathCommander_arm(RTC::Manager* manager)
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
pathCommander_arm::~pathCommander_arm()
{
}



RTC::ReturnCode_t pathCommander_arm::onInitialize()
{
  RTC_INFO(("onInitialize()"));
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("currentPose", m_currentPoseIn);
  addInPort("commandString", m_commandStringIn);
  addInPort("cameraxy", m_cameraxyIn);

  // Set OutPort buffer
  addOutPort("statusString", m_statusStringOut);
  addOutPort("localizationMode", m_localizationModeOut);

  // Set service provider to Ports

  // Set service consumers to Ports
  m_pathFollowerPort.registerConsumer("PathFollower", "RTC::PathFollower", m_FollowerMotion);

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
RTC::ReturnCode_t pathCommander_arm::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t pathCommander_arm::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t pathCommander_arm::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t pathCommander_arm::onActivated(RTC::UniqueId ec_id)
{
  RTC_INFO(("onActivated()"));
  m_waypointDefault.distanceTolerance = 0.1;
  m_waypointDefault.headingTolerance = 0.1;
  m_waypointDefault.timeLimit.nsec = 0;
  m_waypointDefault.timeLimit.sec = 0;
  m_waypointDefault.maxSpeed.vx = 0.3;
  m_waypointDefault.maxSpeed.vy = 0;
  m_waypointDefault.maxSpeed.va = 1.0;

  m_rtcError = false;  //OpenRTM-aist-1.2.0のバグ回避

  if (readPath() <= 0) {
    m_rtcError = true;  //OpenRTM-aist-1.2.0のバグ回避
    return RTC::RTC_ERROR;
  }

  m_statePrev = RTC::FOLLOWER_UNKNOWN;

  m_localizationMode.data = 0;
  m_localizationModeOut.write();

  return RTC::RTC_OK;
}


RTC::ReturnCode_t pathCommander_arm::onDeactivated(RTC::UniqueId ec_id)
{
  RTC_INFO(("onDeactivated()"));
  return RTC::RTC_OK;
}


RTC::ReturnCode_t pathCommander_arm::onExecute(RTC::UniqueId ec_id)
{
  if (m_rtcError) return RTC::RTC_ERROR; //OpenRTM-aist-1.2.0のバグ回避

  if (m_currentPoseIn.isNew()) {
    m_currentPoseIn.read();
  }
  if (m_cameraxyIn.isNew()) {
    m_cameraxyIn.read();
    m_statusString.data = "Path";
    m_statusStringOut.write();
    cout << "m_cameraxyルート突入" << endl;
    cout << m_cameraxy.data << endl;
    double x, y;
    InPath();
    cout << "InRead" << endl;
    stringstream ss;
    string Move;
    ss << m_cameraxy.data;
    ss >> Move;
    if (Move == "Move") {
      ss >> x;
      ss.ignore();
      ss >> y;
      cout << "x=" << x << "y=" << y << endl;
      RTC::Path2D path;
      path.waypoints.length(0);

      if (!ServicePortCheck(m_FollowerMotion)) {
        RTC_INFO(("FollowerMotion port is not ready."));
        cout << "!ServicePortCheck(m_FollowerMotion)" << endl;////////////////////
        return RTC::RTC_OK;
      }
      cout << "m_paths.size = " << m_paths.size() << endl;
      Number = m_paths.size() - 1;
      RTC::RETURN_VALUE rv = m_FollowerMotion->followPathNonBlock(m_paths[Number]);
      RTC_INFO(("rv = %d", int(rv)));
      /*double x, y;
      stringstream ss;
      string Move;
      ss << m_cameraxy.data;
      ss >> Move;
      if (Move == "Move") {
        ss >> x;
        ss.ignore();
        ss >> y;
        cout << "x=" << x << "y=" << y << endl;

        double positionx = m_currentPose.data.position.x + x;
        double positiony = m_currentPose.data.position.y + y - 0.20;
        cout << "positionx:" << positionx << ", positiony:" << positiony << endl;

        //cout << line << ": " << vp.size() << ": " << x << ", " << y << endl;
        //vp.push_back(Point2d(positionx, positiony));
       // RTC::Path2D path;
        //setPath(path, vp);
        //m_paths.push_back(path);
        //vp.clear();
      } else { cout << "??わからない??" << endl; }
      */
    }
  }

  if (m_commandStringIn.isNew()) {
    cout << "txtから出力ルート突入" << endl;
    m_commandStringIn.read();
    string cs(m_commandString.data);
    cout << "cs=" << cs << endl;///////////////////
    istringstream is(cs);
    string head;
    is >> head;
    cout << "head=" << head << endl;/////////////////
    if (head == "path") {
      int pathNumber;
      is >> pathNumber;
      cout << "pathNumber=" << pathNumber << endl;//////////////////
      if (!is) {
        RTC_ERROR(("Command unrecognize: %s", cs.c_str()));
        cout << "!is" << endl;////////////////////
        return RTC::RTC_OK;
      }
      if (pathNumber < 0 || pathNumber >= m_paths.size()) {
        RTC_ERROR(("out of range: %s", cs.c_str()));
        cout << "pathNumber < 0 || pathNumber >= m_paths.size()" << endl;///////////
        return RTC::RTC_OK;
      }
      if (!ServicePortCheck(m_FollowerMotion)) {
        RTC_INFO(("FollowerMotion port is not ready."));
        cout << "!ServicePortCheck(m_FollowerMotion)" << endl;////////////////////
        return RTC::RTC_OK;
      }
      RTC::RETURN_VALUE rv = m_FollowerMotion->followPathNonBlock(m_paths[pathNumber]);
      cout << "rv=" << rv << endl;
      RTC_INFO(("rv = %d", int(rv)));
    } else if (head == "stop") {
      cout << "head=stop" << endl;///////////////////////////////////
      //SimplePathFollowerへ長さ0のpathを渡して停止させる．
      RTC::Path2D path;
      path.waypoints.length(0);
      if (!ServicePortCheck(m_FollowerMotion)) {
        RTC_INFO(("FollowerMotion port is not ready."));
        cout << "!ServicePortCheck(m_FollowerMotion)" << endl;///////////////////////////
        return RTC::RTC_OK;
      }
      RTC::RETURN_VALUE rv = m_FollowerMotion->followPathNonBlock(path);
      RTC_INFO(("rv = %d", int(rv)));
    }
  }
  try {
    if (!ServicePortCheck(m_FollowerMotion)) {
      RTC_INFO(("FollowerMotion port is not ready."));
      return RTC::RTC_OK;
    }
    RTC::FOLLOWER_STATE state;
    RTC::RETURN_VALUE rv = m_FollowerMotion->getState(state);
    if (state != m_statePrev) {
      string ss = "unknown";
      if (state == RTC::FOLLOWER_STOPPED) {
        ss = "stopped";
      } else if (state == RTC::FOLLOWER_FOLLOWING) {
        ss = "following";
      }
      RTC_INFO((("m_statusString.data: " + ss).c_str()));
      m_statusString.data = ss.c_str();
      m_statusStringOut.write();
    }
    m_statePrev = state;
  } catch (...) {
    RTC_ERROR(("catch(...)"));
    return RTC::RTC_ERROR;
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t pathCommander_arm::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t pathCommander_arm::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t pathCommander_arm::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t pathCommander_arm::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t pathCommander_arm::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

int pathCommander_arm::readPath()
{
  ifstream fin;
  double sum_x = 0, sum_y = 0;
  const int ave_set = 10;
  fin.open(m_filename);
  if (!fin) {
    RTC_ERROR(("Cannot open %s", m_filename.c_str()));
    return -1;
  }
  vector <Point2d> vp;
  string buf;
  int line = 0;
  while (getline(fin, buf)) {
    line++;
    //cout << "[" << buf << "]" << endl;
    if (buf[0] == '#') continue;
    //cout << "buf.size(): " << buf.size() << endl;
    //cout << "vp.size(): " << vp.size() << endl;
    if (buf.size() == 0) {
      //空行ならば
      if (vp.size() != 0) {
        //データを読み込んでいるならば
        RTC::Path2D path;
        setPath(path, vp);
        m_paths.push_back(path);
        vp.clear();
      }
      continue;
    }
    istringstream is(buf);
    double x, y, r;
    is >> x >> y >> r;
    if (!is) {
      RTC_ERROR(("Failed to read line %d: %s", line, buf.c_str()));
      return -1;
    }
    cout << line << ": " << vp.size() << ": " << x << ", " << y << "," << r << endl;
    vp.push_back(Point2d(x, y, r));
  }

  if (vp.size() != 0) {
    RTC::Path2D path;
    setPath(path, vp);
    m_paths.push_back(path);
  }
  RTC_INFO(("%d paths read", m_paths.size()));
  return m_paths.size();
}
////////////////////////////////////////////////////////////
int pathCommander_arm::InPath()
{
  cout << "InPath" << endl;
  const int ave_set = 10;
  vector <Point2d> vp;
  string buf;
  int line = 0;
  double x, y, r;
  m_cameraxyIn.read();
  cout << m_cameraxy.data << endl;
  stringstream ss;
  string Move;
  ss << m_cameraxy.data;
  ss >> Move;
  cout << "Move=" << Move << endl;
  if (Move == "Move") {
    ss >> x;
    cout << "x=" << x << endl;
    ss.ignore();
    ss >> y;
    cout << "y=" << y << endl;
    cout << "x=" << x << "y=" << y << endl;

    double positionx = m_currentPose.data.position.x + x - 0.15;
    double positiony = m_currentPose.data.position.y + y - 0.1;
    cout << "m_currentPose.data.position.x:" << m_currentPose.data.position.x << endl;
    cout << "m_currentPose.data.position.y:" << m_currentPose.data.position.y << endl;
    cout << "m_currentPose.data.heading:" << m_currentPose.data.heading << endl;
    cout << "positionx:" << positionx << ", positiony:" << positiony << endl;
    vp.push_back(Point2d(m_currentPose.data.position.x, m_currentPose.data.position.y, m_currentPose.data.heading));
    vp.push_back(Point2d(positionx, positiony, m_currentPose.data.heading));
    if (vp.size() != 0) {
      cout << "vp.size() != 0" << endl;
      RTC::Path2D path;
      setPath(path, vp);
      m_paths.push_back(path);
    }
    cout << line << ": " << vp.size() << ": " << x << ", " << y << endl;
    RTC_INFO(("%d paths read", m_paths.size()));
    return m_paths.size();
  }
}
//////////////////////////////////////////////////////////////
void pathCommander_arm::setPath(RTC::Path2D &path, const std::vector<Point2d> &vp)
{
  path.waypoints.length(vp.size());
  for (int i = 0; i < vp.size(); i++) {
    path.waypoints[i] = m_waypointDefault;
    path.waypoints[i].target.position.x = vp[i].x;
    path.waypoints[i].target.position.y = vp[i].y;
    path.waypoints[i].target.heading = vp[i].r;
  }
}

extern "C"
{

  void pathCommander_armInit(RTC::Manager* manager)
  {
    coil::Properties profile(pathcommander_arm_spec);
    manager->registerFactory(profile,
      RTC::Create<pathCommander_arm>,
      RTC::Delete<pathCommander_arm>);
  }

};


