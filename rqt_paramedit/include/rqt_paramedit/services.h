#ifndef SERVICES_H
#define SERVICES_H


#define _DEBUG

#include <std_srvs/Empty.h>
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <ros/master.h>

#include <set>
#include <string>
#include <algorithm>

class Services
{
  typedef std::set<std::string> setOfServices;
public:
  Services()=default;

  Services *loadData();
  setOfServices *getNames() const;
  setOfServices *findServices();
  setOfServices *findServices(std::string &);

  bool callServices();
  bool callService(std::string service);

  setOfServices getServicesList() const;
  void setServicesList(const setOfServices &value);

private:
  ros::NodeHandle _nh;

  XmlRpc::XmlRpcValue _data;

  ros::ServiceClient _client;
  std_srvs::Empty _srvMsg;

  setOfServices _servicesList;

  // Dev funcs
  void printServices();
};

#endif // SERVICES_H
