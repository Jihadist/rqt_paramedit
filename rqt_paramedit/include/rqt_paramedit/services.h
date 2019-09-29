#ifndef SERVICES_H
#define SERVICES_H

#include <std_srvs/Empty.h>
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <ros/master.h>

#include <set>
#include <string>
#include <algorithm>
#include <map>
#include <utility>

#include <QModelIndex>
#include <QString>

class Services : public QObject
{
  typedef std::map<std::string, QModelIndex> mapOfservices;
  Q_OBJECT
public:
  friend std::string ServiceToXml(const std::vector<std::string>& vec);

  friend std::vector<std::string> split(const std::string& s, char seperator);
  Services() = default;

  Services* loadData();
  mapOfservices* getNames() const;
  mapOfservices* findServices();
  mapOfservices* findServices(std::string&);

  bool callServices();
  bool callService(std::string service);

  mapOfservices getServicesMap() const;
  void setServicesMap(const mapOfservices& servicesMap);
  mapOfservices* raw()
  {
    return &_servicesMap;
  }

private:
  ros::NodeHandle _nh;

  XmlRpc::XmlRpcValue _data;

  ros::ServiceClient _client;
  std_srvs::Empty _srvMsg;

  mapOfservices _servicesMap;

  // Dev funcs
  void printServices();
public slots:
  void call();
};

#endif  // SERVICES_H
