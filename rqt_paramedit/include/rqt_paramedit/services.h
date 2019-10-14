#ifndef SERVICES_H
#define SERVICES_H

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
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
#include <QTreeView>
#include <QPushButton>
#include <QDebug>
#include <QSignalMapper>

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
  // void createButtons(QTreeView* tree);
  void createButtons(QTreeView* tree);

private:
  ros::NodeHandle _nh;

  XmlRpc::XmlRpcValue _data;

  ros::ServiceClient _client;
  // std_srvs::Empty _srvMsg;
  std_srvs::Trigger _srvMsg;
  std_srvs::TriggerRequest _srvReq;
  std_srvs::TriggerResponse _srvRes;

  mapOfservices _servicesMap;

  // Dev funcs
  void printServices();
signals:
  void serviceCalled(std::string& s);
  void serviceFailed(std::string& s);
};

#endif  // SERVICES_H
