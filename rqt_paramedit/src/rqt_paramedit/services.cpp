#include "rqt_paramedit/services.h"

Services::setOfServices Services::getServicesList() const
{
  return _servicesList;
}

void Services::setServicesList(const setOfServices& value)
{
  _servicesList = value;
}

Services* Services::loadData()
{
#ifdef _DEBUG
  ROS_INFO("Services::loadData()");
#endif
  XmlRpc::XmlRpcValue request = "/node";
  XmlRpc::XmlRpcValue response;
  // Get services names
  if (ros::ok())
    ros::master::execute("getSystemState", request, response, _data, true);
  else
  {
    return nullptr;
  }
  return this;
}

void Services::printServices()
{
#ifdef _DEBUG
  ROS_INFO("Services::printServices()");
#endif
  for (const auto& i : _servicesList)
  {
    ROS_INFO("%s", i.c_str());
  }
}

Services::setOfServices* Services::findServices()
{
#ifdef _DEBUG
  ROS_INFO("Services::findServices()");
#endif
  auto& buf = _data[2];
  for (int i = 0; i < buf.size(); ++i)
  {
    // Get only service name from list
    std::string topicName = buf[i][0];
    std::string searchTopic = "update_parameters";

    if (std::equal(searchTopic.rbegin(), searchTopic.rend(), topicName.rbegin()))
      _servicesList.insert(topicName);
  }
  printServices();
  return &_servicesList;
}

Services::setOfServices* Services::findServices(std::string& searchTopic)
{
  auto& buf = _data[2];
  for (int i = 0; i < buf.size(); ++i)
  {
    // Get only service name from list
    std::string topicName = buf[i][0];
    // searchTopic="update_parameters";

    if (std::equal(searchTopic.rbegin(), searchTopic.rend(), topicName.rbegin()))
      _servicesList.insert(topicName);
  }
  printServices();
  return &_servicesList;
}

bool Services::callServices()
{
#ifdef _DEBUG
  ROS_INFO("Services::callServices()");
#endif
  // Call services
  for (auto s : _servicesList)
    callService(s);
  return true;
}

bool Services::callService(std::string service)
{
#ifdef _DEBUG
  ROS_INFO("Services::callService()");
#endif
  // Call service

  _client = _nh.serviceClient<std_srvs::Empty>(service);

  if (_client.call(_srvMsg))
  {
    ROS_INFO("%s", service.append(" is called").c_str());
  }
  else
  {
    std::string errorMsg = "Failed to call service: ";
    ROS_ERROR("%s", errorMsg.append(service).c_str());
    return false;
  }

  return true;
}
