#include "rqt_paramedit/services.h"

std::vector<std::string> split(const std::string& s, char seperator)
{
  std::vector<std::string> output;

  std::string::size_type prev_pos = 0, pos = 0;

  while ((pos = s.find(seperator, pos)) != std::string::npos)
  {
    if (pos - prev_pos != 0)
    {
      std::string substring(s.substr(prev_pos, pos - prev_pos));

      output.push_back(substring);

      prev_pos = ++pos;
    }
    else
    {
      prev_pos = ++pos;
    }
  }

  output.push_back(s.substr(prev_pos, pos - prev_pos));  // Last word

  return output;
}

std::string ServiceToXml(const std::vector<std::string>& vec)
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  QString str;
  for (auto it = vec.rbegin(); it != vec.rend() - 1; ++it)
  {
    str.prepend(QString(it->c_str()).prepend("<name>").append("</name>"));
    str.prepend("<value><struct><member>");
    str.append("</member></struct></value>");
  }
  return str.toStdString();
}

Services* Services::loadData()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  XmlRpc::XmlRpcValue request = "/node";
  XmlRpc::XmlRpcValue response;
  // Get services names
  if (ros::ok())
    ros::master::execute("getSystemState", request, response, _data, true);
  else
  {
    return nullptr;
  }
  ROS_DEBUG_COND(_data.getType() == XmlRpc::XmlRpcValue::TypeArray, "_data is array");
  ROS_DEBUG_COND(_data.getType() == XmlRpc::XmlRpcValue::TypeStruct, "_data is struct");
  ROS_DEBUG_COND(_data.getType() == XmlRpc::XmlRpcValue::TypeString, "_data is string");
  return this;
}

void Services::printServices()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  for (const auto& i : _servicesMap)
  {
    ROS_INFO("%s", i.first.c_str());
  }
}

Services::mapOfservices* Services::findServices()
{
  // XmlRpc::XmlRpcValue::ValueStruct _struct;
  // _struct.insert()
  ROS_DEBUG(__PRETTY_FUNCTION__);

  auto& buf = _data[2];
  ROS_DEBUG_COND(buf.getType() == XmlRpc::XmlRpcValue::TypeArray, "Buffer is array");
  ROS_DEBUG_COND(buf.getType() == XmlRpc::XmlRpcValue::TypeStruct, "Buffer is struct");
  ROS_DEBUG_COND(buf.getType() == XmlRpc::XmlRpcValue::TypeString, "Buffer is string");
  // buf.write(std::cout);

  for (int i = 0; i < buf.size(); ++i)
  {
    // Get only service name from list
    std::string topicName = buf[i][0];
    std::string searchTopic = "update_parameters";

    if (std::equal(searchTopic.rbegin(), searchTopic.rend(), topicName.rbegin()))
    {
      std::vector<std::string> vec;

      //_servicesList.insert(topicName);

      _servicesMap.emplace(topicName, QModelIndex());
      // std::cout << std::endl << buf[i].toXml() << std::endl;

      auto buf = split(topicName, '/');
      // for (const auto i : buf)
      //{
      //  vec.push_back(i);
      // std::cout << i << " : " << vec.size() << std::endl;
      //}
      //_struct.emplace(std::pair<std::string, XmlRpc::XmlRpcValue>(vec.front(), buf[i]));
      // _struct.insert(vec.front(), buf[i]);
      // ROS_DEBUG("%s", ServiceToXml(vec).c_str());
    }
  }
  // for (const auto& i : _struct)
  //  {
  //   std::cout << i.first << " : ";
  //    i.second.write(std::cout);
  //    std::cout << std::endl;

  //  }
  printServices();
  return &_servicesMap;
}

Services::mapOfservices* Services::findServices(std::string& searchTopic)
{
  auto& buf = _data[2];
  for (int i = 0; i < buf.size(); ++i)
  {
    // Get only service name from list
    std::string topicName = buf[i][0];
    // searchTopic="update_parameters";

    if (std::equal(searchTopic.rbegin(), searchTopic.rend(), topicName.rbegin()))
      //_servicesList.insert(topicName);
      _servicesMap.emplace(topicName, QModelIndex());
  }
  printServices();
  return &_servicesMap;
}

bool Services::callServices()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  // Call services
  // for (auto s : _servicesList)
  // callService(s);
  for (auto s : _servicesMap)
    callService(s.first);
  return true;
}

bool Services::callService(std::string service)
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

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

Services::mapOfservices Services::getServicesMap() const
{
  return _servicesMap;
}

void Services::setServicesMap(const mapOfservices& servicesMap)
{
  _servicesMap = servicesMap;
}
