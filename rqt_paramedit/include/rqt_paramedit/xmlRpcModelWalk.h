#ifndef XMLRPCMODELWALK_H
#define XMLRPCMODELWALK_H

#include <QObject>
#include <qt_paramedit/xmlRpcModel.h>
#include <QModelIndex>
#include <map>

class xmlRpcModelWalk : public XmlRpcModel
{
  Q_OBJECT
public:
  xmlRpcModelWalk(XmlRpc::XmlRpcValue* rootData, const std::string& rootPath, ros::NodeHandle* nh)
    : XmlRpcModel(rootData, rootPath, nh)
  {
  }
  void walk(std::map<std::string, QModelIndex>& out)
  {
    // out.clear();
    walkImpl(getRoot(), out);
  }

private:
  void walkImpl(XmlRpcTreeItem* item, std::map<std::string, QModelIndex>& map);
};

#endif  // XMLRPCMODELWALK_H
