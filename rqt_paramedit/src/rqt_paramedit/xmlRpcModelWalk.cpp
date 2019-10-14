#include "rqt_paramedit/xmlRpcModelWalk.h"

void xmlRpcModelWalk::walkImpl(XmlRpcTreeItem* item, std::map<std::string, QModelIndex>& map)
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  for (unsigned int row = 0; row < item->childCount(); ++row)
  {
    std::string name = *(item->child(row)->getPath());
    name.append("/update_parameters");

    auto it = map.find(name);
    if (it != map.end() && (it->second == QModelIndex()))
    {
      auto tmp_ind = this->createIndex(row, 1, item);
      it->second = tmp_ind;
      std::cout << row << " Index add: " << it->first << std::endl;
    }
    walkImpl(item->child(row), map);
  }
}
