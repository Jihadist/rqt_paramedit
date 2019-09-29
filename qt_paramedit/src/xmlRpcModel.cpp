/*
 * Copyright (c) 2015, C. Dornhege, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the University of Freiburg nor the names
 *    of its contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qt_paramedit/xmlRpcModel.h"
#include <QDebug>
#include <ros/console.h>

XmlRpcModel::XmlRpcModel(XmlRpc::XmlRpcValue* rootData, const std::string& rootPath, ros::NodeHandle* nh)
{
#ifdef _DEBUG
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif
  ROS_DEBUG(__PRETTY_FUNCTION__);
  _root = new XmlRpcTreeItem(rootData, NULL, rootPath, nh);

  _maxDisplayLength = 120;
}

XmlRpcModel::~XmlRpcModel()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  delete _root;
}

QModelIndex XmlRpcModel::index(int row, int column, const QModelIndex& parent) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  if (!parent.isValid())
  {
    return createIndex(row, column, _root);
  }

  auto* parentItem = static_cast<XmlRpcTreeItem*>(parent.internalPointer());
  // parent points to the (parent.row, parent.column) child of parentItem and its corresponding tree node
  // We want to create a child of that.
  // First check if that (parent.row, parent.column) child of parentItem can be a parent for an index again
  // Only items in the first column can be parents
  if (parent.column() > 0)
    return QModelIndex();
  // Lookup that child item and use it as parent for the new index
  auto* childItem = parentItem->child(parent.row());
  if (childItem)
  {
    // only items with children can be parents
    if (childItem->childCount() == 0)
      return QModelIndex();
    // create an index referring to the (row, column) child of childItem
    return createIndex(row, column, childItem);
  }
  else
  {
    return QModelIndex();
  }
}

QModelIndex XmlRpcModel::parent(const QModelIndex& index) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  if (!index.isValid())
    return QModelIndex();

  auto* childItem = static_cast<XmlRpcTreeItem*>(index.internalPointer());
  auto* parentItem = childItem->parent();

  // top level
  if (parentItem == NULL)
    return QModelIndex();

  return createIndex(childItem->row(), 0, parentItem);
}

Qt::ItemFlags XmlRpcModel::flags(const QModelIndex& index) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  if (!index.isValid())
    return 0;

  if (index.column() == 1)
  {  // right sides of inner nodes have no flags
    auto* parentItem = static_cast<XmlRpcTreeItem*>(index.internalPointer());
    auto* item = parentItem->child(index.row());
    // index points to index.rows child of parentItem
    if (item && item->childCount() > 0)
    {
      return 0;
    }
  }

  Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;

  if (index.column() == 1)
  {  // right side = value, editable
    // if the data is bool make it checkable
    XmlRpcTreeItem* item = static_cast<XmlRpcTreeItem*>(index.internalPointer());
    if (item->isBool(index.row(), index.column()))
    {
      flags |= Qt::ItemIsUserCheckable;
    }
    else
    {
      flags |= Qt::ItemIsEditable;
    }
  }

  return flags;
}

QVariant XmlRpcModel::data(const QModelIndex& index, int role) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  if (!index.isValid())
    return QVariant();

  // printf("data() Role is %d\n", role);
  if (role != Qt::DisplayRole && role != Qt::CheckStateRole && role != Qt::EditRole)
    return QVariant();

  XmlRpcTreeItem* item = static_cast<XmlRpcTreeItem*>(index.internalPointer());

  if (role == Qt::CheckStateRole)
  {  // only valid for bools
    if (!item->isBool(index.row(), index.column()))
      return QVariant();
    // printf("RET IS %d", item->data(index.row(), index.column()).toBool());

    // for bool values in CheckStateRole return Qt::Checked/Unchecked instead of true/false
    return item->data(index.row(), index.column()).toBool() ? Qt::Checked : Qt::Unchecked;
  }

  if (role == Qt::DisplayRole)
  {  // no text for bools
    if (item->isBool(index.row(), index.column()))
      return QVariant();
    else
    {
      QVariant itemDisplay = item->data(index.row(), index.column());
      if (itemDisplay.type() != QVariant::String)
      {
        return itemDisplay;
      }
      // convert here to string to check _maxDisplayLength
      QString itemStr = itemDisplay.toString();
      unsigned int maxLength = _maxDisplayLength;
      if (maxLength < 3)
        maxLength = 3;
      if (itemStr.length() > (int)maxLength)
      {
        itemStr = itemStr.mid(0, maxLength - 3) + "...";
      }
      return itemStr;
    }
  }

  if (role == Qt::EditRole)
  {
    return item->data(index.row(), index.column());
  }

  // never called
  return QVariant();
}

bool XmlRpcModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  if (!index.isValid())
    return false;
  if (role != Qt::EditRole && role != Qt::CheckStateRole)
    return false;

  if (index.column() != 1)
    return false;

  auto* parentItem = static_cast<XmlRpcTreeItem*>(index.internalPointer());
  if (parentItem->isBool(index.row(), index.column()))
  {
    if (role == Qt::EditRole)
      return false;
  }
  if (!parentItem->isBool(index.row(), index.column()))
  {
    if (role == Qt::CheckStateRole)
      return false;
  }

  auto* item = parentItem->child(index.row());

  // this should always be a leaf, never a struct!
  if (item->setData(value))
  {
    // connect(index.model(),QAbstractItemModel::dataChanged(index,index),)
    // index.model()
    qDebug() << index.data();
    Q_EMIT dataChanged(index, index);
    return true;
  }

  return false;
}

XmlRpcTreeItem* XmlRpcModel::getRoot() const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  return _root;
}

void XmlRpcModel::std()
{
  auto buf = this->persistentIndexList();
  for (const auto& i : buf)
    ROS_INFO("%s", i.data().toString().toStdString().c_str());
}

QVariant XmlRpcModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  if (role != Qt::DisplayRole || orientation != Qt::Horizontal)
    return QVariant();

  if (section == 0)
    return "Parameter";
  if (section == 1)
    return "Value";

  return QVariant();
}

int XmlRpcModel::rowCount(const QModelIndex& parent) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  if (!parent.isValid())
  {
    return _root->childCount();
  }

  if (parent.column() > 0)
    return 0;

  // parent refers to the parent.row'th child of parentItem
  // get that item
  auto* parentItem = static_cast<XmlRpcTreeItem*>(parent.internalPointer());
  auto* item = parentItem->child(parent.row());

  if (item == NULL)
    return 0;
  return item->childCount();
}

int XmlRpcModel::columnCount(const QModelIndex& parent) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  if (!parent.isValid())
  {
    if (_root->childCount() > 0)
      return 2;
    return 0;
  }

  if (parent.column() > 0)
    return 0;

  // parent is the parent.row'th child of parentItem
  // get that item
  auto* parentItem = static_cast<XmlRpcTreeItem*>(parent.internalPointer());
  auto* item = parentItem->child(parent.row());

  if (item == NULL)
    return 0;

  // with children we address 0->name 1->value
  if (item->childCount() > 0)
    return 2;
  return 0;
}
