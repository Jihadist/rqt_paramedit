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

#ifndef XML_RPC_TREE_ITEM_H
#define XML_RPC_TREE_ITEM_H

#include <ros/ros.h>

#include <QVariant>
#include <deque>

using std::deque;

/// A wrapper around the XmlRpcValue including a parent pointer and convenience functions for the qt model interface.
/**
 * There are two kinds of item:
 * If the xmlrpcvalue is a map, this will be a real tree item with children.
 * Otherwise it will be a leaf node only storing the value.
 * The tree is build from the internal XmlRpcValue's maps.
 *
 * Corresponding model indices should always refer to the parent item in its internal pointer.
 *
 * A Model index consisting of an internal pointer (XmlRpcTreeItem) and index (row, col) refers to:
 * The row'th child of the internal pointer. (col 0 = name, col 1 = value).
 */
class XmlRpcTreeItem
{
public:
  /**
   * \param [in] path the path in the parameter server to this item
   */
  XmlRpcTreeItem(XmlRpc::XmlRpcValue* data, XmlRpcTreeItem* parent, const std::string& path, ros::NodeHandle* nh);
  ~XmlRpcTreeItem();

  XmlRpcTreeItem* parent()
  {
    return _parent;
  }

  unsigned int childCount() const;
  XmlRpcTreeItem* child(unsigned int i)
  {
    if (i >= _children.size())
      return NULL;
    return _children[i];
  }
  int childIndexOf(const XmlRpcTreeItem* child) const;

  /// Figure out which row/nth child  we are for the parent
  int row() const;

  /// return the data in the map
  QVariant data(int row, int column) const;
  /// is the stored data a bool
  bool isBool(int row, int column) const;

  /// set data for this item
  bool setData(QVariant val);

protected:
  /// Create all children based on data.
  void createChildren();

  /// Create a child from its data.
  void addChild(const std::string& name, XmlRpc::XmlRpcValue* childData);

  /// Convert a XmlRpcValue to QVariant - only leaf values are supported.
  QVariant xmlToVariant(XmlRpc::XmlRpcValue& val) const;

  /// write this XmlRpcValue to parameter server (for array childs).
  void setParam();

protected:
  XmlRpc::XmlRpcValue* _data;

  XmlRpcTreeItem* _parent;

  std::string _path;

  ros::NodeHandle* _nh;

  deque<XmlRpcTreeItem*> _children;

private:
  void test()
  {
  }
};

#endif
