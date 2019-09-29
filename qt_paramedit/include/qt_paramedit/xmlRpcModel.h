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

#ifndef XML_RPC_MODEL_H
#define XML_RPC_MODEL_H

#include <QAbstractItemModel>

#include "xmlRpcTreeItem.h"

//#define _DEBUG

/// QAbstractItemModel for XmlRpcValues.
/**
 * Corresponding QModelIndeces will have the XmlRpcTreeItem that corresponds
 * to the parent's XmlRpcValue as their internal pointer.
 * So, only TreeItems with maps are contained in model indices.
 *
 * Actual leaf data is accessed via the row/column of the QModelIndex.
 */
class XmlRpcModel : public QAbstractItemModel
{
  Q_OBJECT

public:
  XmlRpcModel(XmlRpc::XmlRpcValue* rootData, const std::string& rootPath, ros::NodeHandle* nh);
  ~XmlRpcModel();

  QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const;
  QModelIndex parent(const QModelIndex& index) const;

  QVariant data(const QModelIndex& index, int role) const;
  Qt::ItemFlags flags(const QModelIndex& index) const;
  QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

  /// should return the number of children for a parent item
  int rowCount(const QModelIndex& parent = QModelIndex()) const;
  int columnCount(const QModelIndex& parent = QModelIndex()) const;

  bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole);

  unsigned int getMaxDisplayLength() const
  {
    return _maxDisplayLength;
  }
  void setMaxDisplayLength(unsigned int l)
  {
    _maxDisplayLength = l;
  }

  XmlRpcTreeItem* getRoot() const;

  void std();

private:
  XmlRpcTreeItem* _root;

  unsigned int _maxDisplayLength;
};

#endif
