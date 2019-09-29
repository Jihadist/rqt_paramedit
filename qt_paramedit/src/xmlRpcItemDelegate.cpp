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

#include "qt_paramedit/xmlRpcItemDelegate.h"
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <ros/console.h>

XmlRpcItemDelegate::XmlRpcItemDelegate(QObject* parent) : QStyledItemDelegate(parent)
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  doubleDecimals = 10;
}

QWidget* XmlRpcItemDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                                          const QModelIndex& index) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  // based on the index figure out which data type we are editing:
  QVariant data = index.model()->data(index, Qt::EditRole);
  switch (data.type())
  {
    case QVariant::Double:  // bring up double spinbox with higher decimals
    {
      auto* editor = new QDoubleSpinBox(parent);
      editor->setDecimals(doubleDecimals);
      editor->setMinimum(-INFINITY);
      editor->setMaximum(INFINITY);
      return editor;
    }
    case QVariant::Int:     // bring up spinbox (done by default)
    case QVariant::String:  // use std line edit, but fill it (done by default)
    default:
      return QStyledItemDelegate::createEditor(parent, option, index);
  }
}

void XmlRpcItemDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  QStyledItemDelegate::setEditorData(editor, index);
}

void XmlRpcItemDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  QStyledItemDelegate::setModelData(editor, model, index);
}

void XmlRpcItemDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                                              const QModelIndex& index) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  QStyledItemDelegate::updateEditorGeometry(editor, option, index);
}

QString XmlRpcItemDelegate::displayText(const QVariant& value, const QLocale& locale) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  if (value.type() == QVariant::Double)
  {
    return locale.toString(value.toDouble(), 'g', doubleDecimals);
  }
  else
  {
    return QStyledItemDelegate::displayText(value, locale);
  }
}
