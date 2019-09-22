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

#include "qt_paramedit/xmlRpcTreeItem.h"
#include <QDateTime>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

XmlRpcTreeItem::XmlRpcTreeItem(XmlRpc::XmlRpcValue* data, XmlRpcTreeItem* parent, const std::string & path, 
      ros::NodeHandle* nh)
   : _data(data), _parent(parent), _path(path), _nh(nh)
{
   ROS_ASSERT(_nh != NULL);

   createChildren();
}

XmlRpcTreeItem::~XmlRpcTreeItem()
{
   forEach(XmlRpcTreeItem* child, _children)
      delete child;
   _children.clear();
}

/**
 * Children are either from the struct and thus real children or from the array and thus contained
 * in the array in _data. 
 */
unsigned int XmlRpcTreeItem::childCount() const 
{ 
   if(_children.size() > 0)
      return _children.size(); 

   return 0;
}

int XmlRpcTreeItem::childIndexOf(const XmlRpcTreeItem* child) const
{
   int index = -1;
   for(unsigned int i = 0; i < _children.size(); i++) {
      if(child == _children[i]) {
         index = i;
         break;
      }
   }
   return index;
}

int XmlRpcTreeItem::row() const
{
   if(_parent) {
      return _parent->childIndexOf(this);
   }

   // root at index 0
   return 0;
}

QVariant XmlRpcTreeItem::xmlToVariant(XmlRpc::XmlRpcValue & val) const
{
   switch(val.getType()) {
      case XmlRpc::XmlRpcValue::TypeBoolean:
         return QVariant( (bool)val );
      case XmlRpc::XmlRpcValue::TypeInt:
         return QVariant( (int)val );
      case XmlRpc::XmlRpcValue::TypeDouble:
         return QVariant( (double)val );
      case XmlRpc::XmlRpcValue::TypeString:
   {
        // val.size()>50 ? return QVariant(std::string(val)).substr(0,50) : QVariant(std::string(val));
      if (val.size()>50)
        return QVariant(std::string(val).substr(0,50).append("...").c_str());
      else
        return QVariant(std::string(val).c_str());
   }

      case XmlRpc::XmlRpcValue::TypeDateTime:
         {
            ROS_WARN_THROTTLE(1.0, "Accessing TypeDateTime is untested.");
            struct tm time = (struct tm)val;
            int ms = 0;
            if(time.tm_sec > 59) // might be > 59 for leap seconds, Qt wants 0-59
               ms = 999;

            return QVariant( QDateTime(
                     /*
                      *       tm_mday	day of the month	1-31
                      *       tm_mon	months since January	0-11
                      *       tm_year	years since 1900
                      */
                     QDate(time.tm_year + 1900, time.tm_mon + 1, time.tm_mday),
                     /*
                      *       tm_sec	seconds after the minute	0-61
                      *       tm_min	minutes after the hour	0-59
                      *       tm_hour	hours since midnight	0-23
                      */
                     QTime(time.tm_hour, time.tm_min, time.tm_sec, ms)));
         }
      case XmlRpc::XmlRpcValue::TypeBase64:
         {
            ROS_WARN_THROTTLE(1.0, "Accessing TypeBase64 is untested.");
            XmlRpc::XmlRpcValue::BinaryData & bd = (XmlRpc::XmlRpcValue::BinaryData&)val;
            QByteArray ba;
            for(std::vector<char>::iterator it = bd.begin(); it != bd.end(); it++)
               ba.append(*it);
            return QVariant(ba);
        }
      default:
         return QVariant();
   }
}

QVariant XmlRpcTreeItem::data(int row, int column) const
{
   // this node always has to be a map!

   if(column > 1)
      return QVariant();

   // get the row'th entry
   if(_data->getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      int count = 0;
      for(auto it = _data->begin(); it != _data->end(); it++) {
         if(count == row) {   // at row'th entry
            if(column == 0) { // get the rows'th key
               return it->first.c_str();
            }
            if(column == 1) {
               XmlRpc::XmlRpcValue & val = it->second;
               return xmlToVariant(val);
            }
         }
         count++;
      }
   } else if(_data->getType() == XmlRpc::XmlRpcValue::TypeArray) {
      if(column == 0) {
         return QString("[%1]").arg(row);
      } else {
         XmlRpc::XmlRpcValue & val = (*_data)[row];
         return xmlToVariant(val);
      }
   }

   // nothing found 
   return QVariant();
}

bool XmlRpcTreeItem::isBool(int row, int column) const
{
   // this node always has to be a map!

   if(column != 1)
      return false;

   // get the row'th entry
   if(_data->getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      int count = 0;
      for(auto it = _data->begin(); it != _data->end(); it++) {
         if(count == row) {   // at row'th entry
            XmlRpc::XmlRpcValue & val = it->second;
            if(val.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
               return true;
            else 
               return false;
         }
         count++;
      }
   } else if(_data->getType() == XmlRpc::XmlRpcValue::TypeArray) {
      XmlRpc::XmlRpcValue & val = (*_data)[row];
      if(val.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
         return true;
      else 
         return false;
   }

   // nothing found 
   return false;
}

bool XmlRpcTreeItem::setData(QVariant val)
{
   XmlRpc::XmlRpcValue::Type type = _data->getType();
   if(type == XmlRpc::XmlRpcValue::TypeStruct || type == XmlRpc::XmlRpcValue::TypeArray 
         || type == XmlRpc::XmlRpcValue::TypeInvalid) {
      return false;
   }

   bool setXmlOK = false;
   switch(type) {
      case XmlRpc::XmlRpcValue::TypeBoolean:
         if(!val.canConvert(QVariant::Bool)) {
            ROS_WARN("XmlRpcValue TypeBoolean -- setData cannot convert from QVariant type %d.", val.type());
            break;
         }
         ROS_DEBUG("Setting bool param.");
         (bool&)(*_data) = val.toBool();
         setXmlOK = true;
         break;
      case XmlRpc::XmlRpcValue::TypeInt:
         {
            if(!val.canConvert(QVariant::Int)) {
               ROS_WARN("XmlRpcValue TypeInt -- setData cannot convert from QVariant type %d.", val.type());
               break;
            }
            bool ok = false;
            int iVal =  val.toInt(&ok);
            if(ok) {
               ROS_DEBUG("Setting int param.");
               (int&)(*_data) = iVal;
               setXmlOK = true;
            }
            break;
         }
      case XmlRpc::XmlRpcValue::TypeDouble:
         {
            if(!val.canConvert(QVariant::Double)) {
               ROS_WARN("XmlRpcValue TypeDouble -- setData cannot convert from QVariant type %d.", val.type());
               break;
            }
            // although e.g. strings can be converted to double in principle, the conversion of "hallo" still fails
            // -> check via OK and only update _data if the conversion succeeded.
            bool ok = false;
            double dVal = val.toDouble(&ok);
            if(ok) {
               ROS_DEBUG("Setting double param.");
               (double&)(*_data) = dVal;
               setXmlOK = true;
            }
            break;
         }
      case XmlRpc::XmlRpcValue::TypeString:
         if(!val.canConvert(QVariant::String)) {
            ROS_WARN("XmlRpcValue TypeString -- setData cannot convert from QVariant type %d.", val.type());
            break;
         }
         ROS_DEBUG("Setting string param.");
         (std::string&)(*_data) = qPrintable(val.toString());
         setXmlOK = true;
         break;
      case XmlRpc::XmlRpcValue::TypeDateTime:
         {
            ROS_WARN_THROTTLE(1.0, "Accessing TypeDateTime is untested.");
            if(!val.canConvert(QVariant::DateTime)) {
               ROS_WARN("XmlRpcValue TypeDateTime -- setData cannot convert from QVariant type %d.", val.type());
               break;
            }
            QDateTime dt = val.toDateTime();
            if(dt.isValid()) {
               ROS_DEBUG("Setting datetime param.");
               struct tm time{};
               time.tm_year = dt.date().year() - 1900;
               time.tm_mon = dt.date().month() - 1;
               time.tm_mday = dt.date().day();
               time.tm_hour = dt.time().hour();
               time.tm_min = dt.time().minute();
               time.tm_sec = dt.time().second();
               time.tm_wday = dt.date().dayOfWeek() - 1;
               time.tm_yday = dt.date().dayOfYear() - 1;
               time.tm_isdst = -1;
               time.tm_zone = "";
               time.tm_gmtoff = 0;

               (struct tm &)(*_data) = time;
               setXmlOK = true;
            }
            break;
         }
      case XmlRpc::XmlRpcValue::TypeBase64:
         {
            ROS_WARN_THROTTLE(1.0, "Accessing TypeBase64 is untested.");
            if(!val.canConvert(QVariant::ByteArray)) {
               ROS_WARN("XmlRpcValue TypeBase64 -- setData cannot convert from QVariant type %d.", val.type());
               break;
            }
            QByteArray ba = val.toByteArray();
            XmlRpc::XmlRpcValue::BinaryData bd;
            for(int i = 0; i < ba.size(); i++) {
               bd.push_back(ba.at(i));
            }
            ROS_DEBUG("Setting base64 param.");
            (XmlRpc::XmlRpcValue::BinaryData &)(*_data) = bd;
            break;
         }
      default:
         return false;
   }
   if(setXmlOK) {
      if(!_path.empty()) {
         ROS_DEBUG("Setting param type %d on server path %s.", _data->getType(), _path.c_str());
         _nh->setParam(_path, *_data);
      } else {
         _parent->setParam();
      }
   }

   return true;
}

void XmlRpcTreeItem::setParam()
{
   ROS_ASSERT(_data->getType() == XmlRpc::XmlRpcValue::TypeArray);

   ROS_DEBUG("Setting param type %d on server path %s.", _data->getType(), _path.c_str());
   if(!_path.empty())
      _nh->setParam(_path, *_data);
}

void XmlRpcTreeItem::createChildren()
{
   // only maps and arrays have children

   if(_data->getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      for(XmlRpc::XmlRpcValue::iterator it = _data->begin(); it != _data->end(); it++) {
         addChild(it->first, &it->second);
      }
   } else if(_data->getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < _data->size(); i++) {
         addChild("", & ((*_data)[i]));
      }
   }
}

void XmlRpcTreeItem::addChild(const std::string & name, XmlRpc::XmlRpcValue* childData)
{
   std::string path = ros::names::append(this->_path, name);
   if(name.empty())
      path = name;   // no path
   auto* child = new XmlRpcTreeItem(childData, this, path, _nh);
   _children.push_back(child);
}

