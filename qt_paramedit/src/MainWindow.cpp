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

#include "MainWindow.h"
#include <cstdio>
#include "settingsDialog.h"

MainWindow::MainWindow(XmlRpc::XmlRpcValue & initParams, const std::string & paramRoot, ros::NodeHandle* nh)
   : _xmlrpc(initParams), _paramRoot(paramRoot), _nh(nh)
{
   setupUi(this);

   _model = new XmlRpcModel(&_xmlrpc, _paramRoot, _nh);
   setModel(_model);

   _delegate = new XmlRpcItemDelegate(treeView);
   treeView->setItemDelegate(_delegate);

   actionReload->setIcon(QIcon::fromTheme("view-refresh"));
}

void MainWindow::setModel(QAbstractItemModel * model)
{
   treeView->setModel(model); 
   treeView->resizeColumnToContents(0);
}

void MainWindow::on_actionReload_triggered()
{
   if(!_nh->getParam(_paramRoot, _xmlrpc)) {
      ROS_ERROR("Could not get parameters at: \"%s\"", _paramRoot.c_str());
      return;
   }

   unsigned int maxDisplayLength = 120;
   if(_model)
     maxDisplayLength = _model->getMaxDisplayLength();

   delete _model;
   _model = new XmlRpcModel(&_xmlrpc, _paramRoot, _nh);

   _model->setMaxDisplayLength(maxDisplayLength);

   setModel(_model);
   ROS_INFO("Reloaded parameters from \"%s\"", _paramRoot.c_str());
}

void MainWindow::on_actionSettings_triggered()
{
   SettingsDialog dialog(this);
   dialog.setDoubleDecimals(_delegate->getDoubleDecimals());
   if(_model)
     dialog.setMaxDisplayLength(_model->getMaxDisplayLength());

   if(dialog.exec()) {
      _delegate->setDoubleDecimals(dialog.getDoubleDecimals());
      if(_model) {
        _model->setMaxDisplayLength(dialog.getMaxDisplayLength());
      }
   }
}

