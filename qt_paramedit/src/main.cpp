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

#include <ros/ros.h>
#include <QApplication>
#include <string>
#include <cstdio>
#include <cstdlib>
#include "MainWindow.h"
#include "qt_paramedit/xmlRpcModel.h"

void usage(char* pname)
{
   printf("Usage: %s [parameter root] (default: /)\n", pname);
   exit(1);
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "qt_paramedit");

   std::string paramRoot = "/";
   if(argc > 1) {
      if(argc > 2)                           // only 1 param
         usage(argv[0]);
      paramRoot = argv[1];
   }

   ros::NodeHandle nh;

   std::string err;
   if(!ros::names::validate(paramRoot, err)) {
      ROS_FATAL("Invalid name: %s - Error: %s", paramRoot.c_str(), err.c_str());
      usage(argv[0]);
   }
   if(paramRoot.empty()) {
      ROS_FATAL("Empty param root.");
      usage(argv[0]);
   }

   XmlRpc::XmlRpcValue xmlrpc;
   if(!nh.getParam(paramRoot, xmlrpc)) {
      ROS_FATAL("Could not get parameters at: \"%s\"", paramRoot.c_str());
      return 1;
   }
   if(xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_FATAL("Requested param root has non struct type. Can only handle dictionaries, not single parameters.");
      usage(argv[0]);
   }
   ROS_DEBUG("Retrieved parameters from: \"%s\"", paramRoot.c_str());

   QApplication app(argc, argv);
   MainWindow gui(xmlrpc, paramRoot, &nh);

   gui.show();

   ros::WallRate loop(20.0);
   while(ros::ok() && gui.isVisible()) {
      ros::spinOnce();

      app.processEvents();

      loop.sleep();
   }

   return 0;
}
