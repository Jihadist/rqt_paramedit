#include "rqt_paramedit/param_edit.h"
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>
#include <QMessageBox>
#include "param_root_chooser.h"
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>

PLUGINLIB_EXPORT_CLASS(rqt_paramedit::ParamEdit, rqt_gui_cpp::Plugin)

namespace rqt_paramedit
{

ParamEdit::ParamEdit() : _treeView(NULL), _model(NULL), _delegate(NULL), _widget(NULL), _updateButton(NULL), _refButton(NULL), _mainLayout(new QVBoxLayout), _horLayout(new QHBoxLayout)
{
    setObjectName("ParamEdit");
    _treeView = new QTreeView();

    _widget=new QWidget();

    _updateButton=new QPushButton();
    _updateButton->setText("Update param");
    _refButton=new QPushButton();
    _refButton->setText("Refresh param");

   _widget->setLayout(_mainLayout);
    _mainLayout->addLayout(_horLayout);
   _mainLayout->addWidget(_treeView);

   _horLayout->addWidget(_refButton);
   _horLayout->addWidget(_updateButton);


   connect(_refButton, SIGNAL (clicked()),this, SLOT (handleRefButton()));
   connect(_updateButton,SIGNAL(clicked()),this,SLOT(handUpdButton()));

}



void ParamEdit::initPlugin(qt_gui_cpp::PluginContext& context)
{


    context.addWidget(_widget);

    _paramRoot = "/";

    _delegate = new XmlRpcItemDelegate(_treeView);
    _treeView->setItemDelegate(_delegate);

    reload();
}

void ParamEdit::reload()
{
    if(!_nh.getParam(_paramRoot, _xmlrpc)) {
        ROS_ERROR("Could not get parameters at: \"%s\"", _paramRoot.c_str());
        QMessageBox::critical(_treeView, "Error loading parameters",
                QString("Could not get parameters at: \"%1\"").arg(_paramRoot.c_str()));

        return;
    } else if(_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Requested parameter at \"%s\" has non-struct type. Only structs are supported, not single parameters.", _paramRoot.c_str());
        QMessageBox::critical(_treeView, "Error loading parameters",
                QString("Requested parameter at \"%1\" has non-struct type. Only structs are supported, not single parameters.\nMaybe, you want to choose the parent containing the selected parameter.").arg(_paramRoot.c_str()));

        // we have now loaded a parameter that can't be interpreted by the model.
        // Try to fix that inconsistency somehow by loading from '/' until the user clears this up
        _paramRoot = "/";
        _nh.getParam(_paramRoot, _xmlrpc);
    }

    delete _model;
    _model = new XmlRpcModel(&_xmlrpc, _paramRoot, &_nh);
    _treeView->setModel(_model);
}

bool ParamEdit::set_new_param()
{
  XmlRpc::XmlRpcValue request = "/node";
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;

  // Get services names
  if (ros::ok() == true)
    ros::master::execute("getSystemState",request,response,payload,true);

  std::vector<std::string> update_list;
  XmlRpc::XmlRpcValue& sub_info = payload[2];
  for (size_t i = 0; i < sub_info.size(); ++i)
  {
    std::string topic_name = sub_info[i][0];
    std::string search_topic="update_parameters";
    auto searched_pos=topic_name.rfind(search_topic);

    if ((searched_pos!=std::string::npos) && topic_name.at(topic_name.size()-search_topic.size()-1)=='/')
      update_list.push_back(topic_name);
  }
      for (const auto &i : update_list)
        std::cout<<i<<std::endl;

  ROS_INFO("Hello world!");


  // Call services
  //ros::NodeHandle nh;
  ros::ServiceClient client;
  //std_msgs::Empty update_msg;
  std_srvs::Empty server;
  for (auto s:update_list)
  {
    client = _nh.serviceClient<std_srvs::Empty>(s);

    if( client.call(server))
    {
        ROS_INFO("%s", s.append(" is called").c_str());
    }
    else
    {
        std::string error_msg="Failed to call service: ";
        ROS_ERROR("%s", error_msg.append(s).c_str());
        return 1;
    }

  }
}

void ParamEdit::handleRefButton()
{
  ROS_INFO("Refresh values initialized");
  reload();
}

void ParamEdit::handUpdButton()
{
  set_new_param();
}

void ParamEdit::shutdownPlugin()
{
}

void ParamEdit::saveSettings(qt_gui_cpp::Settings&  /*global_settings*/, qt_gui_cpp::Settings& perspective_settings) const
{
    perspective_settings.setValue("param_root", _paramRoot.c_str());
}

void ParamEdit::restoreSettings(const qt_gui_cpp::Settings&  /*global_settings*/, const qt_gui_cpp::Settings& perspective_settings)
{
    _paramRoot = qPrintable(perspective_settings.value("param_root", "/").toString());
    reload();
}

void ParamEdit::triggerConfiguration()
{
    ParamRootChooser dialog;
    if(dialog.exec() == QDialog::Accepted) {
        if(dialog.selectedParamRoot().empty()) {
            ROS_ERROR("ParamRootChooser Accepted, but no valid parameter chosen.");
        } else {
            _paramRoot = dialog.selectedParamRoot();
            reload();
        }
    }
}

}

