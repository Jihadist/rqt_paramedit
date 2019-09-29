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
    #ifdef _DEBUG
    ROS_INFO("ParamEdit::initPlugin()");
    #endif
    context.addWidget(_widget);

    _paramRoot = "/";

    _delegate = new XmlRpcItemDelegate(_treeView);
    _treeView->setItemDelegate(_delegate);

    reload();
}

void ParamEdit::reload()
{
    #ifdef _DEBUG
    ROS_INFO("ParamEdit::reload()");
    #endif
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
    _services.loadData();
    //std::string s;
    _services.findServices();
}


void ParamEdit::handleRefButton()
{
    #ifdef _DEBUG
    ROS_INFO("ParamEdit::handleRefButton()");
    #endif
    reload();
}

void ParamEdit::handUpdButton()
{
    #ifdef _DEBUG
    ROS_INFO("ParamEdit::handUpdButton()");
    #endif
    _services.callServices();
}

void ParamEdit::shutdownPlugin()
{
    #ifdef _DEBUG
    ROS_INFO("ParamEdit::shutdownPlugin()");
    #endif
}

void ParamEdit::saveSettings(qt_gui_cpp::Settings&  /*global_settings*/, qt_gui_cpp::Settings& perspective_settings) const
{
    #ifdef _DEBUG
    ROS_INFO("ParamEdit::saveSettings()");
    #endif
    perspective_settings.setValue("param_root", _paramRoot.c_str());
}

void ParamEdit::restoreSettings(const qt_gui_cpp::Settings&  /*global_settings*/, const qt_gui_cpp::Settings& perspective_settings)
{
    #ifdef _DEBUG
    ROS_INFO("ParamEdit::restoreSettings()");
    #endif
    _paramRoot = qPrintable(perspective_settings.value("param_root", "/").toString());
    reload();
}

void ParamEdit::triggerConfiguration()
{
    #ifdef _DEBUG
    ROS_INFO("ParamEdit::triggerConfiguration()");
    #endif
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

