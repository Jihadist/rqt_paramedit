#include "rqt_paramedit/param_edit.h"
#include <pluginlib/class_list_macros.h>
#include <QMessageBox>
#include "param_root_chooser.h"
 
PLUGINLIB_EXPORT_CLASS(rqt_paramedit::ParamEdit, rqt_gui_cpp::Plugin)

namespace rqt_paramedit
{

ParamEdit::ParamEdit() : _treeView(NULL), _model(NULL), _delegate(NULL)
{
    setObjectName("ParamEdit");
}

void ParamEdit::initPlugin(qt_gui_cpp::PluginContext& context)
{
    _treeView = new QTreeView();
    context.addWidget(_treeView);

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

