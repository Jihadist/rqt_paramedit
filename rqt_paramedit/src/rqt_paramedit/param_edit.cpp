#include "rqt_paramedit/param_edit.h"
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>
#include <QMessageBox>
#include "param_root_chooser.h"
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QWidget>

#include <QVariant>
#include <QList>

PLUGINLIB_EXPORT_CLASS(rqt_paramedit::ParamEdit, rqt_gui_cpp::Plugin)

namespace rqt_paramedit
{
ParamEdit::ParamEdit()
  : _treeView(NULL)
  , _model(NULL)
  , _delegate(NULL)
  , _widget(NULL)
  , _updateButton(NULL)
  , _refButton(NULL)
  , _mainLayout(new QVBoxLayout)
  , _horLayout(new QHBoxLayout)

  , _updateLabel(NULL)
{
#ifdef _DEBUG
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif
  setObjectName("ParamEdit");
  _treeView = new QTreeView();
  _widget = new QWidget();

  _updateButton = new QPushButton();
  _updateButton->setText("Update param");
  _refButton = new QPushButton();
  _refButton->setText("Refresh param");

  //_updateLabel = new QLabel("Here you can see called service");
  _updateLabel = new QColorLabel();

  _widget->setLayout(_mainLayout);
  _mainLayout->addLayout(_horLayout);
  _mainLayout->addWidget(_treeView);

  _horLayout->addWidget(_refButton);
  _horLayout->addWidget(_updateButton);
  _horLayout->addWidget(_updateLabel);

  connect(_refButton, SIGNAL(clicked()), this, SLOT(handleRefButton()));
  connect(_updateButton, SIGNAL(clicked()), this, SLOT(handUpdButton()));
  connect(&_services, &Services::serviceCalled, this, &ParamEdit::updateLabelText);
  connect(&_services, &Services::serviceFailed, this, &ParamEdit::failedMsg);
}

void ParamEdit::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  context.addWidget(_widget);

  _paramRoot = "/";

  _delegate = new XmlRpcItemDelegate(_treeView);
  _treeView->setItemDelegate(_delegate);

  reload();
}

void ParamEdit::reload()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  if (!_nh.getParam(_paramRoot, _xmlrpc))
  {
    ROS_ERROR("Could not get parameters at: \"%s\"", _paramRoot.c_str());
    QMessageBox::critical(_treeView, "Error loading parameters",
                          QString("Could not get parameters at: \"%1\"").arg(_paramRoot.c_str()));

    return;
  }
  else if (_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("Requested parameter at \"%s\" has non-struct type. Only structs are supported, not single parameters.",
              _paramRoot.c_str());
    QMessageBox::critical(_treeView, "Error loading parameters",
                          QString("Requested parameter at \"%1\" has non-struct type. Only structs are supported, not "
                                  "single parameters.\nMaybe, you want to choose the parent containing the selected "
                                  "parameter.")
                              .arg(_paramRoot.c_str()));

    // we have now loaded a parameter that can't be interpreted by the model.
    // Try to fix that inconsistency somehow by loading from '/' until the user clears this up
    _paramRoot = "/";
    _nh.getParam(_paramRoot, _xmlrpc);
  }

  delete _model;
  _services.raw()->clear();
  _model = new xmlRpcModelWalk(&_xmlrpc, _paramRoot, &_nh);

  _services.loadData();
  _services.findServices();
  _treeView->setModel(_model);
  _model->walk(*_services.raw());
  _services.createButtons(_treeView);
}

void ParamEdit::handleRefButton()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  reload();
}

void ParamEdit::handUpdButton()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  _services.callServices();
}

void ParamEdit::updateLabelText(std::string& s)
{
  _updateLabel->changeBackgroundColorWithTimer();
}

void ParamEdit::failedMsg(std::string& s)
{
  QMessageBox::critical(nullptr, "rqt_pramedir", QString::fromStdString(s));
}

void ParamEdit::shutdownPlugin()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
}

void ParamEdit::saveSettings(qt_gui_cpp::Settings& /*global_settings*/,
                             qt_gui_cpp::Settings& perspective_settings) const
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  perspective_settings.setValue("param_root", _paramRoot.c_str());
}

void ParamEdit::restoreSettings(const qt_gui_cpp::Settings& /*global_settings*/,
                                const qt_gui_cpp::Settings& perspective_settings)
{
  ROS_DEBUG(__PRETTY_FUNCTION__);
  _paramRoot = qPrintable(perspective_settings.value("param_root", "/").toString());
  reload();
}

void ParamEdit::triggerConfiguration()
{
  ROS_DEBUG(__PRETTY_FUNCTION__);

  ParamRootChooser dialog;
  if (dialog.exec() == QDialog::Accepted)
  {
    if (dialog.selectedParamRoot().empty())
    {
      ROS_ERROR("ParamRootChooser Accepted, but no valid parameter chosen.");
    }
    else
    {
      _paramRoot = dialog.selectedParamRoot();
      reload();
    }
  }
}

}  // namespace rqt_paramedit
