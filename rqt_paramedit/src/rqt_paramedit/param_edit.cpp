#include "rqt_paramedit/param_edit.h"
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>
#include <QMessageBox>
#include "param_root_chooser.h"
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>

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

  _widget->setLayout(_mainLayout);
  _mainLayout->addLayout(_horLayout);
  _mainLayout->addWidget(_treeView);

  _horLayout->addWidget(_refButton);
  _horLayout->addWidget(_updateButton);

  connect(_refButton, SIGNAL(clicked()), this, SLOT(handleRefButton()));
  connect(_updateButton, SIGNAL(clicked()), this, SLOT(handUpdButton()));
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
  //_model=new xmlRpcModelWalk
  //_model = dynamic_cast<xmlRpcModelWalk*>(new XmlRpcModel(&_xmlrpc, _paramRoot, &_nh));
  // _model->XmlRpcModel::XmlRpcModel(&_xmlrpc,_paramRoot,&_nh);
  // _model= new XmlRpcModel(&_xmlrpc,_paramRoot,&_nh);
  _model = new xmlRpcModelWalk(&_xmlrpc, _paramRoot, &_nh);
  _services.loadData();
  // std::string s;
  _services.findServices();
  _model->walk(*_services.raw());
  // int rows = _model->rowCount();
  // int columns = _model->columnCount();
  // std::cout << _model->rowCount() << std::endl;
  // std::cout << _model->columnCount() << std::endl;
  // Это выводит rosdistro
  // std::cout << _model->index(0, 0).data().toString().toStdString() << std::endl;
  // std::cout << _model->headerData(0, Qt::Horizontal).toString().toStdString() << std::endl;
  // if (_model->insertRow(5))
  // // std::cout << "inserted 5";
  // if (_model->insertColumns(2, 3))
  //  std::cout << "inserted 6 ";
  // if (_model->insertRow(2))
  //  std::cout << "inserted2";
  //_model->data(_model->index(0,0),1);
  //_model->setData(_model->index(0, 0), "test");
  _treeView->setModel(_model);
  // for (auto i = 0; i != rows; ++i)
  //{
  //_model->index(i,0);
  // std::cout << _model->index(i, 0).data().toString().toStdString() << std::endl;
  // if (_model->hasChildren(_model->index(i, 0)))
  //{
  // std::cout << "Children " << _model->index(i, 0, _model->index(i, 0)).row() << std::endl;
  //_model->insertRow(i + 1, _model->index(i, 0));
  //_model->insertRow(i + 2, _model->index(i, 0));
  //_model->insertRow(i + -2, _model->index(i, 0));
  // _treeView->setIndexWidget(_model->index(i, 0), new QPushButton);
  //}
  // QList<QVariant*> chldrn = _model->findChildren<QVariant*>();
  // if (!children.empty())
  // for (const auto& i : children)
  // std::cout << i->data();
  //}
  // std::cout << std::endl;
  // _model->createIndex(_model->rowCount(), 0);
  // std::cout << _model->rowCount() << std::endl;
  // std::cout << _model->columnCount() << std::endl;
  auto* it = _model;
  int cnt = 0;
  QModelIndex* index;
  auto chill = _treeView->findChild<QAbstractItemModel*>("joint");
  // if (chill->parent()->objectName() == "joint_position_controller1/")
  //{
  //  ROS_INFO("Params ok");
  //_model->index(0,1,chill->parent()->);
  //}
  //_model->persistentIndexList()
  //_treeView->findChild("joint");
  //  for(;cnt!=_model->rowCount();++cnt)
  //  {
  //      if(it->index(cnt,0).data().toString()=="ihand")
  //      {
  //          cnt=0;
  //          index=
  //         //it=dynamic_cast<XmlRpcModel *>(it->index(cnt,0).model());
  //      }
  //          else {
  //              ++cnt;
  //          }
  //  }
  //_model->std();
  _treeView->setIndexWidget(_model->index(3, 1), new QPushButton("update"));
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
