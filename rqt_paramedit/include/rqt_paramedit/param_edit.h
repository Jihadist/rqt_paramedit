#ifndef RQT_PARAMEDIT_H
#define RQT_PARAMEDIT_H

#include <rqt_gui_cpp/plugin.h>
#include <qt_paramedit/xmlRpcModel.h>
#include <qt_paramedit/xmlRpcItemDelegate.h>

#include <QWidget>
#include <QTreeView>
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <vector>
#include <string>


namespace rqt_paramedit
{

class ParamEdit : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

    public:
        ParamEdit();
 
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);

        virtual void shutdownPlugin();

        virtual void saveSettings(qt_gui_cpp::Settings& global_settings,
                qt_gui_cpp::Settings& perspective_settings) const;

        virtual void restoreSettings(const qt_gui_cpp::Settings& global_settings,
                const qt_gui_cpp::Settings& perspective_settings);

        virtual bool hasConfiguration() const
        {
            return true;
        }

        virtual void triggerConfiguration();

    protected:
        void reload();
        bool set_new_param();
        bool set_new_param(const std::string &s);


    protected:
        QTreeView* _treeView;
        QWidget* _widget;

        QPushButton* _updateButton;
        QPushButton* _refButton;

        QVBoxLayout *_mainLayout;
        QHBoxLayout *_horLayout;


        ros::NodeHandle _nh;
        std::string _paramRoot;
        XmlRpc::XmlRpcValue _xmlrpc;

        XmlRpcModel* _model;
        XmlRpcItemDelegate* _delegate;


    private slots:
      void handleRefButton();
      void handUpdButton();
};

}

#endif

