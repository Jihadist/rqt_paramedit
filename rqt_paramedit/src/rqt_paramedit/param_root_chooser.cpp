#include "param_root_chooser.h"
#include <ros/ros.h>
#include <ros/master.h>
#include <algorithm>
#include <iterator>
#include <set>

ParamRootChooser::ParamRootChooser(QWidget* parent) : QDialog(parent)
{
    ui.setupUi(this);

    // change paramRoot from list/combo/tree (to get somehow) and reload
    std::vector<std::string> allParams = getParamNamesFromMaster();
    std::vector<std::string> roots = getParameterRoots(allParams);

    for(std::vector<std::string>::const_iterator it = roots.begin(); it != roots.end(); it++) {
        ui.paramRootCombo->addItem(it->c_str());
    }
}

std::string ParamRootChooser::selectedParamRoot() const
{
    return _selectedParam;
}

void ParamRootChooser::on_paramRootCombo_currentIndexChanged(const QString & text)
{
    _selectedParam = qPrintable(text);
}

std::vector<std::string> ParamRootChooser::getParamNamesFromMaster() const
{
    XmlRpc::XmlRpcValue params, response, payload;
    params[0] = ros::this_node::getName();
    if(ros::master::execute("getParamNames", params, response, payload, true)) {
        std::vector<std::string> ret;
        ret.reserve(response[2].size());
for(int i = 0; i < response[2].size(); i++) {
            ret.push_back(response[2][i]);
        }

        return ret;
    } 

    ROS_ERROR("Failed to getParamNames from master.");
    return std::vector<std::string>();
}

std::vector<std::string> ParamRootChooser::getParameterRoots(const std::vector<std::string>& paramNames) const
{
    std::set<std::string> roots;
    roots.insert("/");

    // split each param like: /test/param/a into single parts
    // and add each prefix to the param list
    // -> /test, /test/param
    for(auto it = paramNames.begin(); it != paramNames.end(); it++) {
        QString param = it->c_str();
        QStringList parts = param.split("/", QString::SkipEmptyParts);
        if(parts.size() < 2)
            continue;
        QString part = "";
        for(int i = 0; i < parts.size() - 1; i++) {
            part = part + "/" + parts.at(i);
            roots.insert(qPrintable(part));
        }
    }

    std::vector<std::string> ret;
    std::copy(roots.begin(), roots.end(), back_inserter(ret));

    return ret;
}

