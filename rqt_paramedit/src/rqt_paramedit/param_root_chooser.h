#ifndef PARAM_ROOT_CHOOSER_h
#define PARAM_ROOT_CHOOSER_h

#include <QDialog>
#include <string>
#include <vector>
#include "ui_param_root_chooser.h"

class ParamRootChooser : public QDialog
{
  Q_OBJECT

public:
  ParamRootChooser(QWidget* parent = 0);

  std::string selectedParamRoot() const;

private Q_SLOTS:
  void on_paramRootCombo_currentIndexChanged(const QString& text);

private:
  /// Retrieves the names of all parameters from master server.
  std::vector<std::string> getParamNamesFromMaster() const;

  /// Extracts a list of all possible parameter roots from a list of parameter names.
  /**
   * The assumption is that parameter names in the deepest level are single parameters
   * and not dictionaries.
   *
   * E.g.: /test/param/a -> [/test, /test/param]
   */
  std::vector<std::string> getParameterRoots(const std::vector<std::string>& paramNames) const;

private:
  Ui::ParamRootChooserDialog ui;

  std::string _selectedParam;
};

#endif
