#pragma once

// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                   COPYRIGHT: CloudCompare project                      #
// #                                                                        #
// ##########################################################################

#include "ui_shorcutSettings.h"
#include "ui_shortcutEditDialog.h"

//! Widget that captures key sequences to be able to edit a shortcut assigned to
//! an action
class ccShortcutEditDialog final : public QDialog
{
	Q_OBJECT

  public:
	explicit ccShortcutEditDialog(QWidget* parent = nullptr);

	QKeySequence keySequence() const;

	void setKeySequence(const QKeySequence& sequence) const;

	int exec() override;

  private:
	Ui_ShortcutEditDialog* m_ui;
};

//! Shortcut edit dialog
//!
//! List shortcuts for known actions, and allows to edit them
//! Saves to QSettings on each edit
class ccShortcutDialog final : public QDialog
{
	Q_OBJECT
  public:
	explicit ccShortcutDialog(const QList<QAction*>& actions, QWidget* parent = nullptr);

	void restoreShortcutsFromQSettings() const;

  private:
	const QAction* checkConflict(const QKeySequence& sequence) const;
	void           handleDoubleClick(QTableWidgetItem* item);

	Ui_ShortcutDialog*    m_ui;
	ccShortcutEditDialog* m_editDialog;
};
