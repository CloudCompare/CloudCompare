//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_PLUGIN_DIALOG_HEADER
#define CC_PLUGIN_DIALOG_HEADER

#include <QDialog>
#include <QIcon>

#include <ccPluginInfo.h>

class QLabel;
class QPushButton;
class QStringList;
class QTreeWidget;
class QTreeWidgetItem;

//! Dialog to display the loaded plugin list
class ccPluginDlg : public QDialog
{
	Q_OBJECT

public:
	ccPluginDlg(const QStringList &paths,
				const tPluginInfoList &pluginInfoList,
				QWidget *parent = 0);

protected:
	void addPluginInfo(const QStringList &paths, const tPluginInfoList &pluginInfoList);
	void populateTreeWidget(QObject *plugin, const QString &name, const QString &path = QString());
	void addItems(	QTreeWidgetItem *pluginItem,
					const char *interfaceName,
					const QStringList &features);

	QLabel *label;
	QTreeWidget *treeWidget;
	QPushButton *okButton;
	QIcon interfaceIcon;
	QIcon featureIcon;
};

#endif
