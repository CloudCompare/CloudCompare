//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_PLUGIN_DIALOG_HEADER
#define CC_PLUGIN_DIALOG_HEADER

#include <QDialog>
#include <QIcon>

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
	ccPluginDlg(const QString &path,
				const QStringList &fileNames,
				QWidget *parent = 0);

protected:
	void findPlugins(const QString &path, const QStringList &fileNames);
	void populateTreeWidget(QObject *plugin, const QString &text);
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
