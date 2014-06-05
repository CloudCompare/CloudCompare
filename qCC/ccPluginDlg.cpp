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

#include "ccPluginDlg.h"

//qCC_plugins
#include <ccPluginInterface.h>

//Qt
#include <QPluginLoader>
#include <QStringList>
#include <QDir>
#include <QLabel>
#include <QGridLayout>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>

ccPluginDlg::ccPluginDlg(	const QString &path,
							const QStringList &fileNames,
							QWidget *parent)
	: QDialog(parent)
	, label(new QLabel)
	, treeWidget(new QTreeWidget)
	, okButton(new QPushButton(QString("OK")))
{
	treeWidget->setAlternatingRowColors(false);
	treeWidget->setSelectionMode(QAbstractItemView::NoSelection);
	treeWidget->setColumnCount(1);
	treeWidget->header()->hide();

	okButton->setDefault(true);

	connect(okButton, SIGNAL(clicked()), this, SLOT(close()));

	QGridLayout *mainLayout = new QGridLayout;
	mainLayout->setColumnStretch(0, 1);
	mainLayout->setColumnStretch(2, 1);
	mainLayout->addWidget(label, 0, 0, 1, 3);
	mainLayout->addWidget(treeWidget, 1, 0, 1, 3);
	mainLayout->addWidget(okButton, 2, 1);
	setLayout(mainLayout);

	interfaceIcon.addPixmap(style()->standardPixmap(QStyle::SP_DirOpenIcon),QIcon::Normal, QIcon::On);
	interfaceIcon.addPixmap(style()->standardPixmap(QStyle::SP_DirClosedIcon),QIcon::Normal, QIcon::Off);
	featureIcon.addPixmap(style()->standardPixmap(QStyle::SP_FileIcon));

	setWindowTitle(QString("Plugin Information"));
	findPlugins(path, fileNames);
}

void ccPluginDlg::findPlugins(	const QString &path,
								const QStringList &fileNames)
{
	label->setText(QString(	"Found plugins\n"
							"(looked in %1):")
							.arg(QDir::toNativeSeparators(path)));

	const QDir dir(path);

	foreach (QObject *plugin, QPluginLoader::staticInstances())
	{
		populateTreeWidget(plugin, QString("%1 (Static Plugin)")
			.arg(plugin->metaObject()->className()));
	}

	foreach (QString filename, fileNames)
	{
		QPluginLoader loader(dir.absoluteFilePath(filename));
		QObject *plugin = loader.instance();
		if (plugin)
			populateTreeWidget(plugin, filename);
	}
}

void ccPluginDlg::populateTreeWidget(QObject *plugin, const QString &text)
{
	QTreeWidgetItem *pluginItem = new QTreeWidgetItem(treeWidget);
	pluginItem->setText(0, text);
	treeWidget->setItemExpanded(pluginItem, true);

	QFont boldFont = pluginItem->font(0);
	boldFont.setBold(true);
	pluginItem->setFont(0, boldFont);

	if (plugin)
	{
		ccPluginInterface *ccPlugin = qobject_cast<ccPluginInterface*>(plugin);
		if (ccPlugin)
		{
			QStringList features;
			features += QString("name: %1").arg(ccPlugin->getName());
			addItems(pluginItem, "CloudCompare Plugin", features);
		}
	}
}

void ccPluginDlg::addItems(	QTreeWidgetItem *pluginItem,
							const char *interfaceName,
							const QStringList &features)
{
	QTreeWidgetItem *interfaceItem = new QTreeWidgetItem(pluginItem);
	interfaceItem->setText(0, interfaceName);
	interfaceItem->setIcon(0, interfaceIcon);

	foreach (QString feature, features) {
		if (feature.endsWith("..."))
			feature.chop(3);
		QTreeWidgetItem *featureItem = new QTreeWidgetItem(interfaceItem);
		featureItem->setText(0, feature);
		featureItem->setIcon(0, featureIcon);
	}
}
