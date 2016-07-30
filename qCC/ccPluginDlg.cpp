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

ccPluginDlg::ccPluginDlg(	const QStringList &paths,
							const tPluginInfoList &pluginInfoList,
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
	
	// because we are displaying these to the user, make the paths use native separators
	QStringList	nativePaths;
	
	for ( const QString &path : paths )
	{
		nativePaths += QDir::toNativeSeparators( path );
	}
	
	addPluginInfo(nativePaths, pluginInfoList);
}

void ccPluginDlg::addPluginInfo( const QStringList &paths, const tPluginInfoList &pluginInfoList )
{
	label->setText(QString(	"Paths Searched:\n%1" ).arg(paths.join( "\n" )));
	
	for ( const tPluginInfo &info : pluginInfoList )
	{
		if (info.filename.isEmpty())
		{
			//static plugins have no associated filename
			populateTreeWidget(info.qObject, QString("%1 (Static Plugin)").arg(info.object->getName()));
		}
		else
		{
			QFileInfo fileInfo( info.filename );
			populateTreeWidget( info.qObject, fileInfo.fileName(), fileInfo.filePath() );
		}
	}
}

void ccPluginDlg::populateTreeWidget(QObject *plugin, const QString &name, const QString &path/*=QString()*/)
{
	QTreeWidgetItem *pluginItem = new QTreeWidgetItem(treeWidget);
	pluginItem->setText(0, name);
	treeWidget->setItemExpanded(pluginItem, true);

	QFont boldFont = pluginItem->font(0);
	boldFont.setBold(true);
	pluginItem->setFont(0, boldFont);

	if ( !path.isEmpty() )
	{
		pluginItem->setToolTip( 0, path );
	}
	
	if ( plugin == nullptr )
		return;
	
	ccPluginInterface *ccPlugin = qobject_cast<ccPluginInterface*>(plugin);
	
	if ( ccPlugin == nullptr )
		return;

	QStringList features;
	features += QString("name: %1").arg(ccPlugin->getName());
	addItems(pluginItem, "CloudCompare Plugin", features);
}

void ccPluginDlg::addItems(	QTreeWidgetItem *pluginItem,
							const char *interfaceName,
							const QStringList &features)
{
	QTreeWidgetItem *interfaceItem = new QTreeWidgetItem(pluginItem);
	interfaceItem->setText(0, interfaceName);
	interfaceItem->setIcon(0, interfaceIcon);

	for ( QString feature : features )
	{
		if (feature.endsWith("..."))
			feature.chop(3);
		
		QTreeWidgetItem *featureItem = new QTreeWidgetItem(interfaceItem);
		featureItem->setText(0, feature);
		featureItem->setIcon(0, featureIcon);
	}
}
