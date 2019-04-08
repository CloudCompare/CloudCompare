#ifndef CCPLUGININFODLG_H
#define CCPLUGININFODLG_H
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include <QDialog>
#include <QList>

class QModelIndex;
class QSortFilterProxyModel;
class QStandardItem;
class QStandardItemModel;

class ccPluginInterface;

namespace Ui {
	class ccPluginInfoDlg;
}

class ccPluginInfoDlg : public QDialog
{
	Q_OBJECT
	
public:
	explicit ccPluginInfoDlg( QWidget *parent = nullptr );
	~ccPluginInfoDlg() override;
	
	void	setPluginPaths( const QStringList &pluginPaths );
	void	setPluginList( const QList<ccPluginInterface *> &pluginList );
	
private:
	enum {
		PLUGIN_PTR = Qt::UserRole + 1
	};
	
	const ccPluginInterface* pluginFromItemData( const QStandardItem* item ) const;
	
	void	selectionChanged( const QModelIndex &current, const QModelIndex &previous );
	void	itemChanged( QStandardItem* item );
	
	void	updatePluginInfo( const ccPluginInterface *plugin );
	
	Ui::ccPluginInfoDlg *m_UI;
	
	QSortFilterProxyModel	*m_ProxyModel;
	QStandardItemModel		*m_ItemModel;
};

#endif