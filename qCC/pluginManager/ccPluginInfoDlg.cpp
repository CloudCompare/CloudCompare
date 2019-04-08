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

#include <QDebug>
#include <QDir>
#include <QSortFilterProxyModel>
#include <QStandardItemModel>

#include "ccPluginInfoDlg.h"
#include "ccPluginManager.h"
#include "ccStdPluginInterface.h"

#include "ui_ccPluginInfoDlg.h"


static QString sFormatReferenceList( const ccPluginInterface::ReferenceList &list )
{
	const QString linkFormat( " <a href=\"%1\" style=\"text-decoration:none\">&#x1F517;</a>" );
	QString	formattedText;
	int referenceNum = 1;
	
	for ( const ccPluginInterface::Reference &reference : list )
	{
		formattedText += QStringLiteral( "%1. " ).arg( QString::number( referenceNum++ ) );
		
		formattedText += reference.article;
		
		if ( !reference.url.isEmpty() )
		{
			formattedText += linkFormat.arg( reference.url );
		}
		
		formattedText += QStringLiteral( "<br/>" );
	}
	
	return formattedText;
}

static QString sFormatContactList( const ccPluginInterface::ContactList &list, const QString &pluginName )
{
	const QString emailFormat( "&lt;<a href=\"mailto:%1?Subject=CloudCompare %2\">%1</a>&gt;" );
	QString	formattedText;
	
	for ( const ccPluginInterface::Contact &contact : list )
	{
		formattedText += contact.name;
		
		if ( !contact.email.isEmpty() )
		{
			formattedText += emailFormat.arg( contact.email, pluginName );
		}
		
		formattedText += QStringLiteral( "<br/>" );
	}
	
	return formattedText;
}

namespace
{
	class _Icons
	{
	public:
		static QIcon   sGetIcon( CC_PLUGIN_TYPE inPluginType )
		{
			if ( sIconMap.empty() )
			{
				_init();
			}
			
			return sIconMap[inPluginType];
		}
		
	private:
		static void	_init()
		{
			if ( !sIconMap.empty() )
			{
				return;
			}
			
			sIconMap[CC_STD_PLUGIN] = QIcon( ":/CC/pluginManager/images/std_plugin.png" );
			sIconMap[CC_GL_FILTER_PLUGIN] = QIcon( ":/CC/pluginManager/images/gl_plugin.png" );
			sIconMap[CC_IO_FILTER_PLUGIN] = QIcon( ":/CC/pluginManager/images/io_plugin.png" );
		}
		
		static QMap<CC_PLUGIN_TYPE, QIcon>	sIconMap;
	};
	
	QMap<CC_PLUGIN_TYPE, QIcon>	_Icons::sIconMap;
}

ccPluginInfoDlg::ccPluginInfoDlg( QWidget *parent ) :
	QDialog( parent )
  , m_UI( new Ui::ccPluginInfoDlg )
  , m_ProxyModel( new QSortFilterProxyModel( this ) )
  , m_ItemModel( new QStandardItemModel( this ) )
{
	m_UI->setupUi( this );
	
	setWindowTitle( tr("About Plugins" ) );
	
	m_UI->mWarningLabel->setText( tr( "Enabling/disabling plugins will take effect next time you run %1" ).arg( QApplication::applicationName() ) );
	m_UI->mWarningLabel->setStyleSheet( QStringLiteral( "QLabel { background-color : #FFFF99; color : black; }" ) );
	m_UI->mWarningLabel->hide();
	
	m_UI->mSearchLineEdit->setStyleSheet( "QLineEdit, QLineEdit:focus { border: none; }" );
	m_UI->mSearchLineEdit->setAttribute( Qt::WA_MacShowFocusRect, false );
				
	m_ProxyModel->setFilterCaseSensitivity( Qt::CaseInsensitive );
	m_ProxyModel->setSourceModel( m_ItemModel );
	
	m_UI->mPluginListView->setModel( m_ProxyModel );
	m_UI->mPluginListView->setFocus();
	
	connect( m_UI->mSearchLineEdit, &QLineEdit::textEdited, m_ProxyModel, &QSortFilterProxyModel::setFilterFixedString );
	
	connect( m_UI->mPluginListView->selectionModel(), &QItemSelectionModel::currentChanged, 
			 this, &ccPluginInfoDlg::selectionChanged );
}

ccPluginInfoDlg::~ccPluginInfoDlg()
{
	delete m_UI;
}

void ccPluginInfoDlg::setPluginPaths( const QStringList &pluginPaths )
{
	QString	paths;
	
	for ( const QString &path : pluginPaths )
	{
		paths += QDir::toNativeSeparators( path );
		paths += QStringLiteral( "\n" );
	}
	
	m_UI->mPluginPathTextEdit->setText( paths );
}

void ccPluginInfoDlg::setPluginList( const QList<ccPluginInterface *> &pluginList )
{
	m_ItemModel->clear();
	m_ItemModel->setRowCount( pluginList.count() );
	m_ItemModel->setColumnCount( 1 );
	
	int	row = 0;
	
	for ( const ccPluginInterface *plugin : pluginList )
	{
		auto name = plugin->getName();
		auto tooltip = tr( "%1 Plugin" ).arg( plugin->getName() );
		
		if ( plugin->isCore() )
		{
			tooltip += tr( " (core)" );
		}
		else
		{
			name += " 👽";
			tooltip += tr( " (3rd Party)" );
		}
		
		QStandardItem *item = new QStandardItem( name );
				
		item->setCheckable( true );
		
		if ( ccPluginManager::get().isEnabled( plugin ) )
		{
			item->setCheckState( Qt::Checked );
		}
		
		item->setData( QVariant::fromValue( plugin ), PLUGIN_PTR );		
		item->setIcon( _Icons::sGetIcon( plugin->getType() ) );
		item->setToolTip( tooltip );
		
		m_ItemModel->setItem( row, 0, item );
		
		++row;
	}
	
	if ( !pluginList.empty() )
	{
		m_ItemModel->sort( 0 );
		
		QModelIndex	index = m_ItemModel->index( 0, 0 );
		
		m_UI->mPluginListView->setCurrentIndex( index );
	}
	
	connect( m_ItemModel, &QStandardItemModel::itemChanged, this, &ccPluginInfoDlg::itemChanged );
}

const ccPluginInterface *ccPluginInfoDlg::pluginFromItemData( const QStandardItem *item ) const
{
	return item->data( PLUGIN_PTR ).value<const ccPluginInterface*>();;
}

void ccPluginInfoDlg::selectionChanged( const QModelIndex &current, const QModelIndex &previous )
{
	Q_UNUSED( previous );
	
	auto sourceItem = m_ProxyModel->mapToSource( current );
	auto item = m_ItemModel->itemFromIndex( sourceItem );
	
	if ( item == nullptr )
	{
		// This happens if we are filtering and there are no results
		updatePluginInfo( nullptr );
		return;
	}
	
	auto plugin = pluginFromItemData( item );
	
	updatePluginInfo( plugin );	
}

void ccPluginInfoDlg::itemChanged( QStandardItem *item )
{
	bool checked = item->checkState() == Qt::Checked;
	auto plugin = pluginFromItemData( item );
	
	if ( plugin != nullptr )
	{
		ccPluginManager::get().setPluginEnabled( plugin, checked );
		
		if ( m_UI->mWarningLabel->isHidden() )
		{
			ccLog::Warning( m_UI->mWarningLabel->text() );
			
			m_UI->mWarningLabel->show();
		}
	}
}

void ccPluginInfoDlg::updatePluginInfo( const ccPluginInterface *plugin )
{
	if ( plugin == nullptr )
	{
		m_UI->mIcon->setPixmap( QPixmap() );
		m_UI->mNameLabel->setText( tr( "(No plugin selected)" ) );
		m_UI->mDescriptionTextEdit->clear();
		m_UI->mReferencesTextBrowser->clear();
		m_UI->mAuthorsTextBrowser->clear();
		m_UI->mMaintainerTextBrowser->clear();
		return;
	}
	
	const QSize iconSize( 64, 64 );
	
	QPixmap	iconPixmap;
	
	if ( !plugin->getIcon().isNull() )
	{
		iconPixmap = plugin->getIcon().pixmap( iconSize );
	}
	
	switch ( plugin->getType() )
	{
		case CC_STD_PLUGIN:
		{
			if ( iconPixmap.isNull() )
			{
				iconPixmap = QPixmap( ":/CC/pluginManager/images/std_plugin.png" ).scaled( iconSize );
			}
			
			m_UI->mPluginTypeLabel->clear();
			break;				
		}
			
		case CC_GL_FILTER_PLUGIN:
		{
			if ( iconPixmap.isNull() )
			{
				iconPixmap = QPixmap( ":/CC/pluginManager/images/gl_plugin.png" ).scaled( iconSize );
			}
			
			m_UI->mPluginTypeLabel->setText( tr( "GL Shader" ) );
			break;
		}
			
		case CC_IO_FILTER_PLUGIN:
		{
			if ( iconPixmap.isNull() )
			{
				iconPixmap = QPixmap( ":/CC/pluginManager/images/io_plugin.png" ).scaled( iconSize );
			}
			
			m_UI->mPluginTypeLabel->setText( tr( "I/O" ) );
			break;
		}
	}
	
	m_UI->mIcon->setPixmap( iconPixmap );
	
	m_UI->mNameLabel->setText( plugin->getName() );
	m_UI->mDescriptionTextEdit->setHtml( plugin->getDescription() );
	
	const QString referenceText = sFormatReferenceList( plugin->getReferences() );
	
	if ( !referenceText.isEmpty() )
	{
		m_UI->mReferencesTextBrowser->setHtml( referenceText );
		m_UI->mReferencesLabel->show();
		m_UI->mReferencesTextBrowser->show();
	}
	else
	{
		m_UI->mReferencesLabel->hide();
		m_UI->mReferencesTextBrowser->hide();
		m_UI->mReferencesTextBrowser->clear();
	}
	
	const QString authorsText = sFormatContactList( plugin->getAuthors(), plugin->getName() );
	
	if ( !authorsText.isEmpty() )
	{
		m_UI->mAuthorsTextBrowser->setHtml( authorsText );
	}
	else
	{
		m_UI->mAuthorsTextBrowser->clear();
	}
	
	const QString maintainersText = sFormatContactList( plugin->getMaintainers(), plugin->getName() );
	
	if ( !maintainersText.isEmpty() )
	{
		m_UI->mMaintainerTextBrowser->setHtml( maintainersText );
	}
	else
	{
		m_UI->mMaintainerTextBrowser->clear();
	}
}