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
#include <QGlobalStatic>
#include <QMessageBox>
#include <QRegularExpression>
#include <QSettings>
#include <QTranslator>

#include "ccApplicationBase.h"
#include "ccTranslationManager.h"


class _ccTranslationManager : public ccTranslationManager {};	// trick for Q_GLOBAL_STATIC to access constructor
Q_GLOBAL_STATIC( _ccTranslationManager, sTranslationmanager )


ccTranslationManager &ccTranslationManager::get()
{
	return *sTranslationmanager;
}

void ccTranslationManager::registerTranslatorFile( const QString &prefix, const QString &path )
{
	mTranslatorFileInfo.append( { prefix, path } );
}

void ccTranslationManager::loadTranslations()
{
	const QLocale	locale( languagePref() );
	
	const auto	&info = mTranslatorFileInfo;
	
	for ( const auto &fileInfo : info )
	{
		auto translator = new QTranslator( ccApp );
	 
		bool loaded = translator->load( locale, fileInfo.prefix, QStringLiteral( "_" ), fileInfo.path );
	 
		if ( loaded )
		{ 
		   ccApp->installTranslator( translator );
		}
		else
		{
			delete translator;
		}
	}
}

void ccTranslationManager::populateMenu( QMenu *menu, const QString &pathToTranslationFiles )
{
	const LanguageList	cList = availableLanguages( QStringLiteral( "CloudCompare" ), pathToTranslationFiles );
	
	QActionGroup	*group = new QActionGroup( menu );
	
	group->setExclusive( true );
	
	QAction	*action = group->addAction( tr( "No Translation (English)" ) );
	
	action->setCheckable( true );
	action->setChecked( true );
	
	connect( action, &QAction::triggered, this, [this] ()
	{
		setLanguagePref( QStringLiteral( "en" ) );
	} );
	
	QAction	*separator = new QAction( group );
	separator->setSeparator( true );
	
	const QString	currentLanguage = languagePref();
	
	for ( auto &langInfo : cList )
	{
		action = group->addAction( langInfo.second );
	
		action->setCheckable( true );
		
		if ( currentLanguage == langInfo.first )
		{
			action->setChecked( true );
		}
		
		connect( action, &QAction::triggered, this, [=] ()
		{
			setLanguagePref( langInfo.first );
		} );
	}
	
	menu->addActions( group->actions() );
}

const QString ccTranslationManager::languagePref()
{
	QSettings settings;
	
	settings.beginGroup( "Translation" );
	
	const QString	langCode = settings.value( QStringLiteral( "Language" ) ).toString();
	
	settings.endGroup();
	
	return langCode;
}

ccTranslationManager::LanguageList ccTranslationManager::availableLanguages( const QString &appName, const QString &pathToTranslationFiles )
{
	LanguageList theList;

	QDir dir( pathToTranslationFiles );

	const QString     cFilter = QStringLiteral( "%1_*.qm" ).arg( appName );
	const QStringList cFileNames = dir.entryList( { cFilter } );

	QRegularExpression   regExp( QStringLiteral( "%1_(?<langCode>.{2}).*.qm" ).arg( appName ) );

	regExp.optimize();

	for ( const auto &cFileName : cFileNames )
	{
	   QRegularExpressionMatch match = regExp.match( cFileName );

	   if ( !match.hasMatch() )
	   {
		  continue;
	   }

	   const QString  cLangCode( match.captured( QStringLiteral( "langCode" ) ) );

	   QString  language( QLocale( cLangCode ).nativeLanguageName() );

	   if ( language.isEmpty() )
	   {
		  qWarning() << "Language not found for translation file" << cFileName;
		  continue;
	   }

	   language = language.at( 0 ).toUpper() + language.mid( 1 );

	   theList += { cLangCode, language };
	}

	return theList;
}

void ccTranslationManager::setLanguagePref( const QString &languageCode )
{
	if ( languageCode == languagePref() )
	{
		return;
	}
	
	QSettings settings;
	
	settings.beginGroup( "Translation" );
	
	settings.setValue( QStringLiteral( "Language" ), languageCode );
	
	settings.endGroup();
	
	QMessageBox::information( nullptr, 
							  tr( "Language Change" ),
							  tr( "Language change will take effect when CloudCompare is restarted" ) );
}
