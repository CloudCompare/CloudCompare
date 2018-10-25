#ifndef CCTRANSLATIONMANAGER_H
#define CCTRANSLATIONMANAGER_H

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

#include <QMenu>
#include <QPair>
#include <QVector>


class ccTranslationManager : public QObject
{
	Q_OBJECT
	
public:
	ccTranslationManager( QObject *parent );
	~ccTranslationManager() override = default;
	
	void populateMenu( QMenu *menu );
	
	static const QString languagePref();
	
private:
	using TranslationInfo = QPair<QString, QString>;
	using LanguageList = QVector<TranslationInfo>;
	
	LanguageList availableLanguages( const QString &appName );
	
	void setLanguagePref( const QString &languageCode );
};

#endif //CCTRANSLATIONMANAGER_H
