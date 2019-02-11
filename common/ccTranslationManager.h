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
	static ccTranslationManager &get();
	
	~ccTranslationManager() override = default;
	
	/** Register a file prefix for translation files.
	 * The files should be named <prefix>_<lang>.ts where <lang> is the 2-letter ISO 639 
	 * language code in lowercase.
	 *  e.g. CloudCompare_fr.ts
	 * @param prefix The prefix of the file to register
	 * @param path The path to look for the files in
	 */
	void registerTranslatorFile( const QString &prefix, const QString &path );
	
	//! Using the translation file prefixes that were registered, load the actual translations
	void loadTranslations();
	
	//! Populate the menu with a list of languages found using files in 'pathToTranslationFiles'
	void populateMenu( QMenu *menu, const QString &pathToTranslationFiles );
	
protected:
	explicit ccTranslationManager() = default;

private:
	struct TranslatorFile {
		QString	prefix;
		QString path;
	};
	using TranslatorFileList = QVector<TranslatorFile>;
	
	using TranslationInfo = QPair<QString, QString>;
	using LanguageList = QVector<TranslationInfo>;
	
	const QString languagePref();
	
	//! Generate a list of available languages based on the files in the "translation" directory.
	LanguageList availableLanguages( const QString &appName, const QString &pathToTranslationFiles );
	
	void setLanguagePref( const QString &languageCode );
	
	TranslatorFileList	mTranslatorFileInfo;
};

#endif //CCTRANSLATIONMANAGER_H
