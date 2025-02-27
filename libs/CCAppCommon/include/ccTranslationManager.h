#pragma once
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

#include "CCAppCommon.h"

//Qt
#include <QMenu>
#include <QPair>
#include <QVector>

/**
 * \brief Manages internationalization and translation of the CloudCompare application.
 * 
 * \details The ccTranslationManager provides functionality for:
 * - Registering translation files
 * - Loading translations for different languages
 * - Generating language selection menus
 * 
 * Key features:
 * - Supports translation files with ISO 639 language codes
 * - Allows dynamic language switching
 * - Provides a centralized translation management system
 * 
 * \note Follows the Singleton pattern to ensure a single, global translation manager
 * 
 * \example
 * \code
 * // Get the singleton translation manager instance
 * ccTranslationManager& translationMgr = ccTranslationManager::Get();
 * 
 * // Register translation files for the application
 * translationMgr.registerTranslatorFile("CloudCompare", "/path/to/translations");
 * 
 * // Load translations for the preferred language
 * translationMgr.loadTranslations();
 * 
 * // Alternatively, load a specific language
 * translationMgr.loadTranslation("fr");  // Load French translations
 * 
 * // Populate a menu with available languages
 * QMenu* languageMenu = new QMenu("Language");
 * translationMgr.populateMenu(languageMenu, "/path/to/translations");
 * \endcode
 */
class CCAPPCOMMON_LIB_API ccTranslationManager : public QObject
{
	Q_OBJECT

public:
	/**
	 * \brief Get the singleton instance of the translation manager
	 * 
	 * \return Reference to the single ccTranslationManager instance
	 */
	static ccTranslationManager& Get();

	/**
	 * \brief Destructor for the translation manager
	 * 
	 * Cleans up resources associated with translations
	 */
	~ccTranslationManager() override = default;

	/**
	 * \brief Register a translation file prefix for loading translations
	 * 
	 * \details Translation files should follow the naming convention:
	 * <prefix>_<lang>.ts, where <lang> is the 2-letter ISO 639 language code
	 * 
	 * Example: CloudCompare_fr.ts for French translations
	 * 
	 * \param[in] prefix Prefix of the translation files
	 * \param[in] path Directory path where translation files are located
	 */
	void registerTranslatorFile(const QString& prefix, const QString& path);

	/**
	 * \brief Load translations for the preferred language
	 * 
	 * Uses the current language preference to load translations
	 */
	inline void loadTranslations() { loadTranslation(languagePref()); }

	/**
	 * \brief Load translations for a specific language
	 * 
	 * \param[in] language 2-letter ISO 639 language code (lowercase)
	 * 
	 * \note Example: "fr" for French, "en" for English
	 */
	void loadTranslation(QString language);

	/**
	 * \brief Populate a menu with available language options
	 * 
	 * Generates a menu containing language selection items based on 
	 * available translation files
	 * 
	 * \param[in,out] menu Pointer to the QMenu to populate with languages
	 * \param[in] pathToTranslationFiles Directory containing translation files
	 */
	void populateMenu(QMenu *menu, const QString &pathToTranslationFiles);

protected:
	/**
	 * \brief Protected default constructor to enforce Singleton pattern
	 */
	explicit ccTranslationManager() = default;

private: // types
	/**
	 * \brief Structure to store translation file information
	 * 
	 * Contains the prefix and path for a translation file
	 */
	struct CCAPPCOMMON_LIB_API TranslatorFile
	{
		QString	prefix; ///< Prefix of the translation file
		QString path;   ///< Path to the translation file
	};

	/**
	 * \brief List of translator files
	 */
	using TranslatorFileList = QVector<TranslatorFile>;

	/**
	 * \brief Pair representing translation information
	 * 
	 * First element is the language code, second is the language name
	 */
	using TranslationInfo = QPair<QString, QString>;

	/**
	 * \brief List of available translations
	 */
	using LanguageList = QVector<TranslationInfo>;

private: // methods
	/**
	 * \brief Get the current language preference
	 * 
	 * \return Current language code
	 */
	QString languagePref() const;

	/**
	 * \brief Generate a list of available languages
	 * 
	 * Scans the translation directory to find available translation files
	 * 
	 * \param[in] appName Name of the application
	 * \param[in] pathToTranslationFiles Directory containing translation files
	 * \return List of available languages with their codes and names
	 */
	LanguageList availableLanguages(const QString& appName, const QString& pathToTranslationFiles) const;

	/**
	 * \brief Set the preferred language
	 * 
	 * \param[in] languageCode 2-letter ISO 639 language code
	 */
	void setLanguagePref(const QString& languageCode);

private: // members
	/**
	 * \brief List of registered translator files
	 * 
	 * Stores information about translation files to be loaded
	 */
	TranslatorFileList	mTranslatorFileInfo;
};
