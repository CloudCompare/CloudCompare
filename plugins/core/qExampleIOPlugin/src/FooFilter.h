#ifndef CC_FOO_FILTER_HEADER
#define CC_FOO_FILTER_HEADER

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

#include "FileIOFilter.h"


class FooFilter : public FileIOFilter
{
public:
	// inherited from FileIOFilter
	virtual bool importSupported() const override;
	virtual bool exportSupported() const override;
	
	virtual CC_FILE_ERROR loadFile( const QString &fileName, ccHObject &container, LoadParameters &parameters ) override;
	
	virtual QStringList getFileFilters( bool onImport ) const override;
	virtual QString getDefaultExtension() const override;
	
	virtual bool canLoadExtension( const QString &upperCaseExt ) const override;
	virtual bool canSave( CC_CLASS_ENUM type, bool &multiple, bool &exclusive ) const override;
};

#endif