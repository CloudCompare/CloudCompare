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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#ifndef CC_DEFAULT_PLUGIN_INTERFACE_HEADER
#define CC_DEFAULT_PLUGIN_INTERFACE_HEADER

#include <QString>

#include "ccPluginInterface.h"


class ccDefaultPluginData;


class ccDefaultPluginInterface : public ccPluginInterface
{
public:
	ccDefaultPluginInterface( const QString &resourcePath = QString() );

	virtual ~ccDefaultPluginInterface();
	
	virtual bool isCore() const override;

	virtual QString getName() const override;
	virtual QString getDescription() const override;
	
	virtual QIcon getIcon() const override;
	
	virtual ReferenceList getReferences() const override;
	virtual ContactList getAuthors() const override;
	virtual ContactList getMaintainers() const override;
	
private:
	ccDefaultPluginData	*m_data;
};

#endif
