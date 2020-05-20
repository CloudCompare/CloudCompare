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

#pragma once

#include <QString>

#include "ccPluginInterface.h"


class ccDefaultPluginData;


class ccDefaultPluginInterface : public ccPluginInterface
{
public:
	~ccDefaultPluginInterface() override;
		
	bool isCore() const override;

	QString getName() const override;
	QString getDescription() const override;
	
	QIcon getIcon() const override;
	
	ReferenceList getReferences() const override;
	ContactList getAuthors() const override;
	ContactList getMaintainers() const override;
	
	bool start() override { return true; }
	void stop() override {}
	
	ccExternalFactory *getCustomObjectsFactory() const override { return nullptr; }
	
	void registerCommands(ccCommandLineInterface *cmd) override { Q_UNUSED( cmd ); }
	
protected:
	ccDefaultPluginInterface( const QString &resourcePath = QString() );
	
private:
	void setIID( const QString& iid ) override;
	const QString& IID() const override;
		
	ccDefaultPluginData	*m_data;
};
