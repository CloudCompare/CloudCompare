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

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "ccDefaultPluginInterface.h"
#include "ccLog.h"


// This class keeps JSON from being included in the header
class ccDefaultPluginData
{
public:
	inline QString field( const QString &fieldName )
	{
		return doc.object().value( fieldName ).toString();
	}
	
	ccPluginInterface::ReferenceList references( const QString &fieldName )
	{
		ccPluginInterface::ReferenceList	list;
		
		const QJsonArray array = doc.object().value( fieldName ).toArray();
		
		for ( const QJsonValue &value : array )
		{
			const QJsonObject object = value.toObject();
			
			list += ccPluginInterface::Reference{
				object["text"].toString(),
				object["url"].toString()
			};
		}
		
		return list;
		
	}
	
	ccPluginInterface::ContactList	contacts( const QString &fieldName )
	{
		ccPluginInterface::ContactList	list;
		
		const QJsonArray array = doc.object().value( fieldName ).toArray();
		
		for ( const QJsonValue &value : array )
		{
			const QJsonObject object = value.toObject();
			
			list += ccPluginInterface::Contact{
				object["name"].toString(),
				object["email"].toString()
			};
		}
		
		return list;
	}
	
	QString	m_IID;
	QJsonDocument	doc;
};

ccDefaultPluginInterface::ccDefaultPluginInterface( const QString &resourcePath ) :
	m_data( new ccDefaultPluginData )
{
	if ( resourcePath.isNull() )
	{
		return;
	}
	
	QFile myFile( resourcePath );

	bool opened = myFile.open( QIODevice::ReadOnly );
	
	if ( !opened )
	{
		ccLog::Error( QStringLiteral( "Could not load plugin resources: %1" ).arg( resourcePath ) );
		return;
	}
	
	QByteArray json = myFile.readAll();
	
	QJsonParseError	jsonError;
	
	m_data->doc = QJsonDocument::fromJson( json, &jsonError );
	
	if ( jsonError.error != QJsonParseError::NoError )
	{
		ccLog::Error( QStringLiteral( "Could not parse plugin info: %1" ).arg( jsonError.errorString() ) );
		return;
	}
}

ccDefaultPluginInterface::~ccDefaultPluginInterface()
{
	delete m_data;
}

const QString &ccDefaultPluginInterface::IID() const
{
	return m_data->m_IID;
}

bool ccDefaultPluginInterface::isCore() const
{	
	return m_data->doc.object().value( "core" ).toBool();
}

QString ccDefaultPluginInterface::getName() const
{		
    return m_data->field( "name" );
}

QString ccDefaultPluginInterface::getDescription() const
{		
	return m_data->field( "description" );
}

QIcon ccDefaultPluginInterface::getIcon() const
{
	return QIcon( m_data->field( "icon" ) );
}

ccPluginInterface::ReferenceList ccDefaultPluginInterface::getReferences() const
{
	return m_data->references( "references" );
}

ccPluginInterface::ContactList ccDefaultPluginInterface::getAuthors() const
{
	return m_data->contacts( "authors" );
}

ccPluginInterface::ContactList ccDefaultPluginInterface::getMaintainers() const
{
	return m_data->contacts( "maintainers" );
}

void ccDefaultPluginInterface::setIID(const QString &iid)
{
	m_data->m_IID = iid;
}
