#pragma once

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: LAS-IO Plugin                      #
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
//#                   COPYRIGHT: Thomas Montaigu                           #
//#                                                                        #
//##########################################################################

#include "LasDetails.h"
#include "LasExtraScalarField.h"

// LASzip
#include <laszip/laszip_api.h>
// Qt
#include <QDataStream>
#include <QMetaType>
#include <QString>

struct LasVlr
{
	LasVlr() = default;
	explicit LasVlr(const laszip_header& header);
	LasVlr(const LasVlr& rhs);

	LasVlr& operator=(LasVlr rhs);

	static void Swap(LasVlr& lhs, LasVlr& rhs) noexcept;

	inline QString toString() const
	{
		return QString("VLRs: %1").arg(vlrs.size());
	}

	inline laszip_U32 numVlrs() const
	{
		return static_cast<laszip_U32>(vlrs.size());
	}

	friend QDataStream& operator<<(QDataStream& arch, const LasVlr& object)
	{
		arch << static_cast<quint64>(object.vlrs.size());
		for (const laszip_vlr_struct& v : object.vlrs)
		{
			// arch << v;
			arch << v.reserved;
			arch.writeRawData(v.user_id, 16 * sizeof(laszip_CHAR));
			arch << v.record_id;
			arch << v.record_length_after_header;
			arch.writeRawData(v.description, 32 * sizeof(laszip_CHAR));
			arch.writeRawData((const char*)v.data, v.record_length_after_header);
		}

		arch << static_cast<quint64>(object.extraScalarFields.size());
		for (const LasExtraScalarField& e : object.extraScalarFields)
		{
			// arch << e;
			arch.writeRawData((const char*)&e, sizeof(LasExtraScalarField));
		}
		return arch;
	}

	friend QDataStream& operator>>(QDataStream& arch, LasVlr& object)
	{
		quint64 vlrSize = 0;
		arch >> vlrSize;
		object.vlrs.reserve(vlrSize);
		for (quint64 i = 0; i < vlrSize; ++i)
		{
			laszip_vlr_struct v;
			// arch >> v;
			arch >> v.reserved;
			arch.readRawData((char*)v.user_id, 16 * sizeof(laszip_CHAR));
			arch >> v.record_id;
			arch >> v.record_length_after_header;
			arch.readRawData((char*)v.description, 32 * sizeof(laszip_CHAR));
			{
				v.data = new laszip_U8[v.record_length_after_header]; // TODO: potential memory leak
				arch.readRawData((char*)v.data, v.record_length_after_header);
			}
			// arch >> QByteArray((const char*)v.data, v.record_length_after_header);
			object.vlrs.push_back(v);
		}

		quint64 extraScalarFieldCount = 0;
		arch >> extraScalarFieldCount;
		object.extraScalarFields.reserve(extraScalarFieldCount);
		for (quint64 i = 0; i < extraScalarFieldCount; ++i)
		{
			LasExtraScalarField e;
			// arch >> e;
			{
				char* data = (char*)&e;
				uint  len  = sizeof(LasExtraScalarField);
				arch.readRawData(data, len);
			}
			object.extraScalarFields.push_back(e);
		}

		return arch;
	}

	std::vector<laszip_vlr_struct>   vlrs;
	std::vector<LasExtraScalarField> extraScalarFields;
};

Q_DECLARE_METATYPE(LasVlr);
