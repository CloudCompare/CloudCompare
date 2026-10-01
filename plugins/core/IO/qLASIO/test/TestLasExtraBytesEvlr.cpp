#include "TestLasExtraBytesEvlr.h"

#include "LasDetails.h"
#include "LasExtraScalarField.h"

#include <QTemporaryDir>
#include <ccLog.h>
#include <cstring>
#include <laszip/laszip_api.h>

namespace
{
	// Keeps the warnings, to check the ones the parser has to emit
	class WarningLog : public ccLog
	{
	  public:
		QStringList warnings;

		void logMessage(const QString& message, int level) override
		{
			if ((level & ~DEBUG_FLAG) == LOG_WARNING)
			{
				warnings << message;
			}
		}
	};

	WarningLog s_log;

	// the EVLRs start after this many bytes (standing in for the header, VLRs and points)
	constexpr quint64 EVLR_START = 100;

	LasExtraScalarField Field(const char* name, LasExtraScalarField::DataType type)
	{
		LasExtraScalarField field;
		field.type = type;
		strncpy(field.name, name, LasExtraScalarField::MAX_NAME_SIZE - 1);
		return field;
	}

	QByteArray Descriptor(const std::vector<LasExtraScalarField>& fields)
	{
		QByteArray  bytes;
		QDataStream stream(&bytes, QIODevice::WriteOnly);
		for (const LasExtraScalarField& field : fields)
		{
			stream << field;
		}
		return bytes;
	}

	// Writes EVLR_START zero bytes, then one EVLR per (header, payload) pair
	QString WriteFile(const QTemporaryDir& dir, const std::vector<std::pair<LasDetails::EvlrHeader, QByteArray>>& evlrs)
	{
		QString fileName = dir.filePath("test.las");
		QFile   file(fileName);
		if (!file.open(QFile::WriteOnly))
		{
			return {};
		}
		QDataStream      stream(&file);
		const QByteArray padding(static_cast<qsizetype>(EVLR_START), 0);
		stream.writeRawData(padding.constData(), static_cast<int>(padding.size()));
		for (const auto& evlr : evlrs)
		{
			stream << evlr.first;
			stream.writeRawData(evlr.second.constData(), static_cast<int>(evlr.second.size()));
		}
		return fileName;
	}

	LasDetails::EvlrHeader EvlrHeader(const char* userID, uint16_t recordID, uint64_t recordLength)
	{
		LasDetails::EvlrHeader header;
		std::memset(header.userID, 0, LasDetails::EvlrHeader::USER_ID_SIZE);
		std::memset(header.description, 0, LasDetails::EvlrHeader::DESCRIPTION_SIZE);
		strncpy(header.userID, userID, LasDetails::EvlrHeader::USER_ID_SIZE);
		header.recordID     = recordID;
		header.recordLength = recordLength;
		return header;
	}

	// LAS 1.4 header with the given EVLR count, and the given VLR if any
	laszip_header Header(laszip_U32 evlrCount, laszip_vlr_struct* extraBytesVlr)
	{
		laszip_header header{};
		header.version_minor                                  = 4;
		header.number_of_variable_length_records              = (extraBytesVlr ? 1 : 0);
		header.vlrs                                           = extraBytesVlr;
		header.start_of_first_extended_variable_length_record = EVLR_START;
		header.number_of_extended_variable_length_records     = evlrCount;
		return header;
	}

	laszip_vlr_struct ExtraBytesVlr(QByteArray& descriptor)
	{
		laszip_vlr_struct vlr{};
		strcpy(vlr.user_id, "LASF_Spec");
		vlr.record_id                  = 4;
		vlr.record_length_after_header = static_cast<laszip_U16>(descriptor.size());
		vlr.data                       = reinterpret_cast<laszip_U8*>(descriptor.data());
		return vlr;
	}

	const std::vector<LasExtraScalarField> TwoFields{Field("field_a", LasExtraScalarField::u8), Field("field_b", LasExtraScalarField::f32)};

	void CheckTwoFields(const std::vector<LasExtraScalarField>& fields)
	{
		QCOMPARE(fields.size(), size_t(2));
		QCOMPARE(QString(fields[0].name), QString("field_a"));
		QCOMPARE(fields[0].type, LasExtraScalarField::u8);
		QCOMPARE(fields[0].byteOffset, 0u);
		QCOMPARE(QString(fields[1].name), QString("field_b"));
		QCOMPARE(fields[1].type, LasExtraScalarField::f32);
		QCOMPARE(fields[1].byteOffset, 1u);
	}
} // namespace

void TestLasExtraBytesEvlr::initTestCase()
{
	ccLog::RegisterInstance(&s_log);
}

void TestLasExtraBytesEvlr::evlrOnly()
{
	// another EVLR comes first, so the parser has to skip it
	QTemporaryDir    dir;
	const QByteArray other(10, 'x');
	const QByteArray descriptor = Descriptor(TwoFields);
	const QString    fileName   = WriteFile(dir, {{EvlrHeader("other", 1, other.size()), other}, {EvlrHeader("LASF_Spec", 4, descriptor.size()), descriptor}});
	QVERIFY(!fileName.isEmpty());

	s_log.warnings.clear();
	CheckTwoFields(LasExtraScalarField::ParseExtraScalarFields(Header(2, nullptr), fileName));
	QVERIFY(s_log.warnings.isEmpty());
}

void TestLasExtraBytesEvlr::vlrOnly()
{
	QTemporaryDir     dir;
	QByteArray        descriptor = Descriptor(TwoFields);
	laszip_vlr_struct vlr        = ExtraBytesVlr(descriptor);
	const QString     fileName   = WriteFile(dir, {});
	QVERIFY(!fileName.isEmpty());

	s_log.warnings.clear();
	CheckTwoFields(LasExtraScalarField::ParseExtraScalarFields(Header(0, &vlr), fileName));
	QVERIFY(s_log.warnings.isEmpty());
}

void TestLasExtraBytesEvlr::bothPresent()
{
	// the VLR wins, and the EVLR is reported as ignored
	QTemporaryDir     dir;
	QByteArray        vlrDescriptor = Descriptor({Field("vlr_a", LasExtraScalarField::u8)});
	laszip_vlr_struct vlr           = ExtraBytesVlr(vlrDescriptor);
	const QByteArray  descriptor    = Descriptor(TwoFields);
	const QString     fileName      = WriteFile(dir, {{EvlrHeader("LASF_Spec", 4, descriptor.size()), descriptor}});
	QVERIFY(!fileName.isEmpty());

	s_log.warnings.clear();
	const std::vector<LasExtraScalarField> fields = LasExtraScalarField::ParseExtraScalarFields(Header(1, &vlr), fileName);
	QCOMPARE(fields.size(), size_t(1));
	QCOMPARE(QString(fields[0].name), QString("vlr_a"));
	QCOMPARE(s_log.warnings.filter("Extra Bytes EVLR").size(), 1);
}

void TestLasExtraBytesEvlr::truncatedEvlr()
{
	// the EVLR header declares 2 descriptors, but the file ends after 100 bytes
	QTemporaryDir    dir;
	const QByteArray partial(100, 0);
	const QString    fileName = WriteFile(dir, {{EvlrHeader("LASF_Spec", 4, 2 * LasExtraScalarField::VLR_FIELD_SIZE_BYTES), partial}});
	QVERIFY(!fileName.isEmpty());

	s_log.warnings.clear();
	QVERIFY(LasExtraScalarField::ParseExtraScalarFields(Header(1, nullptr), fileName).empty());
	QCOMPARE(s_log.warnings.filter("Truncated EVLR").size(), 1);
}

QTEST_MAIN(TestLasExtraBytesEvlr)
