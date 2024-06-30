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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccColorScale.h"

//Qt
#include <QUuid>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

//CCCoreLib
#include <CCGeom.h>

//Local
#include "ccLog.h"


static const QString s_xmlCloudCompare("CloudCompare");
static const QString s_xmlColorScaleTitle("ColorScale");
static const QString s_xmlColorScaleProperties("Properties");
static const QString s_xmlColorScaleData("Data");
constexpr int s_xmlColorScaleVer = 1;

// These extra definitions are required in C++11.
// In C++17, "static constexpr" is implicitly inline, so these are not required.
constexpr unsigned ccColorScale::MIN_STEPS;
constexpr unsigned ccColorScale::DEFAULT_STEPS;
constexpr unsigned ccColorScale::MAX_STEPS;

ccColorScale::Shared ccColorScale::Create(const QString& name)
{
	return ccColorScale::Shared(new ccColorScale(name));
}

ccColorScale::ccColorScale(const QString& name, const QString& uuid/*=QString()*/)
	: m_name(name)
	, m_uuid(uuid)
	, m_updated(false)
	, m_relative(true)
	, m_locked(false)
	, m_absoluteMinValue(0.0)
	, m_absoluteRange(1.0)
{
	if (m_uuid.isNull())
		generateNewUuid();
}

void ccColorScale::generateNewUuid()
{
	m_uuid = QUuid::createUuid().toString();
}

ccColorScale::Shared ccColorScale::copy(const QString& uuid/*=QString()*/) const
{
	ccColorScale::Shared newCS(new ccColorScale(m_name, uuid));
	try
	{
		newCS->m_relative = m_relative;
		newCS->m_locked = m_locked;
		newCS->m_absoluteMinValue = m_absoluteMinValue;
		newCS->m_absoluteRange = m_absoluteRange;
		newCS->m_steps = m_steps;
		newCS->m_customLabels = m_customLabels;
		newCS->update();
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Not enough memory to copy the color scale");
		return ccColorScale::Shared(nullptr);
	}

	return newCS;
}

void ccColorScale::insert(const ccColorScaleElement& step, bool autoUpdate/*=true*/)
{
	if (m_locked)
	{
		ccLog::Warning(QString("[ccColorScale::insert] Scale '%1' is locked!").arg(m_name));
		return;
	}

	m_steps.push_back(step);

	m_updated = false;

	if (autoUpdate && m_steps.size() >= static_cast<int>(MIN_STEPS))
	{
		update();
	}
}

void ccColorScale::clear()
{
	if (m_locked)
	{
		ccLog::Warning(QString("[ccColorScale::clear] Scale '%1' is locked!").arg(m_name));
		return;
	}

	m_steps.clear();

	m_updated = false;
}

void ccColorScale::remove(int index, bool autoUpdate/*=true*/)
{
	if (m_locked)
	{
		ccLog::Warning(QString("[ccColorScale::remove] Scale '%1' is locked!").arg(m_name));
		return;
	}

	m_steps.removeAt(index);
	m_updated = false;

	if (autoUpdate)
		update();
}

void ccColorScale::sort()
{
	std::sort(m_steps.begin(), m_steps.end(), ccColorScaleElement::IsSmaller);
}

void ccColorScale::update()
{
	m_updated = false;

	if (m_steps.size() >= static_cast<int>(MIN_STEPS))
	{
		sort();

		const unsigned stepCount = static_cast<unsigned>(m_steps.size());
		assert(stepCount >= 2);
		assert(m_steps.front().getRelativePos() == 0.0);
		assert(m_steps.back().getRelativePos() == 1.0);
		if (m_steps.front().getRelativePos() != 0.0 || m_steps.back().getRelativePos() != 1.0)
		{
			ccLog::Warning(QString("[ccColorScale] Scale '%1' is invalid! (boundaries are not [0.0-1.0]").arg(getName()));
		}
		else
		{
			unsigned j = 0; //current interval
			for (unsigned i = 0; i < MAX_STEPS; ++i)
			{
				const double relativePos = static_cast<double>(i) / (MAX_STEPS - 1);

				//forward to the right interval
				while (j + 2 < stepCount && m_steps[j + 1].getRelativePos() < relativePos)
					++j;

				// linear interpolation
				const CCVector3d colBefore (m_steps[j].getColor().redF(),
											m_steps[j].getColor().greenF(),
											m_steps[j].getColor().blueF());
				
				const CCVector3d colNext(	m_steps[j + 1].getColor().redF(),
											m_steps[j + 1].getColor().greenF(),
											m_steps[j + 1].getColor().blueF());

				//interpolation coef
				const double alpha = (relativePos - m_steps[j].getRelativePos()) / (m_steps[j + 1].getRelativePos() - m_steps[j].getRelativePos());

				const CCVector3d interpCol = colBefore + (colNext - colBefore) * alpha;

				m_rgbaScale[i] = ccColor::Rgb(	static_cast<ColorCompType>(interpCol.x * ccColor::MAX),
												static_cast<ColorCompType>(interpCol.y * ccColor::MAX),
												static_cast<ColorCompType>(interpCol.z * ccColor::MAX) );
			}

			m_updated = true;
		}
	}
	else
	{
		ccLog::Warning(QString("[ccColorScale] Scale '%1' is invalid! (not enough elements)").arg(getName()));
	}

	if (!m_updated)
	{
		//I saw an invalid scale and I want it painted black ;)
		for (unsigned i = 0; i < MAX_STEPS; ++i)
		{
			m_rgbaScale[i] = ccColor::black;
		}
	}
}

bool ccColorScale::toFile(QFile& out, short dataVersion) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));
	if (dataVersion < 27)
	{
		assert(false);
		return false;
	}

	QDataStream outStream(&out);

	//name (dataVersion>=27)
	outStream << m_name;

	//UUID (dataVersion>=27)
	outStream << m_uuid;

	//relative state (dataVersion>=27)
	if (out.write((const char*)&m_relative, sizeof(bool)) < 0)
		return WriteError();

	//Absolute min value (dataVersion>=27)
	if (out.write((const char*)&m_absoluteMinValue, sizeof(double)) < 0)
		return WriteError();
	//Absolute range (dataVersion>=27)
	if (out.write((const char*)&m_absoluteRange, sizeof(double)) < 0)
		return WriteError();

	//locked state (dataVersion>=27)
	if (out.write((const char*)&m_locked, sizeof(bool)) < 0)
		return WriteError();

	//steps list (dataVersion>=27)
	{
		//steps count
		uint32_t stepCount = static_cast<uint32_t>(m_steps.size());
		if (out.write((const char*)&stepCount, 4) < 0)
			return WriteError();

		//write each step
		for (uint32_t i = 0; i < stepCount; ++i)
		{
			outStream << m_steps[i].getRelativePos();
			outStream << m_steps[i].getColor();
		}
	}

	//custom labels (dataVersion>=40)
	if (dataVersion >= 40)
	{
		//custom label count
		uint32_t labelCount = static_cast<uint32_t>(m_customLabels.size());
		if (out.write((const char*)&labelCount, 4) < 0)
			return WriteError();

		//write each custom label
		for (LabelSet::const_iterator it = m_customLabels.begin(); it != m_customLabels.end(); ++it)
		{
			outStream << it->value;
			if (dataVersion >= 54)
			{
				outStream << it->text;
			}
		}
	}

	return true;
}

bool ccColorScale::fromFile(QFile& in, short dataVersion, int flags, LoadedIDMap& oldToNewIDMap)
{
	if (dataVersion < 27) //structure appeared at version 27!
		return false;

	QDataStream inStream(&in);

	//name (dataVersion>=27)
	inStream >> m_name;

	//UUID (dataVersion>=27)
	inStream >> m_uuid;

	//relative state (dataVersion>=27)
	if (in.read((char*)&m_relative, sizeof(bool)) < 0)
		return ReadError();

	//Absolute min value (dataVersion>=27)
	if (in.read((char*)&m_absoluteMinValue, sizeof(double)) < 0)
		return ReadError();
	//Absolute range (dataVersion>=27)
	if (in.read((char*)&m_absoluteRange, sizeof(double)) < 0)
		return ReadError();

	//locked state (dataVersion>=27)
	if (in.read((char*)&m_locked, sizeof(bool)) < 0)
		return ReadError();

	//steps list (dataVersion>=27)
	{
		//steps count
		uint32_t stepCount = 0;
		if (in.read((char*)&stepCount, 4) < 0)
			return ReadError();

		//read each step
		m_steps.clear();
		for (uint32_t i = 0; i < stepCount; ++i)
		{
			double relativePos = 0.0;
			QColor color(Qt::white);
			inStream >> relativePos;
			inStream >> color;

			m_steps.push_back(ccColorScaleElement(relativePos, color));
		}

		update();
	}

	//custom labels (dataVersion>=40)
	if (dataVersion >= 40)
	{
		//custom label count
		uint32_t labelCount = 0;
		if (in.read((char*)&labelCount, 4) < 0)
			return ReadError();

		try
		{
			for (uint32_t i = 0; i < labelCount; ++i)
			{
				double label = 0.0;
				QString text;

				inStream >> label;
				if (dataVersion >= 54)
				{
					inStream >> text;
				}

				m_customLabels.insert({ label, text });
			}
		}
		catch (const std::bad_alloc&)
		{
			//not enough memory
			return MemoryError();
		}
	}

	return true;
}

short ccColorScale::minimumFileVersion() const
{
	if (!m_customLabels.empty())
	{
		for (LabelSet::const_iterator it = m_customLabels.begin(); it != m_customLabels.end(); ++it)
		{
			if (!it->text.isEmpty())
			{
				// custom labels with an overridding text --> version 54
				return 54;
			}
		}

		// with custom labels, but no overridding text --> version 40
		return 40;
	}
	else
	{
		// without custom labels  --> version 27
		return 27;
	}
}

void ccColorScale::setAbsolute(double minVal, double maxVal)
{
	assert(maxVal >= minVal);

	m_relative = false;

	m_absoluteMinValue = minVal;
	m_absoluteRange = maxVal - minVal;

	//as 'm_absoluteRange' is used for division, we make sure it is not left to 0!
	m_absoluteRange = std::max(m_absoluteRange, 1e-12);
}

void ccColorScale::getAbsoluteBoundaries(double& minVal, double& maxVal) const
{
	minVal = m_absoluteMinValue;
	maxVal = m_absoluteMinValue + m_absoluteRange;
}

bool ccColorScale::saveAsXML(const QString& filename) const
{
	QFile file(filename);
	if (!file.open(QFile::WriteOnly | QFile::Text))
	{
		ccLog::Error(QString("Failed to open file '%1' for writing!").arg(filename));
		return false;
	}

	//write content
	QXmlStreamWriter stream(&file);
	stream.setAutoFormatting(true);
	stream.writeStartDocument();
	{
		stream.writeStartElement(s_xmlCloudCompare);	// CloudCompare
		{
			stream.writeStartElement(s_xmlColorScaleTitle);	// ColorScale
			{
				//file version
				stream.writeAttribute("version", QString::number(s_xmlColorScaleVer));

				//Properties
				stream.writeStartElement(s_xmlColorScaleProperties);
				{
					stream.writeTextElement("name", getName());
					stream.writeTextElement("uuid", getUuid());
					stream.writeTextElement("absolute", isRelative() ? "0" : "1");
					if (!isRelative())
					{
						stream.writeTextElement("minValue", QString::number(m_absoluteMinValue, 'g', 12));
						stream.writeTextElement("range", QString::number(m_absoluteRange, 'g', 12));
					}
				}
				stream.writeEndElement(); //Properties

				//Data
				stream.writeStartElement(s_xmlColorScaleData);
				{
					//write each step
					{
						for (QList<ccColorScaleElement>::const_iterator it = m_steps.begin(); it != m_steps.end(); ++it)
						{
							stream.writeStartElement("step");
							{
								const ccColorScaleElement& elem = *it;
								const QColor& color = elem.getColor();
								double relativePos = elem.getRelativePos();

								stream.writeAttribute("r", QString::number(color.red()));
								stream.writeAttribute("g", QString::number(color.green()));
								stream.writeAttribute("b", QString::number(color.blue()));
								stream.writeAttribute("pos", QString::number(relativePos, 'g', 12));
							}
							stream.writeEndElement(); //step
						}
					}

					//write custom labels as well (if any)
					{
						for (LabelSet::const_iterator it = m_customLabels.begin(); it != m_customLabels.end(); ++it)
						{
							stream.writeStartElement("label");
							{
								stream.writeAttribute("val", QString::number(it->value, 'g', 12));
								if (!it->text.isEmpty())
								{
									stream.writeAttribute("text", it->text);
								}
							}
							stream.writeEndElement(); //label
						}
					}

				}
				stream.writeEndElement(); //Data
			}
			stream.writeEndElement(); // ColorScale
		}
		stream.writeEndElement(); // CloudCompare
	}
	stream.writeEndDocument();

	return true;
}

ccColorScale::Shared ccColorScale::LoadFromXML(const QString& filename)
{
	QFile file(filename);
	if (!file.open(QFile::ReadOnly | QFile::Text))
	{
		ccLog::Error(QString("Failed to open file '%1' for reading!").arg(filename));
		return Shared(nullptr);
	}

	Shared scale(nullptr);

	//read content
	QXmlStreamReader stream(&file);
	bool error = true;
	while (true) //fake loop for easy break
	{
		//expected: CloudCompare
		if (	!stream.readNextStartElement()
			||	stream.name() != s_xmlCloudCompare)
		{
			break;
		}

		//expected: ColorScale
		if (	!stream.readNextStartElement()
			||	stream.name() != s_xmlColorScaleTitle)
		{
			break;
		}

		//read version number
		QXmlStreamAttributes attributes = stream.attributes();
		if (attributes.size() == 0 || attributes[0].name() != "version")
		{
			break;
		}
		bool ok = false;
		int version = attributes[0].value().toString().toInt(&ok);
		if (!ok || version > s_xmlColorScaleVer)
		{
			if (ok)
				ccLog::Warning(QString("[ccColorScale::LoadFromXML] Unhandled version: %1").arg(version));
			break;
		}

		//expected: Properties
		if (	!stream.readNextStartElement()
			||	stream.name() != s_xmlColorScaleProperties)
		{
			break;
		}

		//we can now create the scale structure
		scale = Shared(new ccColorScale("temp"));

		//read elements
		int missingItems = 3;
		while (!stream.atEnd() && missingItems > 0)
		{
			if (!stream.readNextStartElement())
			{
				break;
			}
			QStringRef itemName = stream.name();
			QString itemValue = stream.readElementText();
			ccLog::Print(QString("[XML] Item '%1': '%2'").arg(itemName.toString(), itemValue));

			if (itemName == "name")
			{
				scale->setName(itemValue);
				--missingItems;
			}
			else if (itemName == "uuid")
			{
				scale->setUuid(itemValue);
				--missingItems;
			}
			else if (itemName == "absolute")
			{
				if (itemValue == "1")
				{
					scale->setAbsolute(0, 1); //the true values will be updated afterwards
					missingItems += 2; //we need the minValue and range items!
				}
				--missingItems;
			}
			else if (itemName == "minValue")
			{
				scale->m_absoluteMinValue = itemValue.toDouble(&ok);
				if (!ok)
					break;
				--missingItems;
			}
			else if (itemName == "range")
			{
				scale->m_absoluteRange = itemValue.toDouble(&ok);
				if (!ok)
					break;
				--missingItems;
			}
		}

		if (missingItems > 0)
		{
			ccLog::Warning(QString("[ccColorScale::LoadFromXML] Missing properties!"));
			break;
		}
		stream.skipCurrentElement();

		//expected: Data
		if (	!stream.readNextStartElement()
			||	stream.name() != s_xmlColorScaleData)
		{
			ccLog::Warning(QString("[ccColorScale::LoadFromXML] Unexpected element: %1").arg(stream.name().toString()));
			break;
		}

		//read data
		bool dataError = false;
		try
		{
			while (!stream.atEnd())
			{
				if (!stream.readNextStartElement())
					break;
				if (stream.name() == "step")
				{
					QXmlStreamAttributes attributes = stream.attributes();
					int attributeCount = attributes.size();
					if (attributeCount < 4)
					{
						dataError = true;
						break;
					}
					QColor rgb;
					double pos = 0;
					for (int i = 0; i < attributes.size(); ++i)
					{
						QString name = attributes[i].name().toString().toUpper();
						QString value = attributes[i].value().toString();
						if (name == "R")
							rgb.setRed(value.toInt());
						else if (name == "G")
							rgb.setGreen(value.toInt());
						else if (name == "B")
							rgb.setBlue(value.toInt());
						else if (name == "POS")
							pos = value.toDouble();
						else
							--attributeCount;
					}

					if (attributeCount < 4)
					{
						ccLog::Warning(QString("[ccColorScale::LoadFromXML] Missing data attributes!"));
						dataError = true;
						break;
					}
					stream.skipCurrentElement();

					scale->insert(ccColorScaleElement(pos, rgb), false);
				}
				else if (stream.name() == "label")
				{
					QXmlStreamAttributes attributes = stream.attributes();
					int attributeCount = attributes.size();
					if (attributeCount < 1)
					{
						dataError = true;
						break;
					}

					double value = std::numeric_limits<double>::quiet_NaN();
					QString text;
					for (int i = 0; i < attributes.size(); ++i)
					{
						QString name = attributes[i].name().toString().toUpper();
						if (name == "VAL")
						{
							QString valueStr = attributes[i].value().toString();
							bool ok = false;
							value = valueStr.toDouble(&ok);
							if (!ok)
							{
								ccLog::Warning(QString("[ccColorScale::LoadFromXML] Invalid value:") + valueStr);
								value = std::numeric_limits<double>::quiet_NaN();
								dataError = true;
							}
						}
						else if (name == "TEXT")
						{
							text = attributes[i].value().toString();
						}
					}

					if (std::isfinite(value))
					{
						// we have a valid label
						scale->m_customLabels.insert({ value, text });
					}

					stream.skipCurrentElement();
				}
			}
		}
		catch (const std::bad_alloc&)
		{
			ccLog::Warning(QString("[ccColorScale::LoadFromXML] Not enough memory!"));
			dataError = true;
		}
		scale->update();

		//end
		error = dataError;
		break;
	}

	if (error)
	{
		scale.clear();
		ccLog::Error(QString("An error occurred while reading file '%1'").arg(filename));
	}

	return scale;
}
