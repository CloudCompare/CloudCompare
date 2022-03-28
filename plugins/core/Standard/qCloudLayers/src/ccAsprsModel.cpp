//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: qCloudLayers                    #
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
//#                     COPYRIGHT: WigginsTech 2022                        #
//#                                                                        #
//##########################################################################

#include "../include/ccAsprsModel.h"

//QT
#include <QSettings>

ccAsprsModel::ccAsprsModel(QObject* parent)
	: QAbstractTableModel(parent)
{
	load();

	if (m_data.size() == 0)
	{
		createDefaultItems();
	}
}

int ccAsprsModel::rowCount(const QModelIndex& parent) const
{
	Q_UNUSED(parent)

	return m_data.size();
}

int ccAsprsModel::columnCount(const QModelIndex& parent) const
{
	Q_UNUSED(parent)

	return LAST;
}

QVariant ccAsprsModel::data(const QModelIndex& index, int role) const
{
	if (!index.isValid() || (role != Qt::DisplayRole && role != Qt::EditRole))
		return {};

	const AsprsItem& item = m_data[index.row()];

	switch (index.column())
	{
		case VISIBLE:
			return item.visible;
		case NAME:
			return item.name;
		case CODE:
			return item.code;
		case COLOR:
			return item.color;
		case COUNT:
			return item.count;
		default:
			assert(false);
			break;
	}

	return {};
}

QVariant ccAsprsModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (role != Qt::DisplayRole)
		return {};
	
	if (orientation == Qt::Vertical)
		return section;
	
	switch (section)
	{
	case VISIBLE:
		return "Visible";
	case NAME:
		return "Name";
	case CODE:
		return "Code";
	case COLOR:
		return "Color";
	case COUNT:
		return "Count";
	default:
		assert(false);
		break;
	}

	return {};
}

bool ccAsprsModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	Q_UNUSED(role)

	if (!index.isValid())
		return false;

	AsprsItem& item = m_data[index.row()];

	switch (index.column())
	{
		case VISIBLE:
		{
			item.visible = value.toBool();
			emit colorChanged(item);
		}
		break;
		
		case NAME:
		{
			const QString name = value.toString();
			if (!isNameExist(name))
			{
				item.name = name;
				break;
			}
			else
			{
				return false;
			}
		}
		
		case CODE:
		{
			const int code = value.toInt();
			if (!isCodeExist(code))
			{
				const int oldCode = item.code;
				item.code = code;
				emit codeChanged(item, oldCode);
				break;
			}
			else
			{
				return false;
			}
		}
		
		case COLOR:
		{
			item.color = value.value<QColor>();
			emit colorChanged(item);
		}
		break;
		
		case COUNT:
		{
			item.count = value.toInt();
		}
		break;
	}

	emit dataChanged(index, index);

	return true;
}

Qt::ItemFlags ccAsprsModel::flags(const QModelIndex& index) const
{
	if (!index.isValid())
		return Qt::ItemIsEnabled;

	Qt::ItemFlags flags = QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
	if (index.column() == COLOR || index.column() == COUNT)
	{
		flags &= ~(Qt::ItemIsEditable);
	}
	
	return flags;
}

QModelIndex ccAsprsModel::createNewItem()
{
	const int rowNumber = m_data.size();
	beginInsertRows(QModelIndex(), rowNumber, rowNumber);
	m_data.append({ false, "UNNAMED", getUnusedCode(), Qt::GlobalColor::black, 0 });
	endInsertRows();

	return createIndex(rowNumber, NAME);
}

bool ccAsprsModel::isNameExist(const QString& name) const
{
	auto item = std::find_if(m_data.begin(), m_data.end(), [name](const AsprsItem& item) { return item.name == name; });
	return item != m_data.end();
}

bool ccAsprsModel::isCodeExist(int code) const
{
	auto item = std::find_if(m_data.begin(), m_data.end(), [code](const AsprsItem& item) { return item.code == code; });
	return item != m_data.end();
}

static void AddClass(QSettings& settings, const QString& className, int classValue, QColor classColor, bool visible = true)
{
	QString cleanClassName = className;
	cleanClassName.replace(QChar('/'), QChar('@'));

	settings.beginGroup(cleanClassName);
	{
		settings.setValue("class", classValue);
		settings.setValue("color", classColor.rgb());
		settings.setValue("visible", visible);
	}
	settings.endGroup();
}

static void ReadClass(const QSettings& settings, const QString& className, ccAsprsModel::AsprsItem& item)
{
	QString cleanClassName = className;
	cleanClassName.replace(QChar('/'), QChar('@'));
	
	QString readableClassName = className;
	readableClassName.replace(QChar('@'), QChar('/'));

	item.name = readableClassName;
	item.code = settings.value(cleanClassName + "/class", 0).toInt();
	item.color = QColor(settings.value(cleanClassName + "/color", 0).toUInt());
	item.visible = settings.value(cleanClassName + "/visible", true).toBool();
	item.count = 0;
}

void ccAsprsModel::createDefaultItems()
{
	QSettings settings;
	settings.beginGroup("qCloudLayers\\ASPRS");
	{
		AddClass(settings, "Not classified", 0, Qt::white);
		AddClass(settings, "Unclassified", 1, Qt::lightGray);
		AddClass(settings, "Ground", 2, qRgb(166, 116, 4));
		AddClass(settings, "Low vegetation", 3, qRgb(38, 114, 0));
		AddClass(settings, "Medium vegetation", 4, qRgb(69, 229, 0));
		AddClass(settings, "High vegetation", 5, qRgb(204, 240, 123));
		AddClass(settings, "Building", 6, qRgb(210, 104, 64));
		AddClass(settings, "Low Noise", 7, qRgb(128, 78, 193));
		AddClass(settings, "Model Keypoint", 8, qRgb(127, 0, 0));
		AddClass(settings, "Water", 9, Qt::blue);
		AddClass(settings, "Rail", 10, qRgb(254, 1, 0));
		AddClass(settings, "Road surface", 11, qRgb(98, 2, 204));
		AddClass(settings, "Overlap", 12, qRgb(233, 188, 203));
		AddClass(settings, "Wire Shield/Neutral/Com", 13, qRgb(191, 231, 205));
		AddClass(settings, "Wire Conductors/Phases", 14, qRgb(193, 230, 125));
		AddClass(settings, "Transmission Tower", 15, qRgb(1, 0, 252));
		AddClass(settings, "Wire Insulators", 16, qRgb(233, 243, 17));
		AddClass(settings, "Bridge Deck", 17, qRgb(182, 102, 85));
		AddClass(settings, "High Noise", 18, qRgb(255, 0, 254));

		AddClass(settings, "Conductor Attachment Points", 64, qRgb(0, 0, 0));
		AddClass(settings, "Shield Attachment Points", 65, qRgb(0, 0, 0));
		AddClass(settings, "Midspan Points", 66, qRgb(0, 0, 0));
		AddClass(settings, "Structure Top Points", 67, qRgb(0, 0, 0));
		AddClass(settings, "Structure Bottom Points", 68, qRgb(0, 0, 0));

		AddClass(settings, "Guy Wire", 70, qRgb(0, 0, 0));
		AddClass(settings, "Substation", 75, qRgb(0, 0, 0));

		AddClass(settings, "Misc Temporary", 81, qRgb(0, 0, 0));
		AddClass(settings, "Misc Permanent", 82, qRgb(0, 0, 0));
		AddClass(settings, "Misc Fences", 83, qRgb(0, 0, 0));
	}
	settings.endGroup();
	settings.sync();

	load();
}

void ccAsprsModel::load()
{
	QSettings settings;
	settings.beginGroup("qCloudLayers\\ASPRS");

	QStringList classes = settings.childGroups();

	m_data.clear();
	m_data.reserve(classes.size());
	for (int i = 0; i < classes.length(); ++i)
	{
		ccAsprsModel::AsprsItem item;
		ReadClass(settings, classes[i], item);
		m_data.append(item);
	}
}

void ccAsprsModel::save() const
{
	QSettings settings;
	settings.remove("qCloudLayers\\ASPRS");
	settings.beginGroup("qCloudLayers\\ASPRS");
	{
		for (int i = 0; i < m_data.length(); ++i)
		{
			const AsprsItem& item = m_data[i];
			AddClass(settings, item.name, item.code, item.color, item.visible);
		}
	}
	settings.endGroup();
	settings.sync();
}

void ccAsprsModel::refreshData()
{
	QModelIndex a = createIndex(0, COUNT);
	QModelIndex b = createIndex(m_data.count() - 1, COUNT);
	emit dataChanged(a, b);
}

bool ccAsprsModel::removeRows(int position, int rows, const QModelIndex& parent)
{
	Q_UNUSED(parent);

	beginRemoveRows(QModelIndex(), position, position + rows - 1);
	for (int i = 0; i < rows; ++i)
	{
		m_data.removeAt(position);
	}
	endRemoveRows();

	return true;
}

ccAsprsModel::AsprsItem* ccAsprsModel::find(QString name)
{
	auto it = std::find_if(m_data.begin(), m_data.end(), [name](const ccAsprsModel::AsprsItem &item) { return item.name == name; });
	return it != m_data.end() ? &(*it) : nullptr;
}

ccAsprsModel::AsprsItem* ccAsprsModel::find(int code)
{
	auto it = std::find_if(m_data.begin(), m_data.end(), [code](const ccAsprsModel::AsprsItem &item) { return item.code == code; });
	return it != m_data.end() ? &(*it) : nullptr;
}
  
int ccAsprsModel::indexOf(QString name) const
{
	auto it = std::find_if(m_data.begin(), m_data.end(), [name](const ccAsprsModel::AsprsItem &item) { return item.name == name; });
	return it != m_data.end() ? it - m_data.begin() : -1;
}

int ccAsprsModel::getUnusedCode() const
{
	auto it = std::max_element(m_data.cbegin(), m_data.cend(), [](const AsprsItem &a, const AsprsItem &b) { return a.code < b.code; });
	return it != m_data.end() ? (*it).code + 1 : 0;
}
