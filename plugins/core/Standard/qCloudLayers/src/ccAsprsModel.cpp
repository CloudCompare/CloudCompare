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

// CloudCompare
#include <ccLog.h>

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

QVariant ccAsprsModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (role != Qt::DisplayRole)
	{
		return {};
	}

	if (orientation == Qt::Vertical)
	{
		return section;
	}

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

QVariant ccAsprsModel::data(const QModelIndex& index, int role) const
{
	if (!index.isValid())
	{
		return {};
	}

	const AsprsItem& item = m_data[index.row()];

	// specific case for the VISIBLE column
	if (index.column() == VISIBLE)
	{
		// we only provide the value for the 'CheckStateRole' role
		if (role == Qt::CheckStateRole)
		{
			return item.visible ? Qt::Checked : Qt::Unchecked;
		}
		else
		{
			return {};
		}
	}

	// for the others, we only provide the values for the 'Display' and 'Edit' roles
	if (role != Qt::DisplayRole && role != Qt::EditRole)
	{
		return {};
	}

	switch (index.column())
	{
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

bool ccAsprsModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	Q_UNUSED(role)

	if (!index.isValid())
	{
		return false;
	}

	AsprsItem& item = m_data[index.row()];

	switch (index.column())
	{
	case VISIBLE:
	{
		if (role == Qt::CheckStateRole)
		{
			item.visible = static_cast<Qt::CheckState>(value.toInt()) == Qt::Checked;
			Q_EMIT colorChanged(item);
		}
		else
		{
			return false;
		}
	}
	break;

	case NAME:
	{
		QString name = value.toString();
		if (nameExists(name))
		{
			ccLog::Error(QObject::tr("Name %1 already used").arg(name));
			return false;
		}
		else
		{
			item.name = name;
			emit classNamedChanged(index.row(), name);
			break;
		}
	}

	case CODE:
	{
		int code = value.toInt();
		if (codeExists(code))
		{
			ccLog::Error(QObject::tr("Class #%1 already used").arg(code));
			return false;
		}
		else
		{
			int oldCode = item.code;
			item.code = code;
			Q_EMIT codeChanged(item, oldCode);
			break;
		}
	}

	case COLOR:
	{
		item.color = value.value<QColor>();
		Q_EMIT colorChanged(item);
	}
	break;

	case COUNT:
	{
		item.count = value.toInt();
	}
	break;
	}

	Q_EMIT dataChanged(index, index);

	return true;
}

Qt::ItemFlags ccAsprsModel::flags(const QModelIndex& index) const
{
	if (!index.isValid())
	{
		return Qt::ItemIsEnabled;
	}

	Qt::ItemFlags f = QAbstractItemModel::flags(index);
	if (index.column() == NAME || index.column() == CODE)
	{
		f |= (Qt::ItemIsEditable);
	}
	else if (index.column() == VISIBLE)
	{
		f |= (Qt::ItemIsUserCheckable);
	}

	return f;
}

QModelIndex ccAsprsModel::createNewItem()
{
	const int rowNumber = m_data.size();
	beginInsertRows(QModelIndex(), rowNumber, rowNumber);
	m_data.append({ false, "UNNAMED", getUnusedCode(), Qt::GlobalColor::black, 0 });
	endInsertRows();

	return createIndex(rowNumber, NAME);
}

bool ccAsprsModel::nameExists(const QString& name) const
{
	auto item = std::find_if(m_data.begin(), m_data.end(), [name](const AsprsItem& item) { return item.name == name; });
	return item != m_data.end();
}

bool ccAsprsModel::codeExists(int code) const
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
	settings.beginGroup("qCloudLayers/ASPRS");
	{
		AddClass(settings, "Not classified",				 0, Qt::white);
		AddClass(settings, "Unclassified",					 1, Qt::lightGray);
		AddClass(settings, "Ground",						 2, qRgb(166, 116, 4));
		AddClass(settings, "Low vegetation",				 3, qRgb(38, 114, 0));
		AddClass(settings, "Medium vegetation",				 4, qRgb(69, 229, 0));
		AddClass(settings, "High vegetation",				 5, qRgb(204, 240, 123));
		AddClass(settings, "Building",						 6, Qt::yellow);
		AddClass(settings, "Low Noise",						 7, Qt::red);
		AddClass(settings, "Model Keypoint",				 8, Qt::magenta);
		AddClass(settings, "Water",							 9, Qt::blue);
		AddClass(settings, "Rail",							10, qRgb(85, 85, 0));
		AddClass(settings, "Road surface",					11, Qt::darkGray);
		AddClass(settings, "Overlap",						12, qRgb(255, 170, 255));
		AddClass(settings, "Wire Shield/Neutral/Com",		13, qRgb(191, 231, 205));
		AddClass(settings, "Wire Conductors/Phases",		14, qRgb(193, 230, 125));
		AddClass(settings, "Transmission Tower",			15, Qt::darkBlue);
		AddClass(settings, "Wire Insulators",				16, Qt::darkYellow);
		AddClass(settings, "Bridge Deck",					17, Qt::darkCyan);
		AddClass(settings, "High Noise",					18, Qt::darkRed);

		AddClass(settings, "Conductor Attachment Points",	64, qRgb(25, 0, 51));
		AddClass(settings, "Shield Attachment Points",		65, qRgb(51, 0, 102));
		AddClass(settings, "Midspan Points",				66, qRgb(76, 0, 153));
		AddClass(settings, "Structure Top Points",			67, qRgb(102, 0, 204));
		AddClass(settings, "Structure Bottom Points",		68, qRgb(127, 0, 255));

		AddClass(settings, "Guy Wire",						70, qRgb(153, 51, 255));
		AddClass(settings, "Substation",					75, qRgb(178, 102, 255));

		AddClass(settings, "Misc Temporary",				81, qRgb(204, 153, 255));
		AddClass(settings, "Misc Permanent",				82, qRgb(229, 204, 255));
		AddClass(settings, "Misc Fences",					83, qRgb(204, 204, 255));
	}
	settings.endGroup();
	settings.sync();

	load();
}

void ccAsprsModel::load()
{
	QSettings settings;
	settings.beginGroup("qCloudLayers/ASPRS");

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
	settings.remove("qCloudLayers/ASPRS");
	settings.beginGroup("qCloudLayers/ASPRS");
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
	Q_EMIT dataChanged(a, b);
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
