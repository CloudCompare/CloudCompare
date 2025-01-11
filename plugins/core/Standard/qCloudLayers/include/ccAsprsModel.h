#pragma once

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

//Qt
#include <QAbstractTableModel>
#include <QColor>

class ccAsprsModel : public QAbstractTableModel
{
	Q_OBJECT

public:
	ccAsprsModel(QObject* parent = nullptr);

	int rowCount(const QModelIndex& parent) const;
	int columnCount(const QModelIndex& parent) const;
	QVariant data(const QModelIndex& index, int role) const;
	QVariant headerData(int section, Qt::Orientation orientation, int role) const;

	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole);
	Qt::ItemFlags flags(const QModelIndex& index) const;

	QModelIndex createNewItem();

	// load asprs items data from qsettings
	void load();

	// save asprs items data to qsettings
	void save() const;

	enum Column
	{
		VISIBLE,
		NAME,
		CODE,
		COLOR,
		COUNT,
		LAST
	};

	struct AsprsItem
	{
		bool visible = false;
		QString name;
		int code = 0;
		QColor color;
		int count = 0;
	};

	void refreshData();
	
	inline const QList<AsprsItem>& getData() const { return m_data; }
	inline QList<AsprsItem>& getData() { return m_data; }
	
	AsprsItem* find(QString name);
	AsprsItem* find(int code);
	
	int indexOf(QString name) const;

public Q_SLOTS:
	bool removeRows(int position, int rows, const QModelIndex& parent);

Q_SIGNALS:
	void codeChanged(AsprsItem item, int oldCode);
	void colorChanged(AsprsItem item);
	void classNamedChanged(int row, QString newName);

private:
	QList<AsprsItem> m_data;

private:
	bool nameExists(const QString& name) const;
	bool codeExists(int code) const;

	// default asprs items
	void createDefaultItems();
	int getUnusedCode() const;
};

