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

#include "../include/ccCheckBoxDelegate.h"

#include <QCheckBox>
#include <QApplication>

ccCheckBoxDelegate::ccCheckBoxDelegate(QObject* parent)
	: QItemDelegate(parent)
{

}

QWidget* ccCheckBoxDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	Q_UNUSED(option);
	Q_UNUSED(index);

	QCheckBox* checkBox = new QCheckBox(parent);
	return checkBox;
}

void ccCheckBoxDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
	QCheckBox* cb = qobject_cast<QCheckBox*>(editor);
	if (cb)
	{
		cb->setChecked(index.data().toBool());
	}
}

void ccCheckBoxDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
	QCheckBox* cb = static_cast<QCheckBox*>(editor);
	if (cb && model)
	{
		bool value = (cb->checkState() == Qt::Checked);
		model->setData(index, value, Qt::EditRole);
	}
}

void ccCheckBoxDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	//retrieve data
	bool data = index.model()->data(index, Qt::DisplayRole).toBool();

	//create CheckBox style
	QStyleOptionButton checkboxstyle;
	QRect checkbox_rect = QApplication::style()->subElementRect(QStyle::SE_CheckBoxIndicator, &checkboxstyle);

	//center
	checkboxstyle.rect = option.rect;
	checkboxstyle.rect.setLeft(option.rect.x() +
		option.rect.width() / 2 - checkbox_rect.width() / 2);

	//checked or not checked
	if (data)
		checkboxstyle.state = QStyle::State_On | QStyle::State_Enabled;
	else
		checkboxstyle.state = QStyle::State_Off | QStyle::State_Enabled;
	
	QApplication::style()->drawControl(QStyle::CE_CheckBox, &checkboxstyle, painter);
}

bool ccCheckBoxDelegate::editorEvent(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option, const QModelIndex& index)
{
	if (model && event && event->type() == QEvent::MouseButtonPress)
	{
		bool data = model->data(index, Qt::DisplayRole).toBool();
		model->setData(index, !data, Qt::DisplayRole);
	}

	return QItemDelegate::editorEvent(event, model, option, index);
}

void ccCheckBoxDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	Q_UNUSED(index);

	if (!editor)
	{
		assert(false);
		return;
	}
	
	QStyleOptionButton checkboxstyle;
	QRect checkbox_rect = QApplication::style()->subElementRect(QStyle::SE_CheckBoxIndicator, &checkboxstyle);

	//center
	checkboxstyle.rect = option.rect;
	checkboxstyle.rect.setLeft(option.rect.x() +
		option.rect.width() / 2 - checkbox_rect.width() / 2);

	editor->setGeometry(checkboxstyle.rect);
}
