#include "../include/ccColorStyledDelegate.h"

//QT
#include <QPainter>

ccColorStyledDelegate::ccColorStyledDelegate(QObject* parent)
	: QStyledItemDelegate(parent)
{
}

void ccColorStyledDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	if (painter && index.model())
	{
		QColor color = index.model()->data(index, Qt::DisplayRole).value<QColor>();
		painter->fillRect(option.rect, color);
	}
}
