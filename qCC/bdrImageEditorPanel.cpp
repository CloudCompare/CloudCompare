#include "bdrImageEditorPanel.h"

//local
#include "mainwindow.h"
#include "bdr2.5DimEditor.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrImageEditorPanel::bdrImageEditorPanel(bdr2Point5DimEditor* img, QWidget* parent)
	: m_pbdrImshow(img)
	, QDialog(parent, Qt::Tool)
	, Ui::bdrImageEditorPanelDlg()
{
	setupUi(this);
	connect(ZoomFitToolButton,		&QAbstractButton::clicked, this, &bdrImageEditorPanel::ZoomFit);
}
void bdrImageEditorPanel::ZoomFit()
{
	if (m_pbdrImshow) {
		m_pbdrImshow->ZoomFit();
	}
}