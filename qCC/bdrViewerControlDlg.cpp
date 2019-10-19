#include "bdrViewerControlDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrViewerControlDlg::bdrViewerControlDlg(QWidget* parent, ccGLWindow* win)
	: QDialog(parent, Qt::SubWindow)
	, m_UI(new Ui::bdrViewerControlDlg)
	, m_win(win)
{
	m_UI->setupUi(this);
	connect(m_UI->zoomGlobalToolButton,			&QAbstractButton::clicked,		this, &bdrViewerControlDlg::doActionZoomGlobal);
	connect(m_UI->zoomFitToolButton,			&QAbstractButton::clicked,		this, &bdrViewerControlDlg::doActionZoomFit);
	connect(m_UI->zoomFitSelectedToolButton,	&QAbstractButton::clicked,		this, &bdrViewerControlDlg::doActionZoomSelected);
	connect(m_UI->viewInfoToolButton,			&QAbstractButton::clicked,		this, &bdrViewerControlDlg::doActionViewInfo);
	connect(m_UI->refreshToolButton,			&QAbstractButton::clicked,		this, &bdrViewerControlDlg::doActionRefresh);
}

void bdrViewerControlDlg::doActionZoomGlobal()
{
}

void bdrViewerControlDlg::doActionZoomFit()
{
}

void bdrViewerControlDlg::doActionZoomSelected()
{
}

void bdrViewerControlDlg::doActionViewInfo()
{
}

void bdrViewerControlDlg::doActionRefresh()
{
}
