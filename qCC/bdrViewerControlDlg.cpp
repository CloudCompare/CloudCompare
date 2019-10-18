#include "bdrViewerControlDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrViewerControlDlg::bdrViewerControlDlg(QWidget* parent)
	: QDialog(parent, Qt::SubWindow)
	, m_UI(new Ui::bdrViewerControlDlg)
{
	m_UI->setupUi(this);

}