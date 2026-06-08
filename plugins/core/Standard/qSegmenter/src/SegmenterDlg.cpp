#include "SegmenterDlg.h"
#include "ui_SegmenterDlg.h" 

SegmenterDlg::SegmenterDlg(QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::SegmenterDialog)
{
	ui->setupUi(this);

	// Buttons (To be implemented later)
	connect(ui->btnUndo, &QPushButton::clicked, this, &SegmenterDlg::undoRequested);
	connect(ui->btnRedo, &QPushButton::clicked, this, &SegmenterDlg::redoRequested);
	connect(ui->btnClear, &QPushButton::clicked, this, &SegmenterDlg::clearRequested);

	connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
	connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

	// Automatically update segmentation on slider changes
	connect(ui->radioPositive, &QRadioButton::toggled, this, &SegmenterDlg::stateChanged);
	connect(ui->spinWs, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SegmenterDlg::stateChanged);
	connect(ui->spinWc, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SegmenterDlg::stateChanged);
	connect(ui->spinTau, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SegmenterDlg::stateChanged);
	
	this->setWindowFlags(this->windowFlags() | Qt::WindowStaysOnTopHint);
}

SegmenterDlg::~SegmenterDlg()
{
	delete ui;
}

double SegmenterDlg::getSpatialWeight() const { return ui->spinWs->value(); }
double SegmenterDlg::getChromaticWeight() const { return ui->spinWc->value(); }
double SegmenterDlg::getThreshold() const { return ui->spinTau->value(); }
bool SegmenterDlg::isAddingPositiveSeeds() const { return ui->radioPositive->isChecked(); }

// Assumes you added QLabel widgets named lblStatus and lblCount in Qt Designer
void SegmenterDlg::setStatusMessage(const QString& msg) {
    if (ui->lblStatus) ui->lblStatus->setText(msg);
}

void SegmenterDlg::setPointCount(int count) {
    if (ui->lblCount) ui->lblCount->setText(QString("%1 points included").arg(count));
}