#include "SegmenterDlg.h"
#include "ui_SegmenterDlg.h"

SegmenterDlg::SegmenterDlg(QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::SegmenterDialog)
{
    ui->setupUi(this);

    connect(ui->btnUndo, &QPushButton::clicked, this, &SegmenterDlg::undoRequested);
    connect(ui->btnRedo, &QPushButton::clicked, this, &SegmenterDlg::redoRequested);
    connect(ui->btnClear, &QPushButton::clicked, this, &SegmenterDlg::clearRequested);
    connect(ui->btnApply, &QPushButton::clicked, this, &SegmenterDlg::applyRequested);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    connect(ui->radioPositive, &QRadioButton::toggled, this, &SegmenterDlg::stateChanged);
    connect(ui->radioNegative, &QRadioButton::toggled, this, &SegmenterDlg::stateChanged);

    connect(ui->sliderWs,  &QSlider::valueChanged, this, &SegmenterDlg::onSliderMoved);
    connect(ui->sliderWc,  &QSlider::valueChanged, this, &SegmenterDlg::onSliderMoved);
    connect(ui->sliderWn,  &QSlider::valueChanged, this, &SegmenterDlg::onSliderMoved);
    connect(ui->sliderTau, &QSlider::valueChanged, this, &SegmenterDlg::onSliderMoved);

    // Collapsible advanced section: toggle visibility and update arrow glyph
    connect(ui->btnAdvanced, &QPushButton::toggled, ui->advancedWidget, &QWidget::setVisible);
    connect(ui->btnAdvanced, &QPushButton::toggled, this, [this](bool checked) {
        ui->btnAdvanced->setText(checked ? "▾ Advanced Settings" : "▸ Advanced Settings");
        adjustSize();
    });

    onSliderMoved();
    this->setWindowFlags(this->windowFlags() | Qt::WindowStaysOnTopHint);
}

SegmenterDlg::~SegmenterDlg()
{
    delete ui;
}

double SegmenterDlg::getSpatialWeight() const   { return ui->sliderWs->value()   / 100.0; }
double SegmenterDlg::getChromaticWeight() const { return ui->sliderWc->value()   / 100.0; }
double SegmenterDlg::getNormalWeight() const    { return ui->sliderWn->value()   / 100.0; }
double SegmenterDlg::getThreshold() const       { return ui->sliderTau->value()  / 10.0;  }
double SegmenterDlg::getRadius() const          { return ui->spinRadius->value() / 100.0; } // cm -> m
bool   SegmenterDlg::isAddingPositiveSeeds() const { return ui->radioPositive->isChecked(); }

void SegmenterDlg::setStatusMessage(const QString& msg) { ui->lblStatus->setText(msg); }
void SegmenterDlg::setPointCount(int count) { ui->lblCount->setText(QString("%1 points").arg(count)); }

void SegmenterDlg::onSliderMoved()
{
    ui->labelWsVal->setText(QString::number(getSpatialWeight(), 'f', 2));
    ui->labelWcVal->setText(QString::number(getChromaticWeight(), 'f', 2));
    ui->labelWnVal->setText(QString::number(getNormalWeight(), 'f', 2));
    ui->labelTauVal->setText(QString::number(getThreshold(), 'f', 1));
}