#include "FFDLatticeParamsDlg.h"
#include "FFDLattice.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QDialogButtonBox>

FFDLatticeParamsDlg::FFDLatticeParamsDlg(QWidget* parent)
	: QDialog(parent)
{
	setWindowTitle("FFD Lattice Parameters");
	
	auto* mainLayout = new QVBoxLayout(this);
	
	// X dimension
	auto* xLayout = new QHBoxLayout();
	xLayout->addWidget(new QLabel("X dimension:", this));
	m_spinX = new QSpinBox(this);
	m_spinX->setRange(2, 20);
	m_spinX->setValue(3);
	m_spinX->setToolTip("Number of control points along X axis");
	xLayout->addWidget(m_spinX);
	mainLayout->addLayout(xLayout);
	
	// Y dimension
	auto* yLayout = new QHBoxLayout();
	yLayout->addWidget(new QLabel("Y dimension:", this));
	m_spinY = new QSpinBox(this);
	m_spinY->setRange(2, 20);
	m_spinY->setValue(3);
	m_spinY->setToolTip("Number of control points along Y axis");
	yLayout->addWidget(m_spinY);
	mainLayout->addLayout(yLayout);
	
	// Z dimension
	auto* zLayout = new QHBoxLayout();
	zLayout->addWidget(new QLabel("Z dimension:", this));
	m_spinZ = new QSpinBox(this);
	m_spinZ->setRange(2, 20);
	m_spinZ->setValue(3);
	m_spinZ->setToolTip("Number of control points along Z axis");
	zLayout->addWidget(m_spinZ);
	mainLayout->addLayout(zLayout);
	
	// Deformation type
	auto* deformLayout = new QHBoxLayout();
	deformLayout->addWidget(new QLabel("Interpolation:", this));
	m_deformTypeCombo = new QComboBox(this);
	m_deformTypeCombo->addItem("Linear (trilinear)", static_cast<int>(DeformationType::Linear));
	m_deformTypeCombo->addItem("B-Spline (cubic)", static_cast<int>(DeformationType::BSpline));
	m_deformTypeCombo->setCurrentIndex(1); // B-Spline by default
	m_deformTypeCombo->setToolTip("Linear: C0-continuous, fast.\nB-Spline: C2-continuous, smoother.");
	deformLayout->addWidget(m_deformTypeCombo);
	mainLayout->addLayout(deformLayout);
	
	// Preview point count
	auto* previewLayout = new QHBoxLayout();
	previewLayout->addWidget(new QLabel("Preview points:", this));
	m_spinPreviewPoints = new QSpinBox(this);
	m_spinPreviewPoints->setRange(1000, 500000);
	m_spinPreviewPoints->setSingleStep(5000);
	m_spinPreviewPoints->setValue(15000); // B-Spline default (matches initial combo selection)
	m_spinPreviewPoints->setToolTip("Number of points used for interactive preview.\nFewer points = faster interaction.");
	previewLayout->addWidget(m_spinPreviewPoints);
	mainLayout->addLayout(previewLayout);
	
	// Update preview default when interpolation type changes
	connect(m_deformTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int) {
		DeformationType type = static_cast<DeformationType>(m_deformTypeCombo->currentData().toInt());
		m_spinPreviewPoints->setValue(type == DeformationType::BSpline ? 15000 : 50000);
	});
	
	// Info label
	auto* infoLabel = new QLabel("Total control points will be X × Y × Z", this);
	infoLabel->setStyleSheet("color: gray; font-size: 10pt;");
	mainLayout->addWidget(infoLabel);
	
	// Buttons
	auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
	connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
	connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
	mainLayout->addWidget(buttonBox);
	
	// Update info label when values change
	auto updateInfo = [this, infoLabel]() {
		int total = m_spinX->value() * m_spinY->value() * m_spinZ->value();
		infoLabel->setText(QString("Total control points: %1 × %2 × %3 = %4")
			.arg(m_spinX->value())
			.arg(m_spinY->value())
			.arg(m_spinZ->value())
			.arg(total));
	};
	
	connect(m_spinX, QOverload<int>::of(&QSpinBox::valueChanged), updateInfo);
	connect(m_spinY, QOverload<int>::of(&QSpinBox::valueChanged), updateInfo);
	connect(m_spinZ, QOverload<int>::of(&QSpinBox::valueChanged), updateInfo);
	
	updateInfo();
}

std::array<unsigned int, 3> FFDLatticeParamsDlg::getLatticeSize() const
{
	return { 
		static_cast<unsigned int>(m_spinX->value()),
		static_cast<unsigned int>(m_spinY->value()),
		static_cast<unsigned int>(m_spinZ->value())
	};
}

DeformationType FFDLatticeParamsDlg::getDeformationType() const
{
	return static_cast<DeformationType>(m_deformTypeCombo->currentData().toInt());
}

size_t FFDLatticeParamsDlg::getPreviewPointCount() const
{
	return static_cast<size_t>(m_spinPreviewPoints->value());
}
