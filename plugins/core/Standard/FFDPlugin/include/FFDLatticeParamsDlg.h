#pragma once

#include <QDialog>
#include <QSpinBox>
#include <QComboBox>
#include <array>

enum class DeformationType;

class FFDLatticeParamsDlg : public QDialog
{
	Q_OBJECT

public:
	explicit FFDLatticeParamsDlg(QWidget* parent = nullptr);
	
	std::array<unsigned int, 3> getLatticeSize() const;
	DeformationType getDeformationType() const;
	size_t getPreviewPointCount() const;

private:
	QSpinBox* m_spinX;
	QSpinBox* m_spinY;
	QSpinBox* m_spinZ;
	QComboBox* m_deformTypeCombo;
	QSpinBox* m_spinPreviewPoints;
};
