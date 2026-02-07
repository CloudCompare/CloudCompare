#pragma once

#include <QDialog>
#include <QSpinBox>
#include <array>

class FFDLatticeParamsDlg : public QDialog
{
	Q_OBJECT

public:
	explicit FFDLatticeParamsDlg(QWidget* parent = nullptr);
	
	std::array<unsigned int, 3> getLatticeSize() const;

private:
	QSpinBox* m_spinX;
	QSpinBox* m_spinY;
	QSpinBox* m_spinZ;
};
