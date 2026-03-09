#pragma once

#include <QDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <array>
#include <vector>

class ccHObject;
class ccPointCloud;
enum class DeformationType;

class FFDLatticeParamsDlg : public QDialog
{
	Q_OBJECT

public:
	explicit FFDLatticeParamsDlg(QWidget* parent = nullptr);
	
	//! Populate the trajectory combo with available point clouds from the DB tree
	void setAvailableTrajectories(const std::vector<ccPointCloud*>& clouds, ccPointCloud* excludeCloud = nullptr);

	std::array<unsigned int, 3> getLatticeSize() const;
	DeformationType getDeformationType() const;
	size_t getPreviewPointCount() const;
	//! Returns the selected trajectory cloud (nullptr if "None" is selected)
	ccPointCloud* getSelectedTrajectory() const;
	//! Returns the Z-axis rotation angle in degrees
	double getRotationAngleDeg() const;

	//! Pre-populate dialog with existing lattice parameters
	void setInitialValues(const std::array<unsigned int, 3>& dims, DeformationType deformType, double rotationDeg);

signals:
	//! Emitted when any lattice geometry parameter changes (dims or rotation)
	void parametersChanged();

private:
	QSpinBox* m_spinX;
	QSpinBox* m_spinY;
	QSpinBox* m_spinZ;
	QComboBox* m_deformTypeCombo;
	QSpinBox* m_spinPreviewPoints;
	QDoubleSpinBox* m_spinRotation;
	QComboBox* m_trajectoryCombo;
	std::vector<ccPointCloud*> m_trajectories;
};
