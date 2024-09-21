//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 3 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################

#include "qVoxFallDialog.h"

//CCPluginAPI
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//qCC_db
#include <ccMesh.h>

//Qt
#include <QMainWindow>
#include <QComboBox>


/*** HELPERS ***/
static QString GetEntityName(ccHObject* obj)
{
	if (!obj)
	{
		assert(false);
		return QString();
	}

	QString name = obj->getName();
	if (name.isEmpty())
		name = "unnamed";
	name += QString(" [ID %1]").arg(obj->getUniqueID());

	return name;
}

static ccMesh* GetMeshFromCombo(QComboBox* comboBox, ccHObject* dbRoot)
{
	assert(comboBox && dbRoot);
	if (!comboBox || !dbRoot)
	{
		assert(false);
		return nullptr;
	}

	//return the mesh currently selected in the combox box
	int index = comboBox->currentIndex();
	if (index < 0)
	{
		assert(false);
		return nullptr;
	}
	assert(comboBox->itemData(index).isValid());
	unsigned uniqueID = comboBox->itemData(index).toUInt();
	ccHObject* item = dbRoot->find(uniqueID);
	if (!item || !item->isA(CC_TYPES::MESH))
	{
		assert(false);
		return nullptr;
	}
	return static_cast<ccMesh*>(item);
}

/*** HELPERS (END) ***/

qVoxFallDialog::qVoxFallDialog(ccMesh* mesh1, ccMesh* mesh2, ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr)
	, Ui::VoxFallDialog()
	, m_app(app)
	, m_mesh1(nullptr)
	, m_mesh2(nullptr)
{
	setupUi(this);

	connect(showMesh1CheckBox,		&QAbstractButton::toggled,	this, &qVoxFallDialog::setMesh1Visibility);
	connect(showMesh2CheckBox,		&QAbstractButton::toggled,	this, &qVoxFallDialog::setMesh2Visibility);

	connect(swapMeshesToolButton,	&QAbstractButton::clicked,	this, &qVoxFallDialog::swapMeshes);

	setMeshes(mesh1, mesh2);

	loadParamsFromPersistentSettings();
}

void qVoxFallDialog::swapMeshes()
{
	setMeshes(m_mesh2, m_mesh1);
}

void qVoxFallDialog::setMeshes(ccMesh* mesh1, ccMesh* mesh2)
{
	if (!mesh1 || !mesh2)
	{
		assert(false);
		return;
	}

	m_mesh1 = mesh1;
	m_mesh2 = mesh2;

	//mesh #1
	mesh1LineEdit->setText(GetEntityName(mesh1));
	showMesh1CheckBox->blockSignals(true);
	showMesh1CheckBox->setChecked(mesh1->isVisible());
	showMesh1CheckBox->blockSignals(false);

	//mesh #2
	mesh2LineEdit->setText(GetEntityName(mesh2));
	showMesh2CheckBox->blockSignals(true);
	showMesh2CheckBox->setChecked(mesh2->isVisible());
	showMesh2CheckBox->blockSignals(false);
}

void qVoxFallDialog::setMesh1Visibility(bool state)
{
	if (m_mesh1)
	{
		m_mesh1->setVisible(state);
		m_mesh1->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}

void qVoxFallDialog::setMesh2Visibility(bool state)
{
	if (m_mesh2)
	{
		m_mesh2->setVisible(state);
		m_mesh2->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}

double qVoxFallDialog::getVoxelSize() const
{
	double voxelSize = voxelSizeDoubleSpinBox->value();
	return voxelSize;
}

double qVoxFallDialog::getAzimuth() const
{
	double azimuth = azDoubleSpinBox->value();
	return azimuth;
}

bool qVoxFallDialog::getExportMeshesActivation() const
{
	return exportCheckBox->isChecked();
}

bool qVoxFallDialog::getLossGainActivation() const
{
	return lossCheckBox->isChecked();
}

int qVoxFallDialog::getMaxThreadCount() const
{
	return QThread::idealThreadCount();
}

void qVoxFallDialog::loadParamsFromPersistentSettings()
{
	QSettings settings("qVoxFall");
	loadParamsFrom(settings);
}

void qVoxFallDialog::loadParamsFrom(const QSettings& settings)
{
	//read parameters
	double voxelSize = settings.value("VoxelSize", voxelSizeDoubleSpinBox->value()).toDouble();
	double azimuth = settings.value("Azimuth", azDoubleSpinBox->value()).toDouble();
	bool exportMeshesEnabled = settings.value("ExportMeshesEnabled", exportCheckBox->isChecked()).toBool();
	bool lossGainEnabled = settings.value("LossGainEnabled", lossCheckBox->isChecked()).toBool();

	//apply parameters
	voxelSizeDoubleSpinBox->setValue(voxelSize);
	azDoubleSpinBox->setValue(azimuth);
	exportCheckBox->setChecked(exportMeshesEnabled);
	lossCheckBox->setChecked(lossGainEnabled);
}

void qVoxFallDialog::saveParamsToPersistentSettings()
{
	QSettings settings("qVoxFall");
	saveParamsTo(settings);
}

void qVoxFallDialog::saveParamsTo(QSettings& settings)
{
	//save parameters
	settings.setValue("VoxelSize", voxelSizeDoubleSpinBox->value());
	settings.setValue("Azimuth", azDoubleSpinBox->value());
	settings.setValue("ExportMeshesEnabled", exportCheckBox->isChecked());
	settings.setValue("LossGainEnabled", lossCheckBox->isChecked());
}
