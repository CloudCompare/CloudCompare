//##########################################################################
//#                                                                        #
//#                  CLOUDCOMPARE PLUGIN: qCSVMatrixIO                     #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#include "CSVMatrixFilter.h"

//Local
#include "CSVMatrixOpenDialog.h"

//Qt
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccHObject.h>
#include <ccMaterial.h>
#include <ccMaterialSet.h>
#include <ccLog.h>

//System
#include <string.h>
#include <assert.h>

//semi-persistent parameters
static QChar s_separator(',');
static double s_xSpacing = 1.0;
static double s_ySpacing = 1.0;
static bool s_inverseRows = false;
static bool s_loadAsMesh = false;
static bool s_useTexture = false;

CSVMatrixFilter::CSVMatrixFilter()
    : FileIOFilter( {
                    "_CSV Matrix Filter",
					DEFAULT_PRIORITY,	// priority
                    QStringList{ "csv" },
                    "csv",
                    QStringList{ "CSV matrix cloud (*.csv)" },
                    QStringList(),
                    Import
                    } )
{
}

CC_FILE_ERROR CSVMatrixFilter::loadFile(const QString& filename,
									ccHObject& container,
									LoadParameters& parameters)
{
	CSVMatrixOpenDialog openDlg(nullptr);
	openDlg.lineEditSeparator->setText(s_separator);
	openDlg.xDoubleSpinBox->setValue(s_xSpacing);
	openDlg.yDoubleSpinBox->setValue(s_ySpacing);
	openDlg.inverseRowCheckBox->setChecked(s_inverseRows);
	openDlg.loadAsMeshCheckBox->setChecked(s_loadAsMesh);
	openDlg.useTextureCheckBox->setChecked(s_useTexture);

	if (!openDlg.exec())
	{
		return CC_FERR_CANCELED_BY_USER;
	}

	if (openDlg.lineEditSeparator->text().isEmpty())
	{
		ccLog::Warning(QString("[CSVMatrixFilter] Invalid separator!"));
		return CC_FERR_BAD_ARGUMENT;
	}

	s_separator = openDlg.lineEditSeparator->text().at(0);
	s_xSpacing = openDlg.xDoubleSpinBox->value();
	s_ySpacing = openDlg.yDoubleSpinBox->value();
	s_inverseRows = openDlg.inverseRowCheckBox->isChecked();
	s_loadAsMesh = openDlg.loadAsMeshCheckBox->isChecked();
	s_useTexture = openDlg.useTextureCheckBox->isChecked();

	QFile file(filename);
	if (!file.open(QFile::ReadOnly | QFile::Text))
		return CC_FERR_READING;

	QTextStream stream(&file);

	unsigned lineIndex = 0;
	int width = -1;
	int row = 0;
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	ccPointCloud* cloud = new ccPointCloud();
	while (file.error() == QFile::NoError && !file.atEnd())
	{
		QString line = stream.readLine();
		++lineIndex;

		//skip comments
		if (line.startsWith("//"))
			continue;

		if (line.size() == 0)
		{
			ccLog::Warning(QString("[CSVMatrixFilter] Line %1 is empty (ignored)").arg(lineIndex));
			continue;
		}

		//we split the current line
		QStringList parts = line.split(s_separator,QString::SkipEmptyParts);

		//if we have reached the max. number of points per cloud
		if (width < 0)
		{
			width = parts.size();
			ccLog::Print(QString("[CSVMatrixFilter] Detected width: %1").arg(width));
		}
		else if (width != parts.size())
		{
			ccLog::Warning(QString("[CSVMatrixFilter] Line %1 has not the same width as the previous ones!").arg(lineIndex));
			result = CC_FERR_MALFORMED_FILE;
			break;
		}

		//reserve memory for next row
		if (!cloud->reserve(cloud->size() + static_cast<unsigned>(width)))
		{
			result = CC_FERR_NOT_ENOUGH_MEMORY;
			break;
		}

		bool ok = true;
		CCVector3 P(0, static_cast<PointCoordinateType>((s_inverseRows ? -row : row) * s_ySpacing), 0);
		for (int i=0; i<width; ++i)
		{
			P.z = parts[i].toDouble(&ok);
			if (!ok)
				break;
			P.x = static_cast<PointCoordinateType>(i * s_xSpacing);

			cloud->addPoint(P);
		}
		++row;
	}
	unsigned rowCount = static_cast<unsigned>(row);
	unsigned colCount = static_cast<unsigned>(width);

	file.close();

	if (result == CC_FERR_NO_ERROR)
	{
		//load as mesh
		ccMesh* mesh = nullptr;
		if (s_loadAsMesh)
		{
			cloud->setName("vertices");
			mesh = new ccMesh(cloud);
			
			unsigned triCount = (colCount-1) * (rowCount-1) * 2;
			if (!mesh->reserve(triCount))
			{
				delete mesh;
				mesh = nullptr;
			}
			else
			{
				mesh->addChild(cloud);
				cloud->setEnabled(false);
				//add triangles
				for (unsigned j=0; j<rowCount-1; ++j)
				{
					unsigned index = j*colCount;
					for (unsigned i=0; i<colCount-1; ++i)
					{
						mesh->addTriangle(index+i,index+colCount+i,index+i+1);
						mesh->addTriangle(index+i+1,index+colCount+i,index+colCount+i+1);
					}
				}
				//compute normals
				if (mesh->computePerVertexNormals())
				{
					mesh->showNormals(true);
				}
				else
				{
					ccLog::Warning(QString("[CSVMatrixFilter] Failed to compute normals on mesh (not enough memory?)"));
				}
			}
		}

		//texture
		if (s_useTexture)
		{
			QString filename = openDlg.textureFilenameLineEdit->text();
			QImage texture;
			if (!texture.load(filename))
			{
				ccLog::Warning(QString("[CSVMatrixFilter] Failed to load texture from file '%1'").arg(filename));
			}
			else
			{
				if (mesh)
				{
					TextureCoordsContainer* texCoords = new TextureCoordsContainer();
					if (	texCoords->reserveSafe(cloud->size())
						&&	mesh->reservePerTriangleTexCoordIndexes()
						&&	mesh->reservePerTriangleMtlIndexes())
					{
						//generate texture coordinates
						{
							for (unsigned j = 0; j < rowCount; ++j)
							{
								TexCoords2D coord(0, static_cast<float>(j) / (rowCount - 1));
								for (unsigned i = 0; i < colCount; ++i)
								{
									coord.tx = static_cast<float>(i) / (colCount - 1);
									texCoords->addElement(coord);
								}
							}
						}
						mesh->setTexCoordinatesTable(texCoords);

						//create material
						ccMaterial::Shared mat(new ccMaterial("texture"));
						mat->setTexture(texture,filename,false);
						ccMaterialSet* matSet = new ccMaterialSet("Materials");
						matSet->push_back(mat);
						mesh->setMaterialSet(matSet);

						//assign texture coordinaetes and material to each triangle
						for (unsigned i = 0; i < mesh->size(); ++i)
						{
							CCLib::VerticesIndexes* tsi = mesh->getTriangleVertIndexes(i);
							mesh->addTriangleTexCoordIndexes(tsi->i1, tsi->i2, tsi->i3);
							mesh->addTriangleMtlIndex(0);
						}

						mesh->showMaterials(true);
					}
					else
					{
						ccLog::Warning("[CSVMatrixFilter] Not enough memory to map the texture on the mesh!");
						texCoords->release();
						texCoords = nullptr;
					}
				}
				else
				{
					if (	texture.width() < static_cast<int>(colCount)
						||	texture.height() < static_cast<int>(rowCount) )
					{
						ccLog::Warning(QString("[CSVMatrixFilter] To map a texture on the cloud, the image size should equal or greater than the grid size (%1x%2)").arg(colCount).arg(rowCount));
					}
					else if (cloud->reserveTheRGBTable())
					{
						for (unsigned j = 0; j < rowCount; ++j)
						{
							for (unsigned i = 0; i < colCount; ++i)
							{
								QRgb col = texture.pixel(static_cast<int>(i), static_cast<int>(j));
								cloud->addColor(ccColor::FromQRgb(col));
							}
						}
						cloud->showColors(true);
					}
					else
					{
						ccLog::Warning("[CSVMatrixFilter] Not enough memory to map the texture on the cloud!");
					}
				}
			}
		}


		if (s_inverseRows)
		{
			CCVector3 T(0,static_cast<PointCoordinateType>((rowCount-1) * s_ySpacing),0);
			ccGLMatrix trans;
			trans.setTranslation(T);
			cloud->applyGLTransformation_recursive(&trans);
			//this transformation is of no interest for the user
			cloud->resetGLTransformationHistory_recursive();
		}
		ccLog::Print(QString("[CSVMatrixFilter] Number of rows: %1").arg(rowCount));
		cloud->setVisible(true);
		container.addChild(mesh ? static_cast<ccHObject*>(mesh) : static_cast<ccHObject*>(cloud));
	}
	else
	{
		delete cloud;
		cloud = nullptr;
	}

	return result;
}
