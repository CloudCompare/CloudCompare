//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qRDBIO                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: RIEGL Laser Measurement Systems GmbH               #
//#                                                                        #
//##########################################################################

#include "RDBFilter.h"

//Local
#include "RDBOpenDialog.h"

//Qt
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QIcon>
#include <QComboBox>

//CClib
#include <ScalarField.h>

//qCC_db
#include <ccPlane.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccMesh.h>
#include <ccHObject.h>
#include <ccMaterial.h>
#include <ccMaterialSet.h>
#include <ccLog.h>
#include <ccScalarField.h>

//RDB
#include <riegl/rdb.hpp>

//System
#include <string.h>
#include <assert.h>
#include <sstream>
#include <array>


RDBFilter::RDBFilter()
	: FileIOFilter( {
					"RDB 2.0 Filter",
					DEFAULT_PRIORITY,	//priority
					QStringList{ "rdbx", "vxls", "ptch", "mtch" },
					"RDB2",
					QStringList{ "RDB Pointcloud file (*.rdbx)",
						"RDB Voxelfile (*.vxls)",
						"RDB Plane Patch file (*.ptch)",
						"RDB Match file (*.mtch)" },
					QStringList(),
					Import
					} )
{
}

namespace //local helper
{

void updateTable(RDBOpenDialog &openDlg, riegl::rdb::Pointcloud &rdb)
{
	static const QIcon xIcon		(QString::fromUtf8(":/CC/images/typeXCoordinate.png"));
	static const QIcon yIcon		(QString::fromUtf8(":/CC/images/typeYCoordinate.png"));
	static const QIcon zIcon		(QString::fromUtf8(":/CC/images/typeZCoordinate.png"));
	static const QIcon NormIcon		(QString::fromUtf8(":/CC/images/typeNormal.png"));
	static const QIcon RGBIcon		(QString::fromUtf8(":/CC/images/typeRgbCcolor.png"));
	static const QIcon GreyIcon		(QString::fromUtf8(":/CC/images/typeGrayColor.png"));
	static const QIcon ScalarIcon	(QString::fromUtf8(":/CC/images/typeSF.png"));

	//checks if fields are available
	bool rdb_hasRGBA = false;
	bool rdb_hasNormals = false;
	bool rdb_has_pca_axis_min = false;

	{
		int idx = 0;
		//Get list of point attributes
		std::vector<std::string> attributes = rdb.pointAttribute().list();
		openDlg.rdbTableWidget->setColumnCount(2);
		openDlg.rdbTableWidget->setRowCount(static_cast<int>(attributes.size()));
		openDlg.rdbTableWidget->setColumnWidth(0, 200);
		openDlg.rdbTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);

		QStringList headerLabels;
		headerLabels << "Name" << "Load as";
		openDlg.rdbTableWidget->setHorizontalHeaderLabels(headerLabels);

		ccLog::Print(QString("set row count to: %1").arg(attributes.size()));

		//check for existance of attribute and add it to qList
		auto list_known_attribute = [] (RDBOpenDialog &openDlg,
				int &idx, std::vector<std::string> &attributes, const char *att, bool &rdb_has_flag)
		{
			if (std::find(attributes.begin(),attributes.end(), att) != attributes.end())
			{
				rdb_has_flag = true;
				openDlg.rdbTableWidget->setItem(idx, 0, new QTableWidgetItem(att));
				//openDlg.rdbTableWidget->cellWidget(idx,0)->setEnabled(false);
				idx++;
				attributes.erase(std::remove(attributes.begin(), attributes.end(), att), attributes.end());
			}
		};
		auto add_qcombobox_scalar = [] (RDBOpenDialog &openDlg,
				int &idx, std::vector<std::string> &attributes, const std::string &att, bool active)
		{
			if (std::find(attributes.begin(),attributes.end(), att) != attributes.end())
			{
				openDlg.rdbTableWidget->setItem(idx, 0, new QTableWidgetItem(att.c_str()));
				//openDlg.rdbTableWidget->cellWidget(idx,0)->setEnabled(false);
				attributes.erase(std::remove(attributes.begin(), attributes.end(), att), attributes.end());

				QComboBox* columnHeaderWidget = new QComboBox();
				columnHeaderWidget->addItem(RDB_OPEN_DLG_TYPES_NAMES[RDB_OPEN_DLG_None]);
				columnHeaderWidget->addItem(RDB_OPEN_DLG_TYPES_NAMES[RDB_OPEN_DLG_Scalar]);
				columnHeaderWidget->setMaxVisibleItems(2);
				columnHeaderWidget->setItemIcon(1, ScalarIcon);
				if (active)
					columnHeaderWidget->setCurrentIndex(1);
				else
					columnHeaderWidget->setCurrentIndex(0);

				openDlg.rdbTableWidget->setCellWidget(idx,1, columnHeaderWidget);
				idx++;
			}
		};

		bool dummy = false;

		list_known_attribute(openDlg, idx, attributes, "riegl.xyz",           dummy);
		list_known_attribute(openDlg, idx, attributes, "riegl.rgba",          rdb_hasRGBA);
		list_known_attribute(openDlg, idx, attributes, "riegl.surface_normal",rdb_hasNormals);
		list_known_attribute(openDlg, idx, attributes, "riegl.pca_axis_min",  rdb_has_pca_axis_min);

		idx = 0;
		{ //XYZ
			QComboBox* columnHeaderWidget = new QComboBox();
			columnHeaderWidget->addItem(RDB_OPEN_DLG_TYPES_NAMES[RDB_OPEN_DLG_XYZ]);
			columnHeaderWidget->setMaxVisibleItems(1);
			columnHeaderWidget->setCurrentIndex(0);
			columnHeaderWidget->setItemIcon(0, xIcon);
			openDlg.rdbTableWidget->setCellWidget(idx,1, columnHeaderWidget);
			idx++;
		}
		if (rdb_hasRGBA)
		{ //RGB
			QComboBox* columnHeaderWidget = new QComboBox();
			columnHeaderWidget->addItem(RDB_OPEN_DLG_TYPES_NAMES[RDB_OPEN_DLG_RGB]);
			columnHeaderWidget->setMaxVisibleItems(1);
			columnHeaderWidget->setCurrentIndex(0);
			columnHeaderWidget->setItemIcon(0, RGBIcon);
			openDlg.rdbTableWidget->setCellWidget(idx,1, columnHeaderWidget);
			idx++;
		}
		if (rdb_hasNormals)
		{ //normals
			QComboBox* columnHeaderWidget = new QComboBox();
			columnHeaderWidget->addItem(RDB_OPEN_DLG_TYPES_NAMES[RDB_OPEN_DLG_NORM]);
			columnHeaderWidget->setMaxVisibleItems(1);
			columnHeaderWidget->setCurrentIndex(0);
			columnHeaderWidget->setItemIcon(0, NormIcon);
			openDlg.rdbTableWidget->setCellWidget(idx,1, columnHeaderWidget);
			idx++;
		}
		if (rdb_has_pca_axis_min)
		{ //normals
			QComboBox* columnHeaderWidget = new QComboBox();
			columnHeaderWidget->addItem(RDB_OPEN_DLG_TYPES_NAMES[RDB_OPEN_DLG_NORM]);
			columnHeaderWidget->setMaxVisibleItems(1);
			columnHeaderWidget->setCurrentIndex(0);
			columnHeaderWidget->setItemIcon(0, NormIcon);
			openDlg.rdbTableWidget->setCellWidget(idx,1, columnHeaderWidget);
			idx++;
		}
		add_qcombobox_scalar(openDlg, idx, attributes, "riegl.id",            false);
		add_qcombobox_scalar(openDlg, idx, attributes, "riegl.reflectance",   true);
		add_qcombobox_scalar(openDlg, idx, attributes, "riegl.amplitude",     true);
		add_qcombobox_scalar(openDlg, idx, attributes, "riegl.deviation",     true);

		ccLog::Print(QString("unknown attributes: %1").arg(attributes.size()));
		//openDlg.rdbAttributesList->insertItem(idx, "--- unknown attributes ---");
		std::vector<std::string> attr_copy(attributes);
		for (std::string &att : attr_copy)
		{
			add_qcombobox_scalar(openDlg, idx, attributes, att, false);
		}
	}
}

} //end namespace local helper


CC_FILE_ERROR RDBFilter::loadFile( const QString &filename, ccHObject &container, LoadParameters &parameters )
{	
	RDBOpenDialog openDlg(nullptr);

	//New RDB library context
	riegl::rdb::Context context;

	//check for known rdb vector attributes
	bool rdb_hasRGBA = false;
	bool rdb_hasNormals = false;
	bool rdb_has_pca_axis_min = false;

	//needed for plane patch
	//bool rdb_hasNormals = false;
	bool rdb_has_plane_up     = false;
	bool rdb_has_plane_width  = false;
	bool rdb_has_plane_height = false;

	{
		//Access existing database
		riegl::rdb::Pointcloud rdb(context);
		riegl::rdb::pointcloud::OpenSettings settings(context);
		rdb.open(filename.toStdString(), settings);

		//Fill the table with attributes
		updateTable(openDlg, rdb);

		//Get index graph root node
		riegl::rdb::pointcloud::QueryStat stat = rdb.stat();
		riegl::rdb::pointcloud::GraphNode root = stat.index();
		openDlg.rdbPointsNumber->setText(QString("%1").arg(root.pointCountTotal));

		{
			//Get list of point attributes
			std::vector<std::string> attributes = rdb.pointAttribute().list();

			//check for existance of specific rdb attribute
			auto listContains = [] (std::vector<std::string> &attributes, const char *att)
			{
				return std::find(attributes.begin(),attributes.end(), att) != attributes.end();
			};

			rdb_hasRGBA         = listContains(attributes, "riegl.rgba"          );
			rdb_hasNormals      = listContains(attributes, "riegl.surface_normal");
			rdb_has_pca_axis_min= listContains(attributes, "riegl.pca_axis_min"  );
			rdb_has_plane_up    = listContains(attributes, "riegl.plane_up"      );
			rdb_has_plane_width = listContains(attributes, "riegl.plane_width"   );
			rdb_has_plane_height= listContains(attributes, "riegl.plane_height"  );

			if (rdb_hasNormals && rdb_has_pca_axis_min)
			{
				//prefer plane normals
				rdb_has_pca_axis_min = false;
			}
		}
	}

	//open dialog and wait for user to configure
	if (!openDlg.exec())
	{
		return CC_FERR_CANCELED_BY_USER;
	}

	//define needed parameter for scalar fields
	struct Conversion
	{
		//name of the attribute
		std::string att;
		//pointer to scalar field
		ccScalarField *sf;
		//buffer for rdb select
		std::vector<float> buffer;
	};

	//Load the file
	const size_t BUFFER_SIZE = 100000;
	size_t total = 0;

	//point cloud from CC to fill with points from rdb file
	ccPointCloud* cloud = new ccPointCloud();
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;
	//if plane patch attributes are present create plane patch objects
	bool create_planes = false;
	ccHObject *plane_set = nullptr;
	if (rdb_hasNormals && rdb_has_plane_up && rdb_has_plane_width && rdb_has_plane_height)
	{
		create_planes = true;
		cloud->setName("Plane Centers");
		plane_set = new ccHObject();
	}

	//create a list of the scalars
	std::vector<std::string> scalars2load;
	for (int i=0; i<openDlg.rdbTableWidget->rowCount(); ++i)
	{
		QComboBox* cbox = static_cast<QComboBox*>(openDlg.rdbTableWidget->cellWidget(i, 1));
		if (cbox->currentText() == RDB_OPEN_DLG_TYPES_NAMES[RDB_OPEN_DLG_Scalar])
		{
			QString att = openDlg.rdbTableWidget->item(i,0)->text();
			scalars2load.push_back(att.toStdString());
			ccLog::Print(QString("load scalar: %1").arg(att));
		}
	}

	//initialize scalar fields and rdb buffers
	std::vector<Conversion> conversions;
	for (std::string &att : scalars2load)
	{
		Conversion conv;
		conv.att = att;
		conv.sf = new ccScalarField(att.c_str());
		conv.sf->link();
		conv.buffer.resize(BUFFER_SIZE);
		cloud->addScalarField(conv.sf);
		conversions.push_back(conv);
	}

	//if there are rgb values load and show them
	if (rdb_hasRGBA)
	{
		cloud->showColors(true);
	}
	//if there are surface normals load and show them
	if (rdb_hasNormals || rdb_has_pca_axis_min)
	{
		cloud->showNormals(true);
	}
	if (conversions.size() > 0)
	{
		cloud->setCurrentDisplayedScalarField(0);
		cloud->showSF(true);
	}
	{
		//Access existing database
		riegl::rdb::Pointcloud rdb(context);
		riegl::rdb::pointcloud::OpenSettings settings(context);
		rdb.open(filename.toStdString(), settings);
		//Get index graph root node
		riegl::rdb::pointcloud::QueryStat stat = rdb.stat();
		riegl::rdb::pointcloud::GraphNode root = stat.index();

		//progress dialog
		ccProgressDialog pdlg(true, parameters.parentWidget);
		CCLib::NormalizedProgress nprogress(&pdlg, root.pointCountTotal/BUFFER_SIZE);
		{
			std::stringstream ss;
			ss << "Loading RDB file [" << filename.toStdString().c_str()<< "]";
			pdlg.setMethodTitle(ss.str().c_str());
			pdlg.setInfo("Points loaded: 0");
			pdlg.start();
		}

		//prepare point attribute buffers
		std::vector< std::array<float, 3> >   buffer_xyz(BUFFER_SIZE);
		std::vector< std::array<float, 3> >   buffer_normals;
		std::vector< std::array<uint8_t, 4> > buffer_rgba;
		std::vector< std::array<float, 3> >   buffer_plane_up;
		std::vector< float >                  buffer_plane_width;
		std::vector< float >                  buffer_plane_height;
		//reserve memory for optional entries
		if (rdb_hasNormals)
			buffer_normals.resize(BUFFER_SIZE);
		if (rdb_hasRGBA)
			buffer_rgba.resize(BUFFER_SIZE);
		if (rdb_has_plane_up)
			buffer_plane_up.resize(BUFFER_SIZE);
		if (rdb_has_plane_width)
			buffer_plane_width.resize(BUFFER_SIZE);
		if (rdb_has_plane_height)
			buffer_plane_height.resize(BUFFER_SIZE);

		//Start new select query to read all points
		riegl::rdb::pointcloud::QuerySelect select = rdb.select();

		//Bind target data buffers to query
		select.bind("riegl.xyz", riegl::rdb::pointcloud::DataType::SINGLE, buffer_xyz.data());
		if (rdb_hasRGBA)
			select.bind("riegl.rgba", riegl::rdb::pointcloud::DataType::UINT8, buffer_rgba.data());
		if (rdb_hasNormals)
			select.bind("riegl.surface_normal", riegl::rdb::pointcloud::DataType::SINGLE, buffer_normals.data());
		if (rdb_has_pca_axis_min)
			select.bind("riegl.pca_axis_min", riegl::rdb::pointcloud::DataType::SINGLE, buffer_normals.data());
		if (rdb_has_plane_up)
			select.bind("riegl.plane_up", riegl::rdb::pointcloud::DataType::SINGLE, buffer_plane_up.data());

		riegl::rdb::pointcloud::DataType ScalarTypeRiegl = (sizeof(ScalarType) == 4 ? riegl::rdb::pointcloud::DataType::SINGLE : riegl::rdb::pointcloud::DataType::DOUBLE);
		if (rdb_has_plane_width)
			select.bind("riegl.plane_width", ScalarTypeRiegl, buffer_plane_width.data());
		if (rdb_has_plane_height)
			select.bind("riegl.plane_height", ScalarTypeRiegl, buffer_plane_height.data());
		for (Conversion &conv : conversions)
		{
			select.bind(conv.att, ScalarTypeRiegl, conv.buffer.data());
		}

		{ //pre allocate memory for cloud to load
			unsigned total_entries = 0;
			//Start new select query to read all points
			riegl::rdb::pointcloud::QuerySelect count_select = rdb.select();
			while (const uint32_t count = count_select.next(BUFFER_SIZE))
			{
				total_entries += count;
			}
			if (cloud->reserve(total_entries))
			{
				if (rdb_hasRGBA && !cloud->reserveTheRGBTable())
				{
					ccLog::Print(QString("[RDBFilter] Not enough memory to reserve RGB table for points. Try to load as many as possible. Total: %1").arg(total_entries));
					result = CC_FERR_NOT_ENOUGH_MEMORY;
				}
				if ((rdb_hasNormals || rdb_has_pca_axis_min) && !cloud->reserveTheNormsTable())
				{
					ccLog::Print(QString("[RDBFilter] Not enough memory to reserve normal table for points. Try to load as many as possible. Total: %1").arg(total_entries));
					result = CC_FERR_NOT_ENOUGH_MEMORY;
				}
			}
			else //not enough memory for all points
			{
				ccLog::Print(QString("[RDBFilter] Not enough memory to load all points. Try to load as many as possible. Total: %1").arg(total_entries));
				result = CC_FERR_NOT_ENOUGH_MEMORY;
			}
		}

		//Read and process all points block-wise
		while (const uint32_t count = select.next(BUFFER_SIZE))
		{
			//reserve memory for next block (only if we don't have enough memory
			//to hold the complete pointcloud)
			if (result == CC_FERR_NOT_ENOUGH_MEMORY)
			{
				if (!cloud->reserve(cloud->size() + count))
				{
					//can't load more points, exiting
					result = CC_FERR_NOT_ENOUGH_MEMORY;
					break;
				}
				if (rdb_hasRGBA && !cloud->reserveTheRGBTable())
				{
					result = CC_FERR_NOT_ENOUGH_MEMORY;
					break;
				}
				if ((rdb_hasNormals || rdb_has_pca_axis_min) && !cloud->reserveTheNormsTable())
				{
					result = CC_FERR_NOT_ENOUGH_MEMORY;
					break;
				}
			}

			//buffers to hold the CC vectors
			ccColor::Rgb col;
			for (uint32_t i=0; i<count; ++i)
			{
				//read XYZ
				cloud->addPoint(CCVector3::fromArray(buffer_xyz[i].data()));
				//read RGB
				if (rdb_hasRGBA)
				{
					col.r = static_cast<unsigned char>(buffer_rgba[i][0]);
					col.g = static_cast<unsigned char>(buffer_rgba[i][1]);
					col.b = static_cast<unsigned char>(buffer_rgba[i][2]);
					cloud->addColor(col);
				}
				//read normals
				if (rdb_hasNormals || rdb_has_pca_axis_min)
				{
					cloud->addNorm(CCVector3::fromArray(buffer_normals[i].data()));
				}
				//read selected scalars
				for (Conversion &conv : conversions)
				{
					conv.sf->addElement(conv.buffer[i]);
				}

				if (create_planes)
				{
					//calculate tranformation matrix from norm and up vector
					//to define the position and orientation of the plane patch
					CCVector3 plane_norm = CCVector3::fromArray(buffer_normals[i].data());
					CCVector3 plane_up   = CCVector3::fromArray(buffer_plane_up[i].data());
					CCVector3 plane_side = plane_norm.cross(-plane_up);
					Vector3Tpl<float> X(plane_side);
					Vector3Tpl<float> Y(plane_up);
					Vector3Tpl<float> Z(plane_norm);
					Vector3Tpl<float> t(
								static_cast<float>(buffer_xyz[i][0]),
								static_cast<float>(buffer_xyz[i][1]),
								static_cast<float>(buffer_xyz[i][2]));
					ccGLMatrix mat(X, Y, Z, t);
					//create plane patch
					ccPlane *plane = new ccPlane(
								static_cast<PointCoordinateType>(buffer_plane_width[i]),
								static_cast<PointCoordinateType>(buffer_plane_height[i]),
								&mat);
					if (rdb_hasRGBA)
					{
						col.r = static_cast<unsigned char>(buffer_rgba[i][0]);
						col.g = static_cast<unsigned char>(buffer_rgba[i][1]);
						col.b = static_cast<unsigned char>(buffer_rgba[i][2]);
						plane->setColor(col);
					}
					plane->showNormalVector(true); //per default show the normal vector
					plane_set->addChild(plane);
				}
			}
			total += count;

			{
				//update the progress info
				pdlg.setInfo(QString("Points loaded: %1").arg(total));
			}

			if (!nprogress.oneStep())
			{
				//cancel requested
				result = CC_FERR_CANCELED_BY_USER;
				break;
			}
		}
	}

	ccLog::Print(QString("[RDBFilter] Number of points: %1").arg(total));

	//compute min and max of scalars
	for (Conversion &conv : conversions)
	{
		conv.sf->computeMinAndMax();
		conv.sf->release();
	}
	cloud->setVisible(true);
	container.addChild(static_cast<ccHObject*>(cloud));
	if (create_planes)
	{
		container.addChild(plane_set);
	}

	return result;
}

bool RDBFilter::canSave( CC_CLASS_ENUM type, bool &multiple, bool &exclusive ) const
{
	Q_UNUSED( type );
	Q_UNUSED( multiple );
	Q_UNUSED( exclusive );

	//... can we save this?
	return false;
}
