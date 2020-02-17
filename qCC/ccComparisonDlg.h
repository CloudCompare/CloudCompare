//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_COMPARISON_DIALOG_HEADER
#define CC_COMPARISON_DIALOG_HEADER

//qCC_db
#include <ccOctree.h>

//Qt
#include <QDialog>
#include <QString>

#include <ui_comparisonDlg.h>

class ccHObject;
class ccPointCloud;
class ccGenericPointCloud;
class ccGenericMesh;

//! Dialog for cloud/cloud or cloud/mesh comparison setting
class ccComparisonDlg: public QDialog, public Ui::ComparisonDialog
{
	Q_OBJECT

public:

	//! Comparison type
	enum CC_COMPARISON_TYPE
	{
		CLOUDCLOUD_DIST = 0,
		CLOUDMESH_DIST  = 1,
	};

	//! Default constructor
	ccComparisonDlg(ccHObject* compEntity,
					ccHObject* refEntity,
					CC_COMPARISON_TYPE cpType,
					QWidget* parent = 0,
					bool noDisplay = false);

	//! Default destructor
	~ccComparisonDlg();

	//! Should be called once after the dialog is created
	inline bool initDialog() { return computeApproxDistances(); }

	//! Returns compared entity
	ccHObject* getComparedEntity() const { return m_compEnt; }
	//! Returns compared entity
	ccHObject* getReferenceEntity() { return m_refEnt; }

public slots:
	bool computeDistances();
	void applyAndExit();
	void cancelAndExit();

protected slots:
	void showHisto();
	void locaModelChanged(int);
	void maxDistUpdated();

protected:

	bool isValid();
	bool prepareEntitiesForComparison();
	bool computeApproxDistances();
	int getBestOctreeLevel();
	int determineBestOctreeLevel(double);
	void updateDisplay(bool showSF, bool hideRef);
	void releaseOctrees();

	//! Compared entity
	ccHObject* m_compEnt;
	//! Compared entity equivalent cloud
	ccPointCloud* m_compCloud;
	//! Compared entity's octree
	ccOctree::Shared m_compOctree;
	//! Whether the compared entity octree is partial or not
	bool m_compOctreeIsPartial;
	//! Initial compared entity visibility
	bool m_compSFVisibility;

	//! Reference entity
	ccHObject* m_refEnt;
	//! Reference entity equivalent cloud (if any)
	ccGenericPointCloud* m_refCloud;
	//! Reference entity equivalent mesh (if any)
	ccGenericMesh* m_refMesh;
	//! Reference entity's octree
	ccOctree::Shared m_refOctree;
	//! Whether the reference entity octree is partial or not
	bool m_refOctreeIsPartial;
	//! Initial reference entity visibility
	bool m_refVisibility;

	//! Comparison type
	CC_COMPARISON_TYPE m_compType;

	//! last computed scalar field name
	QString m_sfName;

	//! Initial SF name enabled on the compared entity
	QString m_oldSfName;

	//! Whether a display is active (and should be refreshed) or not
	bool m_noDisplay;

	//! Best octree level (or 0 if none has been guessed already)
	int m_bestOctreeLevel;
};

#endif
