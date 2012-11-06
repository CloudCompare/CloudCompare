//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author:: dgm                                                            $
//$Rev:: 2239                                                              $
//$LastChangedDate:: 2012-09-20 00:14:45 +0200 (jeu., 20 sept. 2012)       $
//**************************************************************************
//

#ifndef CC_COMPARISON_DIALOG_HEADER
#define CC_COMPARISON_DIALOG_HEADER

#include <QDialog>
#include <QString>

#include <DgmOctree.h>

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
    typedef enum
    {
        CLOUDCLOUD_DIST = 0,
        CLOUDMESH_DIST  = 1,
    } CC_COMPARISON_TYPE;

    //! Default constructor
	ccComparisonDlg(ccHObject* compEntity, ccHObject* refEntity, CC_COMPARISON_TYPE cpType, QWidget* parent = 0);

	//! Default destructor
	~ccComparisonDlg();

protected slots:
    void showHisto();
    void compute();
	void split3DCheckboxToggled(bool);
    void applyAndExit();
    void cancelAndExit();
	void locaModelChanged(int);
    //! automatically updates best octree level guess when maxDist parameter changes
    void updateOctreeLevel(double);
    //! sets whether to guess automatically best octree level
    void guessBestOctreeLevel(int);

protected:

    bool isValid();
    bool prepareEntitiesForComparison();
    void computeApproxResults();
    int determineBestOctreeLevelForDistanceComputation(DistanceType maxDist);
    void clean();
    void updateDisplay(bool showSF, bool hideRef);

    ccHObject *compEnt,*refEnt;
    CCLib::DgmOctree *compOctree,*refOctree;
    ccPointCloud* compCloud;
    ccGenericPointCloud* refCloud;
    ccGenericMesh* refMesh;
    CC_COMPARISON_TYPE compType;

    //! last computed scalar field name
    QString sfName;

    bool refVisibility;
    bool compSFVisibility;
    QString oldSfName;
};

#endif
