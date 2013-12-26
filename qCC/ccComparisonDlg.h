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

public slots:

	bool compute();
    void applyAndExit();
    void cancelAndExit();

protected slots:
    void showHisto();
	void split3DCheckboxToggled(bool);
	void locaModelChanged(int);

protected:

    bool isValid();
    bool prepareEntitiesForComparison();
    int computeApproxResults();
    void updateOctreeLevel(double);
    int determineBestOctreeLevel(double);
    void updateDisplay(bool showSF, bool hideRef);
    void clean();

    ccHObject *m_compEnt,*m_refEnt;
    CCLib::DgmOctree *m_compOctree,*m_refOctree;
    ccPointCloud* m_compCloud;
    ccGenericPointCloud* m_refCloud;
    ccGenericMesh* m_refMesh;
    CC_COMPARISON_TYPE m_compType;

    //! last computed scalar field name
    QString m_sfName;

	//initial state
    bool m_refVisibility;
    bool m_compSFVisibility;
    QString m_oldSfName;

	//! Whether the current SF is a distance field or not
	bool m_currentSFIsDistance;

	//! Whether a display is active (and should be refreshed) or not
	bool m_noDisplay;
};

#endif
