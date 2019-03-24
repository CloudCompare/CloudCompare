//##########################################################################
//#                                                                        #
//#                    CLOUDCOMPARE PLUGIN: qRANSAC_SD                     #
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

#ifndef BDR_FACETFILTER_DLG_HEADER
#define BDR_FACETFILTER_DLG_HEADER

//Local
#include "ccOverlayDialog.h"
#include "ccPickingListener.h"

//qCC_db
#include <ccGLMatrix.h>
//qCC_gl
#include <ccGLUtils.h>

//system
#include <map>

class QMdiSubWindow;
class ccGLWindow;
class ccHObject;
class ccPickingHub;

namespace Ui
{
	class BDRFacetFilterDlg;
}

#include "ui_bdrFacetFilterDlg.h"
#include "mainwindow.h"

//! Dialog for facetfilter plugin

class bdrFacetFilterDlg : public QDialog, Ui::BDRFacetFilterDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrFacetFilterDlg(QWidget* parent = 0);

	//! Destructor
	~bdrFacetFilterDlg() override;

public slots:
	void iDistThresholdChanged(int);
	void iConfThresholdChanged(int);
	void dDistThresholdChanged(double);
	void dConfThresholdChanged(double);
	void CheckModel();
	void Restore();
		
public:
	//! Inits dialog values with specified window
	void initWith(ccGLWindow* win, ccHObject::Container _facet);
	std::map<ccHObject*, bool> m_initialstate;
	std::map<ccHObject*, bool> m_oldstate;

private:
	ccHObject::Container m_facetObjs;
	ccGLWindow* m_win;
};

#endif
