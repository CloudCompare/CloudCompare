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
//$Rev:: 2274                                                              $
//$LastChangedDate:: 2012-10-17 19:17:38 +0200 (mer., 17 oct. 2012)        $
//**************************************************************************
//

#ifndef CC_HEIGHT_GRID_GENERATION_DLG_HEADER
#define CC_HEIGHT_GRID_GENERATION_DLG_HEADER

#include <QString>
#include <ui_heightGridGenerationDlg.h>

#include "ccHeightGridGeneration.h"

//! Height grid generation algorithm dialog
class ccHeightGridGenerationDlg : public QDialog, public Ui::HeightGridGenerationDialog
{
    Q_OBJECT

    public:
        //! Default constructor
        ccHeightGridGenerationDlg(QWidget* parent=0);

        //! Returns projection grid step
        double getGridStep() const;

        //! Returns whether grid should be generated as a cloud
        bool generateCloud() const;

        //! Returns whether grid should be generated as an image
        bool generateImage() const;

        //! Returns whether grid should be generated as an ASCII file
        bool generateASCII() const;

		//! Returns projection dimension
		/** \return dimension as int (0: X, 1: Y, 2:Z)
		**/
        unsigned char getProjectionDimension() const;

        //! Returns type of projection
        ccHeightGridGeneration::ProjectionType getTypeOfProjection() const;

		//! Returns type of SF interpolation
        ccHeightGridGeneration::ProjectionType getTypeOfSFInterpolation() const;

        //! Returns strategy for empty cell filling
        ccHeightGridGeneration::EmptyCellFillOption getFillEmptyCellsStrategy() const;

        //! Returns user defined height for empty cells
        double getCustomHeightForEmptyCells() const;

    protected slots:

        //! Save persistent settings and 'accept' dialog
        void saveSettings();

        //! slot to handle projection type change
        void projectionChanged(int);

        //! updates the "fill empty cells" frame state
        void toggleFillEmptyCells(bool);

    protected:
        //! Load persistent settings
        void loadSettings();
};

#endif
