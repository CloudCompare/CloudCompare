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

#ifndef CC_BUNDLER_IMPORT_DIALOG_HEADER
#define CC_BUNDLER_IMPORT_DIALOG_HEADER

#include <QDialog>

//local
#include "qCC_io.h"

#include "ui_openBundlerFileDlg.h"

class ccGLMatrix;

//! Dialog for importation of Snavely's Bundler files
class /*QCC_IO_LIB_API*/ BundlerImportDlg : public QDialog, public Ui::BundlerImportDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit BundlerImportDlg(QWidget* parent = 0);

	//! Destructor
	virtual ~BundlerImportDlg() = default;

	//! Returns whether keypoints should be imported
	bool importKeypoints() const;
	//! Returns whether alternative keypoints should be used
	bool useAlternativeKeypoints() const;
	//! Returns whether images should be imported
	bool importImages() const;
	//! Returns whether images should be undistorted
	bool undistortImages() const;
	//! Returns whether images should be ortho-rectified as clouds
	bool orthoRectifyImagesAsClouds() const;
	//! Returns whether images should be ortho-rectified as images
	bool orthoRectifyImagesAsImages() const;
	//! Returns whether colored pseudo-DTM should be generated
	bool generateColoredDTM() const;
	//! Returns images should be kept in memory or not
	bool keepImagesInMemory() const;

	//! Image ortho-rectification methods
	enum OrthoRectMethod { OPTIMIZED, DIRECT_UNDISTORTED, DIRECT };
	//! Returns the ortho-rectification method (for images)
	OrthoRectMethod getOrthorectificationMethod() const;

	//! Sets keypoints count on initialization
	void setKeypointsCount(unsigned count);
	//! Sets cameras count on initialization
	void setCamerasCount(unsigned count);
	//! Sets file version on initialization
	void setVer(unsigned majorVer, unsigned minorVer);

	//! Sets default image list filename (full path)
	void setImageListFilename(const QString& filename);
	//! Gets image list filename (full path)
	QString getImageListFilename() const;

	//! Sets default alternative keypoints filename (full path)
	void setAltKeypointsFilename(const QString& filename);
	//! Gets alternative keypoints filename (full path)
	QString getAltKeypointsFilename() const;

	//! Returns scale factor
	double getScaleFactor() const;

	//! Returns desired number of vertices for DTM
	unsigned getDTMVerticesCount() const;

	//! Returns the optional transformation matrix (if defined)
	bool getOptionalTransfoMatrix(ccGLMatrix& mat);

protected slots:
	void browseImageListFilename();
	void browseAltKeypointsFilename();
	void acceptAndSaveSettings();

protected:

	//! Inits dialog state from persistent settings
	void initFromPersistentSettings();

	//! Saves dialog state from persistent settings
	void saveToPersistentSettings();

};

#endif //CC_BUNDLER_IMPORT_DIALOG_HEADER
