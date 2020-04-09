//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qHoughNormals               #
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

#ifndef QHOUGH_NORMALS_HEADER
#define QHOUGH_NORMALS_HEADER

#include <QDialog>

namespace Ui {
	class HoughNormalsDialog;
}

class qHoughNormalsDialog : public QDialog
{
public:

	//! Default constructor
	explicit qHoughNormalsDialog( QWidget* parent = nullptr );
	
	~qHoughNormalsDialog();

	//Settings
	struct Parameters
	{
		int K = 100;
		int T = 1000;
		int n_phi = 15;
		int n_rot = 5;
		bool use_density = false;
		float tol_angle_rad = 0.79f;
		int k_density = 5;
	};
	
	void setParameters( const Parameters& params );
	void getParameters( Parameters& params );
	
private:
	Ui::HoughNormalsDialog* m_ui;
};

#endif //QHOUGH_NORMALS_HEADER
