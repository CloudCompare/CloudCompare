//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qTreeIso                          #
//#                                                                                     #
//#        This program is free software; you can redistribute it and/or modify         #
//#        it under the terms of the GNU General Public License as published by         #
//#        the Free Software Foundation; version 2 or later of the License.             #
//#                                                                                     #
//#        This program is distributed in the hope that it will be useful,              #
//#        but WITHOUT ANY WARRANTY; without even the implied warranty of               #
//#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
//#        GNU General Public License for more details.                                 #
//#                                                                                     #
//#        Please cite the following paper if you find this tool helpful                #
//#                                                                                     #
//#        Xi, Z.; Hopkinson, C. 3D Graph-Based Individual-Tree Isolation (Treeiso)     #
//#        from Terrestrial Laser Scanning Point Clouds. Remote Sens. 2022, 14, 6116.   #
//#        https://doi.org/10.3390/rs14236116                                           #
//#                                                                                     #
//#		   Our work relies on the cut-pursuit algorithm, please also consider citing:   #
//#        Landrieu, L.; Obozinski, G. Cut Pursuit: Fast Algorithms to Learn Piecewise  #
//#        Constant Functions on General Weighted Graphs. SIAM J. Imaging Sci.          #
//#        2017, 10, 1724–1766.                                                         #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#                  Artemis Lab, Department of Geography & Environment                 #
//#                            University of Lethbridge, Canada                         #
//#                                                                                     #
//#                                                                                     #
//#                           Zhouxin Xi and Chris Hopkinson;                           #
//#                    truebelief2010@gmail.com; c.hopkinson@uleth.ca                   #
//#                                                                                     #
//#######################################################################################


#pragma once
// A Matlab version shared via:
// https://github.com/truebelief/artemis_treeiso

//system
#include <vector>
#include <string>
#include <QProgressDialog>


class ccMainAppInterface;
class ccPointCloud;
class QWidget;
class ccMesh;


class TreeIso
{
public:

	//! Parameters
	TreeIso();

	void setProgressDialog(QProgressDialog* qProgress);

	virtual ~TreeIso() = default;
	bool LoadPcd(ccMainAppInterface* app/*=nullptr*/);
	bool init_seg(const unsigned min_nn1, const float regStrength1, const float PR_DECIMATE_RES1, ccMainAppInterface* app/*=nullptr*/, QWidget* parent/*=nullptr*/);
	bool intermediate_seg(const unsigned PR_MIN_NN2, const float PR_REG_STRENGTH2, const float PR_DECIMATE_RES2, const float PR_MAX_GAP, ccMainAppInterface* app/*=nullptr*/, QWidget* parent/*=nullptr*/);
	bool final_seg(const unsigned PR_MIN_NN3, const float PR_REL_HEIGHT_LENGTH_RATIO, const float PR_VERTICAL_WEIGHT, ccMainAppInterface* app/*=nullptr*/, QWidget* parent/*=nullptr*/);

	bool init_seg_pcd(ccPointCloud* pc, const unsigned min_nn1, const float regStrength1, const float PR_DECIMATE_RES1);
	bool intermediate_seg_pcd(ccPointCloud* pc, const unsigned PR_MIN_NN2, const float PR_REG_STRENGTH2, const float PR_DECIMATE_RES2, const float PR_MAX_GAP);
	bool final_seg_pcd(ccPointCloud* pc, const unsigned PR_MIN_NN3, const float PR_REL_HEIGHT_LENGTH_RATIO, const float PR_VERTICAL_WEIGHT);


	template <
		class result_t = std::chrono::milliseconds,
		class clock_t = std::chrono::steady_clock,
		class duration_t = std::chrono::milliseconds
	>
		auto since(std::chrono::time_point<clock_t, duration_t> const& start)
	{
		return std::chrono::duration_cast<result_t>(clock_t::now() - start);
	}

private:
	QProgressDialog* m_progress;
};
