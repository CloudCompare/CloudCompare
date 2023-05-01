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


//TreeIso
#include "TreeIso.h"
#include "TreeIsoHelper.h"

//CC
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//qCC_db
#include <ccPointCloud.h>
//#include "ccPointCloudInterpolator.h"

//Qt
#include <QProgressDialog>
#include <QCoreApplication>
#include <QElapsedTimer>

//system
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>

//custom
using namespace CP;

#include <qapplication.h>



TreeIso::TreeIso()
{

}

void TreeIso::setProgressDialog(QProgressDialog* qProgress)
{
	m_progress = qProgress;
	//m_progress->hide();
}

bool TreeIso::init_seg_pcd(ccPointCloud* pc, const unsigned min_nn1, const float regStrength1, const float PR_DECIMATE_RES1) {

	ccPointCloud* m_pointCloud = pc;
	const unsigned pointCount = m_pointCloud->size();

	std::vector<std::vector<float>> pc_vec;
	toVec2DTranslate(m_pointCloud, pc_vec);

	std::vector<size_t> ia, ic;
	std::vector<std::vector<float>> pc_dec, pc_sub;
	decimate_vec(pc_vec, PR_DECIMATE_RES1, pc_dec);
	unique_index_by_rows(pc_dec, ia, ic);
	get_subset(pc_vec, ia, pc_sub);
	const unsigned K = (min_nn1 - 1);

	std::vector<std::vector<unsigned int>> nn_idx;
	std::vector<std::vector<float>> nn_D;

	std::vector<uint32_t> in_component;
	std::vector<std::vector<uint32_t>> components;

	std::vector<float> edgeWeight;
	std::vector<uint32_t> Eu;
	std::vector<uint32_t> Ev;

	perform_cut_pursuit(K, regStrength1, pc_sub, edgeWeight, Eu, Ev, in_component, components);

	//export segments as a new scalar field
	char* sfName = "init_segs";
	int outSFIndex = m_pointCloud->getScalarFieldIndexByName(sfName);
	if (outSFIndex < 0)
	{
		outSFIndex = m_pointCloud->addScalarField(sfName);
		if (outSFIndex < 0)
		{
			ccLog::Error("[TreeIso] Not enough memory!");
			return false;
		}
	}
	CCCoreLib::ScalarField* outSF = m_pointCloud->getScalarField(outSFIndex);
	outSF->fill(CCCoreLib::NAN_VALUE);

	std::vector<uint32_t> clusterIdx(pointCount);
	for (unsigned i = 0; i < pointCount; ++i)
	{
		clusterIdx[i] = in_component[ic[i]];
		outSF->setValue(i, in_component[ic[i]]);
	}
	outSF->computeMinAndMax();
	m_pointCloud->colorsHaveChanged();
	m_pointCloud->setCurrentDisplayedScalarField(outSFIndex);
	return true;
}



bool TreeIso::intermediate_seg_pcd(ccPointCloud* pc, const unsigned PR_MIN_NN2, const float PR_REG_STRENGTH2, const float PR_DECIMATE_RES2, const float PR_MAX_GAP)
{
	ccPointCloud* m_pointCloud = pc;
	unsigned pointCount = m_pointCloud->size();

	int initSFIndex = m_pointCloud->getScalarFieldIndexByName("init_segs");
	if (initSFIndex < 0)
	{
		ccLog::Error("[TreeIso] Please run initial segmentation first!");
		return false;
	}
	CCCoreLib::ScalarField* initSF = m_pointCloud->getScalarField(initSFIndex);

	initSF->size();
	std::vector<uint32_t> in_component;
	in_component.resize(pointCount);
	for (int i = 0; i < pointCount; ++i) {
		in_component[i] = initSF->getValue(i);
	}
	std::vector<std::vector<uint32_t>> clusterVGroup;
	unique_group(in_component, clusterVGroup);

	std::vector<std::vector<float>> pc_vec;
	toVec2DTranslate(m_pointCloud, pc_vec);

	int n_clusters = clusterVGroup.size();

	std::vector<std::vector<float>> clusterCentroids(n_clusters);
	std::vector<std::vector<std::vector<float>>> currentClusterDecs(n_clusters);
	std::vector<std::vector<size_t>> currentClusterDecsICs(n_clusters);

	for (unsigned i = 0; i < n_clusters; ++i) {
		std::vector<std::vector<float>> currentClusterPos;
		get_subset(pc_vec, clusterVGroup[i], currentClusterPos);

		if (currentClusterPos.size() > 1)
		{
			std::vector<size_t> ia, ic;
			std::vector<std::vector<float>> currentClusterPosDec;
			decimate_vec(currentClusterPos, PR_DECIMATE_RES2, currentClusterPosDec);
			unique_index_by_rows(currentClusterPosDec, ia, ic);

			std::vector<std::vector<float>> currentClusterPosUnq;
			get_subset(currentClusterPos, ia, currentClusterPosUnq);
			currentClusterDecs[i] = currentClusterPosUnq;

			mean_col(currentClusterPos, clusterCentroids[i]);
			currentClusterDecsICs[i] = ic;
		}
		else {
			currentClusterDecs[i] = currentClusterPos;
			clusterCentroids[i] = currentClusterPos[0];
			currentClusterDecsICs[i] = std::vector<size_t>(1, 0);
		}
	}

	std::vector<std::vector<size_t>> minIdxsC;
	std::vector<std::vector<float>> minIdxsD;
	knn_cpp_nearest_neighbors(clusterCentroids, PR_MIN_NN2, minIdxsC, minIdxsD, 8);

	int n_centroids = minIdxsC.size();
	int n_K = minIdxsC[0].size();
	std::vector<Eigen::MatrixXf> currentClusterDecMats;
	std::vector<knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>> knn_kdtrees;
	for (unsigned i = 0; i < n_centroids; ++i)
	{
		std::vector<std::vector<float>> currentClusterDec = currentClusterDecs[i];
		Eigen::MatrixXf currentClusterDecMat(currentClusterDec[0].size(), currentClusterDec.size());
		for (int k = 0; k < currentClusterDec.size(); k++) {
			currentClusterDecMat.col(k) = Eigen::VectorXf::Map(&currentClusterDec[k][0], currentClusterDec[k].size());
		}
		currentClusterDecMats.push_back(currentClusterDecMat);
	}

	std::vector<std::vector<float>> nnDists;
	nnDists.resize(n_centroids, std::vector<float>(n_K));
	for (unsigned i = 0; i < n_centroids; ++i)
	{
		knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> knn_kdtree(currentClusterDecMats[minIdxsC[i][0]]);
		knn_cpp_build(knn_kdtree);
		for (unsigned j = 1; j < n_K; ++j)
		{
			if (minIdxsD[i][j] > 0)
			{
				std::vector<std::vector<size_t>> min_c;
				std::vector<std::vector<float>> min_D2;
				float min_D = knn_cpp_query_min_d(knn_kdtree, currentClusterDecMats[minIdxsC[i][j]], 1);
				nnDists[i][j] = min_D;
			}
		}

	}

	std::vector<std::vector<float>> currentClusterDecsFlat;
	std::vector<size_t> currentClusterDecsFlatIndex;
	std::vector<size_t> currentClusterDecsICsFlatIndex;
	std::vector<uint32_t> currentClusterDecsIDs;


	for (auto& vec : clusterVGroup) {
		currentClusterDecsIDs.insert(currentClusterDecsIDs.end(), std::make_move_iterator(vec.begin()), std::make_move_iterator(vec.end()));
	}
	for (auto& vec : currentClusterDecs) {
		currentClusterDecsFlat.insert(currentClusterDecsFlat.end(), std::make_move_iterator(vec.begin()), std::make_move_iterator(vec.end()));
		currentClusterDecsICsFlatIndex.insert(currentClusterDecsICsFlatIndex.end(), vec.size());
	}

	for (unsigned i = 0; i < currentClusterDecs.size(); i++) {
		for (unsigned j = 0; j < currentClusterDecs[i].size(); ++j) {
			currentClusterDecsFlatIndex.push_back(i);
		}
	}

	int nNodes = currentClusterDecsFlat.size();
	int nKs = PR_MIN_NN2;

	std::vector<std::vector<size_t>> minIdxs;
	std::vector<std::vector<float>> Ds;

	knn_cpp_nearest_neighbors(currentClusterDecsFlat, PR_MIN_NN2, minIdxs, Ds, 8);

	std::vector<float> edgeWeight;
	std::vector<uint32_t> Eu;
	std::vector<uint32_t> Ev;

	for (int i = 0; i < minIdxs.size(); i++) {
		int currentNode = currentClusterDecsFlatIndex[i];
		std::vector<float> currentDists = nnDists[currentNode];
		for (unsigned j = 1; j < minIdxs[0].size(); ++j) {
			int nnNode = currentClusterDecsFlatIndex[minIdxs[i][j]];
			std::vector<size_t> nnCand = minIdxsC[currentNode];
			auto it = std::find(nnCand.begin(), nnCand.end(), nnNode);
			if (it != nnCand.end())
			{
				float nnDist = currentDists[it - nnCand.begin()];
				if (nnDist < PR_MAX_GAP) {
					Eu.push_back(i);
					Ev.push_back(minIdxs[i][j]);
					edgeWeight.push_back(10 / ((nnDist + 0.1) / 0.01));
				}
			}
		}
	}

	std::vector<uint32_t> in_component2d;
	perform_cut_pursuit2d(PR_MIN_NN2, PR_REG_STRENGTH2, currentClusterDecsFlat, edgeWeight, Eu, Ev, in_component2d);

	std::vector<uint32_t> currentClusterDecsICsReverse;
	std::vector<std::vector<float>> currentClusterDecsReverse;
	int ia_counter = 0;
	for (unsigned i = 0; i < currentClusterDecs.size(); i++) {
		int N = currentClusterDecsICs[i].size();
		std::vector<uint32_t> v(in_component2d.begin() + ia_counter, in_component2d.begin() + ia_counter + currentClusterDecsICsFlatIndex[i]);
		for (unsigned j = 0; j < N; j++) {
			currentClusterDecsICsReverse.push_back(v[currentClusterDecsICs[i][j]]);
		}
		ia_counter += currentClusterDecsICsFlatIndex[i];
	}

	std::vector<size_t> currentClusterDecsIDsSortedIdx;
	std::vector<uint32_t> currentClusterDecsIDsSorted;

	sort_indexes(currentClusterDecsIDs, currentClusterDecsIDsSortedIdx, currentClusterDecsIDsSorted);
	std::vector<uint32_t> currentClusterDecsICsReverseSorted;
	currentClusterDecsICsReverseSorted.reserve(currentClusterDecsICsReverse.size());
	std::transform(currentClusterDecsIDsSortedIdx.begin(), currentClusterDecsIDsSortedIdx.end(), std::back_inserter(currentClusterDecsICsReverseSorted),
		[&](const int& i) { return currentClusterDecsICsReverse[i]; });

	//export segments as a new scalar field
	char* sfName = "intermediate_segs";
	int outSFIndex = m_pointCloud->getScalarFieldIndexByName(sfName);
	if (outSFIndex < 0)
	{
		outSFIndex = m_pointCloud->addScalarField(sfName);
		if (outSFIndex < 0)
		{
			ccLog::Error("[TreeIso] Not enough memory!");
			return false;
		}
	}
	CCCoreLib::ScalarField* outSF = m_pointCloud->getScalarField(outSFIndex);
	outSF->fill(CCCoreLib::NAN_VALUE);

	std::vector<uint32_t> groupIdx(pointCount);
	for (unsigned i = 0; i < pointCount; ++i)
	{
		groupIdx[i] = currentClusterDecsICsReverseSorted[i];
		outSF->setValue(i, currentClusterDecsICsReverseSorted[i]);
	}
	outSF->computeMinAndMax();
	m_pointCloud->colorsHaveChanged();
	m_pointCloud->setCurrentDisplayedScalarField(outSFIndex);
}


bool TreeIso::final_seg_pcd(ccPointCloud* pc, const unsigned PR_MIN_NN3, const float PR_REL_HEIGHT_LENGTH_RATIO, const float PR_VERTICAL_WEIGHT) {

	ccPointCloud* m_pointCloud = pc;
	unsigned pointCount = m_pointCloud->size();
	std::vector<std::vector<float>> pc_vec;
	toVec2DTranslate(m_pointCloud, pc_vec);

	int initIdx = m_pointCloud->getScalarFieldIndexByName("init_segs");
	if (initIdx < 0)
	{
		ccLog::Error("[TreeIso] Please run initial segmentation first!");
		return false;
	}
	int groupIdx = m_pointCloud->getScalarFieldIndexByName("intermediate_segs");
	if (groupIdx < 0)
	{
		ccLog::Error("[TreeIso] Please run intermeditate segmentation first!");
		return false;
	}

	CCCoreLib::ScalarField* initSF = m_pointCloud->getScalarField(initIdx);
	initSF->size();
	std::vector<uint32_t> segs_init_ids;
	segs_init_ids.resize(pointCount);
	for (int i = 0; i < pointCount; ++i) {
		segs_init_ids[i] = initSF->getValue(i);
	}

	std::vector<std::vector<uint32_t>> initVGroup;
	std::vector<uint32_t> initU;
	std::vector<uint32_t> initUI;
	unique_group(segs_init_ids, initVGroup, initU, initUI);


	int n_init_clusters = initVGroup.size();
	std::vector<std::vector<float>> clusterCentroids(n_init_clusters);


	for (int i = 0; i < n_init_clusters; ++i) {
		std::vector<std::vector<float>> clusterPts;
		get_subset(pc_vec, initVGroup[i], clusterPts);
		std::vector<float> clusterCentroid;
		mean_col(clusterPts, clusterCentroid);
		clusterCentroids[i] = clusterCentroid;
	}


	CCCoreLib::ScalarField* groupSF = m_pointCloud->getScalarField(groupIdx);

	groupSF->size();
	std::vector<uint32_t> segs_group_ids;
	segs_group_ids.resize(pointCount);

	for (int i = 0; i < pointCount; ++i) {
		segs_group_ids[i] = groupSF->getValue(i);
	}

	std::vector<uint32_t> cluster_ids;
	get_subset(segs_group_ids, initUI, cluster_ids);

	std::vector<std::vector<uint32_t>> clusterVGroup;
	std::vector<uint32_t> clusterU0;
	unique_group(cluster_ids, clusterVGroup, clusterU0);


	if (clusterVGroup.size() > 1)
	{
		auto start = std::chrono::steady_clock::now();
		int n_clusters = clusterVGroup.size();

		std::vector<int> toMergeIds;
		std::vector<int> prevToMergeIds;
		std::vector<int> mergedRemainIds;


		int n_to_merge_ids = 1;
		int n_prev_merge_ids = -1;

		int iter = 1;

		std::vector<std::vector<uint32_t>> groupVGroup;
		std::vector<uint32_t> groupU;

		std::vector<std::vector<float>> centroid2DFeatures;
		Eigen::MatrixXf groupCentroidsToMerge;
		Eigen::MatrixXf groupCentroidsRemain;

		while ((n_to_merge_ids > 0) & (n_to_merge_ids != n_prev_merge_ids)) {
			if (iter > 1) {
				n_prev_merge_ids = n_to_merge_ids;
			}
			unique_group(segs_group_ids, groupVGroup, groupU);
			int nGroups = groupVGroup.size();
			centroid2DFeatures.resize(nGroups, std::vector<float>(2));

			std::vector<float> zFeatures;
			zFeatures.resize(nGroups);
			std::vector<float> lenFeatures;
			lenFeatures.resize(nGroups);

			std::vector<polygon> groupHulls;
			for (int i = 0; i < nGroups; ++i) {
				std::vector<std::vector<float>> groupPts;
				get_subset(pc_vec, groupVGroup[i], groupPts);
				std::vector<float> groupCentroids;
				mean_col(groupPts, groupCentroids);
				centroid2DFeatures[i][0] = groupCentroids[0];
				centroid2DFeatures[i][1] = groupCentroids[1];

				std::vector<float> minPts;
				min_col(groupPts, minPts);
				zFeatures[i] = minPts[2];
				std::vector<float> maxPts;
				max_col(groupPts, maxPts);
				lenFeatures[i] = maxPts[2] - minPts[2];

				polygon hull;
				multi_point conv_points;
				for (const auto& p : groupPts)
				{
					conv_points.push_back(point_xy(p[0], p[1]));
				}

				boost::geometry::convex_hull(conv_points, hull);
				groupHulls.push_back(hull);
			}


			int knncpp_nn = (PR_MIN_NN3 < n_clusters ? PR_MIN_NN3 : n_clusters);
			std::vector<std::vector<size_t>> groupNNIdxC;
			std::vector<std::vector<float>> groupNNCDs;
			knn_cpp_nearest_neighbors(centroid2DFeatures, knncpp_nn, groupNNIdxC, groupNNCDs, 8);
			std::vector<float> mds;
			mean_col(groupNNCDs, mds);
			float sigmaD = mds[1];

			std::vector<int> toMergeIds;
			std::vector<int> toMergeCandidateIds;
			std::vector<float> toMergeCandidateMetrics;
			for (int i = 0; i < nGroups; ++i)
			{
				std::vector<size_t> nnGroupId = groupNNIdxC[i];
				std::vector<float> nnGroupZ; nnGroupZ.resize(nnGroupId.size());
				std::vector<float> nnGroupLen; nnGroupLen.resize(nnGroupId.size());
				for (int j = 0; j < nnGroupId.size(); ++j)
				{
					nnGroupZ[j] = zFeatures[nnGroupId[j]];
					nnGroupLen[j] = lenFeatures[nnGroupId[j]];
				}
				float minZ = min_col(nnGroupZ);
				float minLen = min_col(nnGroupLen);

				float currentGroupRelHt = (zFeatures[i] - minZ) / lenFeatures[i];

				if (abs(currentGroupRelHt) > PR_REL_HEIGHT_LENGTH_RATIO) {
					if (iter == 1) {
						float initialLenRatio = lenFeatures[i] / median_col(lenFeatures);
						if (initialLenRatio > 1.5) {
							toMergeIds.push_back(i);
						}
						toMergeCandidateIds.push_back(i);
						toMergeCandidateMetrics.push_back(initialLenRatio);
					}
					else {
						toMergeIds.push_back(i);
					}

				}
			}
			if ((iter == 1) & (toMergeCandidateMetrics.size() == 0))
			{
				break;
			}
			if ((iter == 1) & (toMergeIds.size() == 0)) {
				int cand_ind = arg_max_col(toMergeCandidateMetrics);
				toMergeIds.push_back(toMergeCandidateIds[cand_ind]);
			}


			std::vector<int> remainIds;
			std::vector<int> allIds(nGroups);
			std::iota(std::begin(allIds), std::end(allIds), 0);
			std::set_difference(allIds.begin(), allIds.end(), toMergeIds.begin(), toMergeIds.end(), std::inserter(remainIds, remainIds.begin()));


			get_subset(centroid2DFeatures, toMergeIds, groupCentroidsToMerge);
			get_subset(centroid2DFeatures, remainIds, groupCentroidsRemain);
			int knncpp_nn2 = (PR_MIN_NN3 < remainIds.size() ? PR_MIN_NN3 : remainIds.size());


			knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> knn_kdtree(groupCentroidsRemain);
			knn_cpp_build(knn_kdtree);
			std::vector <std::vector<size_t>>groupNNIdx;
			std::vector <std::vector<float>>groupNNIdxDists;

			knn_cpp_query(knn_kdtree, groupCentroidsToMerge, knncpp_nn2, groupNNIdx, groupNNIdxDists);

			int nToMergeIds = toMergeIds.size();
			int nRemainIds = remainIds.size();
			for (int i = 0; i < nToMergeIds; ++i)
			{
				int toMergeId = toMergeIds[i];

				Eigen::MatrixXf currentClusterCentroids;
				get_subset(clusterCentroids, clusterVGroup[toMergeId], currentClusterCentroids);

				int nNNs = groupNNIdx.size();

				std::vector<float> scores;
				std::vector<int> filteredRemainIds;
				std::vector<float> min3DSpacings;

				for (int j = 0; j < nNNs; ++j)
				{
					int remainId = remainIds[groupNNIdx[j][i]];

					float lineSegs2 = zFeatures[toMergeId] + lenFeatures[toMergeId] - zFeatures[remainId];
					float lineSegs1 = zFeatures[remainId] + lenFeatures[remainId] - zFeatures[toMergeId];

					float verticalOverlapRatio = (lineSegs2 > lineSegs1 ? lineSegs1 : lineSegs2) / (lineSegs1 > lineSegs2 ? lineSegs1 : lineSegs2);
					float horizontalOverlapRatio;
					if ((boost::geometry::num_points(groupHulls[toMergeId]) > 3) & (boost::geometry::num_points(groupHulls[remainId]) > 3)) {
						multi_polygon intersection;
						boost::geometry::intersection(groupHulls[toMergeId], groupHulls[remainId], intersection);
						float intersect_area = boost::geometry::area(intersection);
						float area1 = boost::geometry::area(groupHulls[toMergeId]);
						float area2 = boost::geometry::area(groupHulls[remainId]);
						horizontalOverlapRatio = intersect_area / (area1 < area2 ? area1 : area2);
					}
					else {
						horizontalOverlapRatio = 0.0;
					}

					Eigen::MatrixXf nnClusterCentroids;
					get_subset(clusterCentroids, clusterVGroup[remainId], nnClusterCentroids);


					knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> knn_kdtree2(nnClusterCentroids);
					knn_cpp_build(knn_kdtree2);
					std::vector <std::vector<size_t>>min3D_idx;
					std::vector <std::vector<float>>min3D_dists;

					knn_cpp_query(knn_kdtree2, currentClusterCentroids, 1, min3D_idx, min3D_dists);
					float min3DSpacing = min_col(min3D_dists[0]);
					min3DSpacings.push_back(min3DSpacing);

					Eigen::MatrixXf nnClusterCentroids2D = nnClusterCentroids.block(0, 0, 2, nnClusterCentroids.cols());
					Eigen::MatrixXf currentClusterCentroids2D = currentClusterCentroids.block(0, 0, 2, currentClusterCentroids.cols());
					Eigen::VectorXf nnClusterCentroids2Dmean = nnClusterCentroids2D.rowwise().mean();
					Eigen::VectorXf currentClusterCentroids2Dmean = currentClusterCentroids2D.rowwise().mean();
					float min2DSpacing = sqrt((nnClusterCentroids2Dmean[0] - currentClusterCentroids2Dmean[0]) * (nnClusterCentroids2Dmean[0] - currentClusterCentroids2Dmean[0]) + (nnClusterCentroids2Dmean[1] - currentClusterCentroids2Dmean[1]) * (nnClusterCentroids2Dmean[1] - currentClusterCentroids2Dmean[1]));

					verticalOverlapRatio = verticalOverlapRatio > 0 ? verticalOverlapRatio : 0;

					float score = exp(-(1 - horizontalOverlapRatio) * (1 - horizontalOverlapRatio) - PR_VERTICAL_WEIGHT * (1 - verticalOverlapRatio) * (1 - verticalOverlapRatio) - ((min3DSpacing < min2DSpacing ? min3DSpacing : min2DSpacing) / sigmaD) * ((min3DSpacing < min2DSpacing ? min3DSpacing : min2DSpacing) / sigmaD));
					scores.push_back(score);

					filteredRemainIds.push_back(remainId);
				}


				std::vector<size_t> scoreSortI;
				std::vector<float> scoreSort;

				sort_indexes(scores, scoreSortI, scoreSort);
				float score_highest = scoreSort[0];
				if (score_highest == 0) {
					continue;
				}
				std::vector<float> scoreSortRatio;

				scoreSortRatio.reserve(scoreSort.size());
				std::transform(scoreSort.begin(), scoreSort.end(), std::back_inserter(scoreSortRatio),
					[score_highest](float value) { return value / score_highest; });

				std::vector<std::size_t> scoreSortCandidateIdx;
				for (std::size_t i = 0; i < scoreSortRatio.size(); ++i)
				{
					if (scoreSortRatio[i] > 0.7)
					{
						scoreSortCandidateIdx.push_back(i);
					}
				}
				int nScoreSortCandidateIdx = scoreSortCandidateIdx.size();
				int mergeNNId;
				if (nScoreSortCandidateIdx == 1)
				{
					mergeNNId = groupU[filteredRemainIds[scoreSortI[scoreSortCandidateIdx[0]]]];

				}
				else if (nScoreSortCandidateIdx > 1) {
					std::vector<float> min3DSpacingsFiltered;
					std::vector<size_t> scoreSortIFiltered;
					get_subset(scoreSortI, scoreSortCandidateIdx, scoreSortIFiltered);
					get_subset(min3DSpacings, scoreSortIFiltered, min3DSpacingsFiltered);

					size_t filterMinSpacingIdx = arg_min_col(min3DSpacingsFiltered);
					mergeNNId = groupU[filteredRemainIds[scoreSortI[filterMinSpacingIdx]]];
				}
				else {
					continue;
				}


				std::vector<uint32_t> currentVGroup = groupVGroup[toMergeIds[i]];
				for (unsigned k = 0; k < currentVGroup.size(); ++k)
				{
					segs_group_ids[currentVGroup[k]] = mergeNNId;
				}
				mergedRemainIds.push_back(mergeNNId);
				mergedRemainIds.push_back(groupU[toMergeIds[i]]);
			}

			n_to_merge_ids = toMergeIds.size();
			get_subset(segs_group_ids, initUI, cluster_ids);
			unique_group(cluster_ids, clusterVGroup, clusterU0);
			n_clusters = clusterVGroup.size();
			iter++;


		}
		unique_group(segs_group_ids, groupVGroup, groupU);
		for (unsigned j = 0; j < groupVGroup.size(); ++j)
		{
			std::vector<uint32_t> currentVGroup = groupVGroup[j];
			for (unsigned k = 0; k < currentVGroup.size(); ++k)
			{
				segs_group_ids[currentVGroup[k]] = j + 1;
			}
		}

		//export segments as a new scalar field
		char* sfName = "final_segs";
		int outSFIndex = m_pointCloud->getScalarFieldIndexByName(sfName);
		if (outSFIndex < 0)
		{
			outSFIndex = m_pointCloud->addScalarField(sfName);
			if (outSFIndex < 0)
			{
				ccLog::Error("[TreeIso] Not enough memory!");
				return false;
			}
		}
		CCCoreLib::ScalarField* outSF = m_pointCloud->getScalarField(outSFIndex);
		outSF->fill(CCCoreLib::NAN_VALUE);

		std::vector<uint32_t> groupIdx(pointCount);
		for (unsigned i = 0; i < pointCount; ++i)
		{
			outSF->setValue(i, segs_group_ids[i]);
		}
		outSF->computeMinAndMax();
		m_pointCloud->colorsHaveChanged();
		m_pointCloud->setCurrentDisplayedScalarField(outSFIndex);
	}



}

//1. initial 3D segmentation
bool TreeIso::init_seg(const unsigned min_nn1, const float regStrength1, const float PR_DECIMATE_RES1,ccMainAppInterface* app/*=nullptr*/,
	QWidget* parent/*=nullptr*/)
{

	m_progress->setRange(0, 100);

	const ccHObject::Container& selectedEntities = app->getSelectedEntities();

	if (selectedEntities.empty())
	{
		assert(false);
		app->dispToConsole("[TreeIso] Select at least one cloud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	ccHObject* ent = selectedEntities[0];

	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		app->dispToConsole("[TreeIso] Not a point cloud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}
	ccPointCloud* m_pointCloud = static_cast<ccPointCloud*>(ent);
	const unsigned pointCount = m_pointCloud->size();


	std::vector<std::vector<float>> pc_vec;
	toVec2DTranslate(m_pointCloud, pc_vec);
	m_progress->setValue(10);
	QApplication::processEvents();

	std::vector<size_t> ia,ic;
	std::vector<std::vector<float>> pc_dec, pc_sub;
	decimate_vec(pc_vec, PR_DECIMATE_RES1, pc_dec);
	unique_index_by_rows(pc_dec, ia, ic);

	get_subset(pc_vec, ia, pc_sub);
	m_progress->setValue(30);
	QApplication::processEvents();

	const unsigned K = (min_nn1 - 1);

	std::vector<std::vector<unsigned int>> nn_idx;
	std::vector<std::vector<float>> nn_D;

	auto start = std::chrono::steady_clock::now();     // start timer

	std::vector<uint32_t> in_component;
	std::vector<std::vector<uint32_t>> components;

	std::vector<float> edgeWeight;
	std::vector<uint32_t> Eu;
	std::vector<uint32_t> Ev;

	perform_cut_pursuit(K, regStrength1, pc_sub, edgeWeight, Eu, Ev, in_component, components);
	m_progress->setValue(90);
	QApplication::processEvents();


	//export segments as a new scalar field
	char* sfName = "init_segs";
	int outSFIndex = m_pointCloud->getScalarFieldIndexByName(sfName);
	if (outSFIndex < 0)
	{
		outSFIndex = m_pointCloud->addScalarField(sfName);
		if (outSFIndex < 0)
		{
			ccLog::Error("[TreeIso] Not enough memory!");
			return false;
		}
	}
	CCCoreLib::ScalarField* outSF = m_pointCloud->getScalarField(outSFIndex);
	outSF->fill(CCCoreLib::NAN_VALUE);

	std::vector<uint32_t> clusterIdx(pointCount);
	for (unsigned i = 0; i < pointCount; ++i)
	{
		clusterIdx[i] = in_component[ic[i]];
		outSF->setValue(i, in_component[ic[i]]);
	}
	outSF->computeMinAndMax();
	m_pointCloud->colorsHaveChanged();
	m_pointCloud->setCurrentDisplayedScalarField(outSFIndex);

	m_progress->setValue(100);
	QApplication::processEvents();

	auto elapsed = since(start).count() / 1000;
	app->dispToConsole(QString("[TreeIso] Init segs took: %1 seconds !!!").arg(elapsed));

	ent->redrawDisplay();

	return true;
}

//2. Bottom-up 2D segmentation
bool TreeIso::intermediate_seg(const unsigned PR_MIN_NN2, const float PR_REG_STRENGTH2, const float PR_DECIMATE_RES2, const float PR_MAX_GAP, ccMainAppInterface* app/*=nullptr*/,
						QWidget* parent/*=nullptr*/)
{
	m_progress->setRange(0, 100);

	const ccHObject::Container& selectedEntities = app->getSelectedEntities();

	if (selectedEntities.empty())
	{
		assert(false);
		app->dispToConsole("[TreeIso] Select at least one cloud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	ccHObject* ent = selectedEntities[0];

	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		app->dispToConsole("[TreeIso] Not a point cloud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}

	ccPointCloud* m_pointCloud = static_cast<ccPointCloud*>(ent);
	unsigned pointCount = m_pointCloud->size();


	int initSFIndex = m_pointCloud->getScalarFieldIndexByName("init_segs");
	if (initSFIndex < 0)
	{
		ccLog::Error("[TreeIso] Please run initial segmentation first!");
		return false;
	}
	CCCoreLib::ScalarField* initSF = m_pointCloud->getScalarField(initSFIndex);

	initSF->size();
	std::vector<uint32_t> in_component;
	in_component.resize(pointCount);
	for (int i = 0; i < pointCount;++i) {
		in_component [i]=initSF->getValue(i);
	}
	std::vector<std::vector<uint32_t>> clusterVGroup;
	unique_group(in_component, clusterVGroup);


	std::vector<std::vector<float>> pc_vec;
	toVec2DTranslate(m_pointCloud, pc_vec);



	auto start = std::chrono::steady_clock::now();
	int n_clusters = clusterVGroup.size();

	std::vector<std::vector<float>> clusterCentroids(n_clusters);
	std::vector<std::vector<std::vector<float>>> currentClusterDecs(n_clusters);
	std::vector<std::vector<size_t>> currentClusterDecsICs(n_clusters);

	for (unsigned i = 0; i < n_clusters; ++i) {
		std::vector<std::vector<float>> currentClusterPos;
		get_subset(pc_vec, clusterVGroup[i], currentClusterPos);

		if (currentClusterPos.size() > 1)
		{
			std::vector<size_t> ia, ic;
			std::vector<std::vector<float>> currentClusterPosDec;
			decimate_vec(currentClusterPos, PR_DECIMATE_RES2, currentClusterPosDec);
			unique_index_by_rows(currentClusterPosDec, ia, ic);

			std::vector<std::vector<float>> currentClusterPosUnq;
			get_subset(currentClusterPos, ia, currentClusterPosUnq);
			currentClusterDecs[i] = currentClusterPosUnq;

			mean_col(currentClusterPos, clusterCentroids[i]);
			currentClusterDecsICs[i] = ic;
		}
		else {
			currentClusterDecs[i] = currentClusterPos;
			clusterCentroids[i] = currentClusterPos[0];
			currentClusterDecsICs[i] = std::vector<size_t>(1, 0);
		}
	}

	m_progress->setValue(10);
	QApplication::processEvents();

	std::vector<std::vector<size_t>> minIdxsC;
	std::vector<std::vector<float>> minIdxsD;
	knn_cpp_nearest_neighbors(clusterCentroids, PR_MIN_NN2, minIdxsC, minIdxsD, 8);

	m_progress->setValue(15);
	QApplication::processEvents();

	int n_centroids = minIdxsC.size();
	int n_K = minIdxsC[0].size();
	std::vector<Eigen::MatrixXf> currentClusterDecMats;
	std::vector<knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>> knn_kdtrees;
	for (unsigned i = 0; i < n_centroids; ++i)
	{
		std::vector<std::vector<float>> currentClusterDec = currentClusterDecs[i];
		Eigen::MatrixXf currentClusterDecMat(currentClusterDec[0].size(), currentClusterDec.size());
		for (int k = 0; k < currentClusterDec.size(); k++) {
			currentClusterDecMat.col(k) = Eigen::VectorXf::Map(&currentClusterDec[k][0], currentClusterDec[k].size());
		}
		currentClusterDecMats.push_back(currentClusterDecMat);
	}

	m_progress->setValue(20);
	QApplication::processEvents();

	std::vector<std::vector<float>> nnDists;
	nnDists.resize(n_centroids, std::vector<float>(n_K));
	for (unsigned i = 0; i < n_centroids; ++i)
	{
		knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> knn_kdtree(currentClusterDecMats[minIdxsC[i][0]]);
		knn_cpp_build(knn_kdtree);
		for (unsigned j = 1; j < n_K; ++j)
		{
			if (minIdxsD[i][j] > 0)
			{
				std::vector<std::vector<size_t>> min_c;
				std::vector<std::vector<float>> min_D2;
				float min_D = knn_cpp_query_min_d(knn_kdtree, currentClusterDecMats[minIdxsC[i][j]], 1);
				nnDists[i][j] = min_D;
			}
		}

	}

	m_progress->setValue(40);
	QApplication::processEvents();

	std::vector<std::vector<float>> currentClusterDecsFlat;
	std::vector<size_t> currentClusterDecsFlatIndex;
	std::vector<size_t> currentClusterDecsICsFlatIndex;
	std::vector<uint32_t> currentClusterDecsIDs;


	for (auto& vec : clusterVGroup) {
		currentClusterDecsIDs.insert(currentClusterDecsIDs.end(), std::make_move_iterator(vec.begin()), std::make_move_iterator(vec.end()));
	}
	for (auto& vec : currentClusterDecs) {
		currentClusterDecsFlat.insert(currentClusterDecsFlat.end(), std::make_move_iterator(vec.begin()), std::make_move_iterator(vec.end()));
		currentClusterDecsICsFlatIndex.insert(currentClusterDecsICsFlatIndex.end(), vec.size());
	}

	for (unsigned i = 0; i < currentClusterDecs.size(); i++) {
		for (unsigned j = 0; j < currentClusterDecs[i].size(); ++j) {
			currentClusterDecsFlatIndex.push_back(i);
		}
	}

	m_progress->setValue(50);
	QApplication::processEvents();

	int nNodes = currentClusterDecsFlat.size();
	int nKs = PR_MIN_NN2;

	std::vector<std::vector<size_t>> minIdxs;
	std::vector<std::vector<float>> Ds;

	knn_cpp_nearest_neighbors(currentClusterDecsFlat, PR_MIN_NN2, minIdxs, Ds, 8);

	std::vector<float> edgeWeight;
	std::vector<uint32_t> Eu;
	std::vector<uint32_t> Ev;

	for (int i = 0; i < minIdxs.size(); i++) {
		int currentNode = currentClusterDecsFlatIndex[i];
		std::vector<float> currentDists = nnDists[currentNode];
		for (unsigned j = 1; j < minIdxs[0].size(); ++j) {
			int nnNode = currentClusterDecsFlatIndex[minIdxs[i][j]];
			std::vector<size_t> nnCand = minIdxsC[currentNode];
			auto it = std::find(nnCand.begin(), nnCand.end(), nnNode);
			if (it != nnCand.end())
			{
				float nnDist = currentDists[it - nnCand.begin()];
				if (nnDist < PR_MAX_GAP) {
					Eu.push_back(i);
					Ev.push_back(minIdxs[i][j]);
					edgeWeight.push_back(10 / ((nnDist + 0.1) / 0.01));
				}
			}
		}
	}


	m_progress->setValue(55);
	QApplication::processEvents();

	std::vector<uint32_t> in_component2d;
	perform_cut_pursuit2d(PR_MIN_NN2, PR_REG_STRENGTH2, currentClusterDecsFlat, edgeWeight, Eu, Ev, in_component2d);

	m_progress->setValue(80);
	QApplication::processEvents();

	std::vector<uint32_t> currentClusterDecsICsReverse;
	std::vector<std::vector<float>> currentClusterDecsReverse;
	int ia_counter = 0;
	for (unsigned i = 0; i < currentClusterDecs.size(); i++) {
		int N = currentClusterDecsICs[i].size();
		std::vector<uint32_t> v(in_component2d.begin() + ia_counter, in_component2d.begin() + ia_counter + currentClusterDecsICsFlatIndex[i]);
		for (unsigned j = 0; j < N; j++) {
			currentClusterDecsICsReverse.push_back(v[currentClusterDecsICs[i][j]]);
		}
		ia_counter += currentClusterDecsICsFlatIndex[i];
	}

	std::vector<size_t> currentClusterDecsIDsSortedIdx;
	std::vector<uint32_t> currentClusterDecsIDsSorted;

	sort_indexes(currentClusterDecsIDs, currentClusterDecsIDsSortedIdx, currentClusterDecsIDsSorted);
	std::vector<uint32_t> currentClusterDecsICsReverseSorted;
	currentClusterDecsICsReverseSorted.reserve(currentClusterDecsICsReverse.size());
	std::transform(currentClusterDecsIDsSortedIdx.begin(), currentClusterDecsIDsSortedIdx.end(), std::back_inserter(currentClusterDecsICsReverseSorted),
		[&](const int& i) { return currentClusterDecsICsReverse[i]; });

	//export segments as a new scalar field
	char* sfName = "intermediate_segs";
	int outSFIndex = m_pointCloud->getScalarFieldIndexByName(sfName);
	if (outSFIndex < 0)
	{
		outSFIndex = m_pointCloud->addScalarField(sfName);
		if (outSFIndex < 0)
		{
			ccLog::Error("[TreeIso] Not enough memory!");
			return false;
		}
	}
	CCCoreLib::ScalarField* outSF = m_pointCloud->getScalarField(outSFIndex);
	outSF->fill(CCCoreLib::NAN_VALUE);

	std::vector<uint32_t> groupIdx(pointCount);
	for (unsigned i = 0; i < pointCount; ++i)
	{
		groupIdx[i] = currentClusterDecsICsReverseSorted[i];
		outSF->setValue(i, currentClusterDecsICsReverseSorted[i]);
	}
	outSF->computeMinAndMax();
	m_pointCloud->colorsHaveChanged();
	m_pointCloud->setCurrentDisplayedScalarField(outSFIndex);


	m_progress->setValue(100);
	QApplication::processEvents();

	auto elapsed = since(start).count() / 1000;
	app->dispToConsole(QString("[TreeIso] Intermediate segs took: %1 seconds !!!").arg(elapsed));

	ent->redrawDisplay();
return true;
}
//bool TreeIso::LoadPcd(ccMainAppInterface* app/*=nullptr*/)
//{
//	//ccHObject* cloudContainer = app->loadFile("F:\\prj\\CC\\data\\JP8_plot_2cm_filtered_georef_itc_edit_demo.las", true);
//	ccHObject* cloudContainer = app->loadFile("F:\\prj\\CC\\data\\JP1_plot1_sep_demo_sub3_res2.las", true);
//	app->addToDB(cloudContainer);
//	return true;
//
//}

//3. Global edge refinement
bool TreeIso::final_seg(const unsigned PR_MIN_NN3, const float PR_REL_HEIGHT_LENGTH_RATIO, const float PR_VERTICAL_WEIGHT, ccMainAppInterface* app/*=nullptr*/,
	QWidget* parent/*=nullptr*/)
{
	m_progress->setRange(0, 100);

	const ccHObject::Container& selectedEntities = app->getSelectedEntities();

	if (selectedEntities.empty())
	{
		assert(false);
		app->dispToConsole("[TreeIso] Select at least one cloud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}


	ccHObject* ent = selectedEntities[0];
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		app->dispToConsole("[TreeIso] Not a point cloud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return false;
	}
	ccPointCloud* m_pointCloud = static_cast<ccPointCloud*>(ent);
	unsigned pointCount = m_pointCloud->size();
	std::vector<std::vector<float>> pc_vec;
	toVec2DTranslate(m_pointCloud, pc_vec);


	int initIdx = m_pointCloud->getScalarFieldIndexByName("init_segs");
	if (initIdx < 0)
	{
		ccLog::Error("[TreeIso] Please run initial segmentation first!");
		return false;
	}
	int groupIdx = m_pointCloud->getScalarFieldIndexByName("intermediate_segs");
	if (groupIdx < 0)
	{
		ccLog::Error("[TreeIso] Please run intermeditate segmentation first!");
		return false;
	}

	CCCoreLib::ScalarField* initSF = m_pointCloud->getScalarField(initIdx);
	initSF->size();
	std::vector<uint32_t> segs_init_ids;
	segs_init_ids.resize(pointCount);
	for (int i = 0; i < pointCount; ++i) {
		segs_init_ids[i] = initSF->getValue(i);
	}

	std::vector<std::vector<uint32_t>> initVGroup;
	std::vector<uint32_t> initU;
	std::vector<uint32_t> initUI;
	unique_group(segs_init_ids, initVGroup, initU, initUI);


	int n_init_clusters = initVGroup.size();
	std::vector<std::vector<float>> clusterCentroids(n_init_clusters);


	for (int i = 0; i < n_init_clusters; ++i) {
		std::vector<std::vector<float>> clusterPts;
		get_subset(pc_vec, initVGroup[i], clusterPts);
		std::vector<float> clusterCentroid;
		mean_col(clusterPts, clusterCentroid);
		clusterCentroids[i] = clusterCentroid;
	}


	CCCoreLib::ScalarField* groupSF = m_pointCloud->getScalarField(groupIdx);

	groupSF->size();
	std::vector<uint32_t> segs_group_ids;
	segs_group_ids.resize(pointCount);

	for (int i = 0; i < pointCount; ++i) {
		segs_group_ids[i] = groupSF->getValue(i);
	}

	std::vector<uint32_t> cluster_ids;
	get_subset(segs_group_ids, initUI, cluster_ids);

	std::vector<std::vector<uint32_t>> clusterVGroup;
	std::vector<uint32_t> clusterU0;
	unique_group(cluster_ids, clusterVGroup, clusterU0);


	if (clusterVGroup.size() > 1)
	{
		auto start = std::chrono::steady_clock::now();
		int n_clusters = clusterVGroup.size();

		std::vector<int> toMergeIds;
		std::vector<int> prevToMergeIds;
		std::vector<int> mergedRemainIds;


		int n_to_merge_ids = 1;
		int n_prev_merge_ids = -1;

		int iter = 1;

		std::vector<std::vector<uint32_t>> groupVGroup;
		std::vector<uint32_t> groupU;

		std::vector<std::vector<float>> centroid2DFeatures;
		Eigen::MatrixXf groupCentroidsToMerge;
		Eigen::MatrixXf groupCentroidsRemain;

		while ((n_to_merge_ids>0) & (n_to_merge_ids != n_prev_merge_ids)) {
			if (iter > 1) {
				n_prev_merge_ids = n_to_merge_ids;
			}			
			unique_group(segs_group_ids, groupVGroup, groupU);
			int nGroups = groupVGroup.size();
			centroid2DFeatures.resize(nGroups, std::vector<float>(2));

			std::vector<float> zFeatures;
			zFeatures.resize(nGroups);
			std::vector<float> lenFeatures;
			lenFeatures.resize(nGroups);

			std::vector<polygon> groupHulls;
			for (int i = 0; i < nGroups; ++i) {
				std::vector<std::vector<float>> groupPts;
				get_subset(pc_vec, groupVGroup[i], groupPts);
				std::vector<float> groupCentroids;
				mean_col(groupPts, groupCentroids);
				centroid2DFeatures[i][0] = groupCentroids[0];
				centroid2DFeatures[i][1] = groupCentroids[1];

				std::vector<float> minPts;
				min_col(groupPts, minPts);
				zFeatures[i] = minPts[2];
				std::vector<float> maxPts;
				max_col(groupPts, maxPts);
				lenFeatures[i] = maxPts[2] - minPts[2];

				polygon hull;
				multi_point conv_points;
				for (const auto& p : groupPts)
				{
					conv_points.push_back(point_xy(p[0], p[1]));
				}

				boost::geometry::convex_hull(conv_points, hull);
				groupHulls.push_back(hull);
			}


			int knncpp_nn = (PR_MIN_NN3 < n_clusters ? PR_MIN_NN3 : n_clusters);
			std::vector<std::vector<size_t>> groupNNIdxC;
			std::vector<std::vector<float>> groupNNCDs;
			knn_cpp_nearest_neighbors(centroid2DFeatures, knncpp_nn, groupNNIdxC, groupNNCDs, 8);
			std::vector<float> mds;
			mean_col(groupNNCDs, mds);
			float sigmaD = mds[1];

			std::vector<int> toMergeIds;
			std::vector<int> toMergeCandidateIds;
			std::vector<float> toMergeCandidateMetrics;
			for (int i = 0; i < nGroups; ++i)
			{
				std::vector<size_t> nnGroupId = groupNNIdxC[i];
				std::vector<float> nnGroupZ; nnGroupZ.resize(nnGroupId.size());
				std::vector<float> nnGroupLen; nnGroupLen.resize(nnGroupId.size());
				for (int j = 0; j < nnGroupId.size(); ++j)
				{
					nnGroupZ[j] = zFeatures[nnGroupId[j]];
					nnGroupLen[j] = lenFeatures[nnGroupId[j]];
				}
				float minZ = min_col(nnGroupZ);
				float minLen = min_col(nnGroupLen);

				float currentGroupRelHt = (zFeatures[i] - minZ) / lenFeatures[i];

				if (abs(currentGroupRelHt) > PR_REL_HEIGHT_LENGTH_RATIO) {
					if (iter == 1) {
						float initialLenRatio=lenFeatures[i] / median_col(lenFeatures);
						if (initialLenRatio > 1.5) {
							toMergeIds.push_back(i);
						}
						toMergeCandidateIds.push_back(i);
						toMergeCandidateMetrics.push_back(initialLenRatio);
					}
					else {
						toMergeIds.push_back(i);
					}

				}
			}
			if ((iter == 1) & (toMergeCandidateMetrics.size() == 0))
			{
				break;
			}
			if ((iter == 1) & (toMergeIds.size() == 0)) {
				int cand_ind= arg_max_col(toMergeCandidateMetrics);
				toMergeIds.push_back(toMergeCandidateIds[cand_ind]);
			}

			std::vector<int> remainIds;
			std::vector<int> allIds(nGroups);
			std::iota(std::begin(allIds), std::end(allIds), 0);
			std::set_difference(allIds.begin(), allIds.end(), toMergeIds.begin(), toMergeIds.end(), std::inserter(remainIds, remainIds.begin()));


			get_subset(centroid2DFeatures, toMergeIds, groupCentroidsToMerge);
			get_subset(centroid2DFeatures, remainIds, groupCentroidsRemain);
			int knncpp_nn2 = (PR_MIN_NN3 < remainIds.size() ? PR_MIN_NN3 : remainIds.size());


			knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> knn_kdtree(groupCentroidsRemain);
			knn_cpp_build(knn_kdtree);
			std::vector <std::vector<size_t>>groupNNIdx;
			std::vector <std::vector<float>>groupNNIdxDists;

			knn_cpp_query(knn_kdtree, groupCentroidsToMerge, knncpp_nn2, groupNNIdx, groupNNIdxDists);

			int nToMergeIds = toMergeIds.size();
			int nRemainIds = remainIds.size();
			for (int i = 0; i < nToMergeIds; ++i)
			{
				int toMergeId = toMergeIds[i];

				Eigen::MatrixXf currentClusterCentroids;
				get_subset(clusterCentroids, clusterVGroup[toMergeId], currentClusterCentroids);

				int nNNs = groupNNIdx.size();

				std::vector<float> scores;
				std::vector<int> filteredRemainIds;
				std::vector<float> min3DSpacings;

				for (int j = 0; j < nNNs; ++j)
				{
					int remainId = remainIds[groupNNIdx[j][i]];

					float lineSegs2 = zFeatures[toMergeId] + lenFeatures[toMergeId] - zFeatures[remainId];
					float lineSegs1 = zFeatures[remainId] + lenFeatures[remainId] - zFeatures[toMergeId];

					float verticalOverlapRatio = (lineSegs2 > lineSegs1 ? lineSegs1 : lineSegs2) / (lineSegs1 > lineSegs2 ? lineSegs1 : lineSegs2);
					float horizontalOverlapRatio;
					if ((boost::geometry::num_points(groupHulls[toMergeId]) > 3) & (boost::geometry::num_points(groupHulls[remainId]) > 3)) {
						multi_polygon intersection;
						boost::geometry::intersection(groupHulls[toMergeId], groupHulls[remainId], intersection);
						float intersect_area = boost::geometry::area(intersection);
						float area1 = boost::geometry::area(groupHulls[toMergeId]);
						float area2 = boost::geometry::area(groupHulls[remainId]);
						horizontalOverlapRatio = intersect_area / (area1 < area2 ? area1 : area2);
					}
					else {
						horizontalOverlapRatio = 0.0;
					}

					Eigen::MatrixXf nnClusterCentroids;
					get_subset(clusterCentroids, clusterVGroup[remainId], nnClusterCentroids);


					knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> knn_kdtree2(nnClusterCentroids);
					knn_cpp_build(knn_kdtree2);
					std::vector <std::vector<size_t>>min3D_idx;
					std::vector <std::vector<float>>min3D_dists;

					knn_cpp_query(knn_kdtree2, currentClusterCentroids, 1, min3D_idx, min3D_dists);
					float min3DSpacing = min_col(min3D_dists[0]);
					min3DSpacings.push_back(min3DSpacing);

					Eigen::MatrixXf nnClusterCentroids2D = nnClusterCentroids.block(0, 0, 2, nnClusterCentroids.cols());
					Eigen::MatrixXf currentClusterCentroids2D = currentClusterCentroids.block(0, 0, 2, currentClusterCentroids.cols());
					Eigen::VectorXf nnClusterCentroids2Dmean = nnClusterCentroids2D.rowwise().mean();
					Eigen::VectorXf currentClusterCentroids2Dmean = currentClusterCentroids2D.rowwise().mean();
					float min2DSpacing = sqrt((nnClusterCentroids2Dmean[0] - currentClusterCentroids2Dmean[0]) * (nnClusterCentroids2Dmean[0] - currentClusterCentroids2Dmean[0]) + (nnClusterCentroids2Dmean[1] - currentClusterCentroids2Dmean[1]) * (nnClusterCentroids2Dmean[1] - currentClusterCentroids2Dmean[1]));

					verticalOverlapRatio = verticalOverlapRatio > 0 ? verticalOverlapRatio : 0;

					float score = exp(-(1 - horizontalOverlapRatio) * (1 - horizontalOverlapRatio) - PR_VERTICAL_WEIGHT * (1 - verticalOverlapRatio) * (1 - verticalOverlapRatio) - ((min3DSpacing < min2DSpacing ? min3DSpacing : min2DSpacing) / sigmaD) * ((min3DSpacing < min2DSpacing ? min3DSpacing : min2DSpacing) / sigmaD));
					scores.push_back(score);

					filteredRemainIds.push_back(remainId);
				}


				std::vector<size_t> scoreSortI;
				std::vector<float> scoreSort;

				sort_indexes(scores, scoreSortI, scoreSort);
				float score_highest = scoreSort[0];
				if (score_highest == 0) {
					continue;
				}
				std::vector<float> scoreSortRatio;

				scoreSortRatio.reserve(scoreSort.size());
				std::transform(scoreSort.begin(), scoreSort.end(), std::back_inserter(scoreSortRatio),
					[score_highest](float value) { return value / score_highest; });

				std::vector<std::size_t> scoreSortCandidateIdx;
				for (std::size_t i = 0; i < scoreSortRatio.size(); ++i)
				{
					if (scoreSortRatio[i] > 0.7)
					{
						scoreSortCandidateIdx.push_back(i);
					}
				}
				int nScoreSortCandidateIdx = scoreSortCandidateIdx.size();
				int mergeNNId;
				if (nScoreSortCandidateIdx == 1)
				{
					mergeNNId = groupU[filteredRemainIds[scoreSortI[scoreSortCandidateIdx[0]]]];

				}
				else if (nScoreSortCandidateIdx > 1) {
					std::vector<float> min3DSpacingsFiltered;
					std::vector<size_t> scoreSortIFiltered;
					get_subset(scoreSortI, scoreSortCandidateIdx, scoreSortIFiltered);
					get_subset(min3DSpacings, scoreSortIFiltered, min3DSpacingsFiltered);

					size_t filterMinSpacingIdx = arg_min_col(min3DSpacingsFiltered);
					mergeNNId = groupU[filteredRemainIds[scoreSortI[filterMinSpacingIdx]]];
				}
				else {
					continue;
				}


				std::vector<uint32_t> currentVGroup = groupVGroup[toMergeIds[i]];
				for (unsigned k = 0; k < currentVGroup.size(); ++k)
				{
					segs_group_ids[currentVGroup[k]] = mergeNNId;
				}
				mergedRemainIds.push_back(mergeNNId);
				mergedRemainIds.push_back(groupU[toMergeIds[i]]);
			}

			n_to_merge_ids= toMergeIds.size();
			get_subset(segs_group_ids, initUI, cluster_ids);
			unique_group(cluster_ids, clusterVGroup,clusterU0);
			n_clusters = clusterVGroup.size();
			iter++;


		}
		unique_group(segs_group_ids, groupVGroup, groupU);
		for (unsigned j = 0; j < groupVGroup.size(); ++j)
		{
			std::vector<uint32_t> currentVGroup = groupVGroup[j];
			for (unsigned k = 0; k < currentVGroup.size(); ++k)
			{
				segs_group_ids[currentVGroup[k]] = j + 1;
			}
		}

		//export segments as a new scalar field
		char* sfName = "final_segs";
		int outSFIndex = m_pointCloud->getScalarFieldIndexByName(sfName);
		if (outSFIndex < 0)
		{
			outSFIndex = m_pointCloud->addScalarField(sfName);
			if (outSFIndex < 0)
			{
				ccLog::Error("[TreeIso] Not enough memory!");
				return false;
			}
		}
		CCCoreLib::ScalarField* outSF = m_pointCloud->getScalarField(outSFIndex);
		outSF->fill(CCCoreLib::NAN_VALUE);

		std::vector<uint32_t> groupIdx(pointCount);
		for (unsigned i = 0; i < pointCount; ++i)
		{
			outSF->setValue(i, segs_group_ids[i]);
		}
		outSF->computeMinAndMax();
		m_pointCloud->colorsHaveChanged();
		m_pointCloud->setCurrentDisplayedScalarField(outSFIndex);

		m_progress->setValue(100);
		QApplication::processEvents();

		auto elapsed = since(start).count() / 1000;
		app->dispToConsole(QString("[TreeIso] Final segs took: %1 seconds !!!").arg(elapsed));
	}

	ent->redrawDisplay();

	//app->updateUI();
	//app->refreshAll();

}


template <typename T>
void testWriteLog(std::vector< std::vector<T>>arr, std::string fname) {
	std::ofstream outfile(fname);
	for (const auto& row : arr) {
		for (const auto& elem : row) {
			outfile << elem << " ";
		}
		outfile << std::endl;
	}
	outfile.close();
}
template <typename T>
void testWriteLog(std::vector<T>arr, std::string fname) {
	std::ofstream outfile(fname);
	for (const auto& elem : arr) {
		outfile << elem << " ";
		outfile << std::endl;
	}
	outfile.close();
}

void knn_cpp_build(knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>& kdtree) {
	kdtree.setBucketSize(16);
	kdtree.setSorted(true);
	kdtree.setThreads(0);
	kdtree.build();
	return;
}

void knn_cpp_build(knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>& kdtree, int n_thread) {
	kdtree.setBucketSize(16);
	kdtree.setSorted(true);
	kdtree.setThreads(n_thread);
	kdtree.build();
	return;
}

void knn_cpp_query(knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>& kdtree, Eigen::MatrixXf& query_points, size_t k, std::vector <std::vector<size_t>>& res_idx, std::vector <std::vector<float>>& res_dists) {

	res_idx.clear();
	res_dists.clear();

	knncpp::Matrixi indices;
	Eigen::MatrixXf distances;

	kdtree.query(query_points, k, indices, distances);
	res_idx.resize(indices.rows(), std::vector<size_t>(indices.cols()));
	for (int i = 0; i < indices.rows(); i++) {
		res_idx[i] = std::vector<size_t>(indices.row(i).data(), indices.row(i).data() + indices.cols());
	}
	res_dists.resize(distances.rows(), std::vector<float>(distances.cols()));
	for (int i = 0; i < distances.rows(); i++) {
		res_dists[i] = std::vector<float>(distances.row(i).data(), distances.row(i).data() + distances.cols());
	}
}


float knn_cpp_query_min_d(knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>>& kdtree, Eigen::MatrixXf& query_points, size_t k) {
	knncpp::Matrixi indices;
	Eigen::MatrixXf distances;

	kdtree.query(query_points, k, indices, distances);
	return distances.minCoeff();
}


void knn_cpp_nearest_neighbors(const std::vector<std::vector<float>>& dataset, size_t k, std::vector <std::vector<size_t>>& res_idx, std::vector <std::vector<float>>& res_dists, int n_thread) {

	Eigen::MatrixXf mat(dataset[0].size(), dataset.size());
	for (int i = 0; i < dataset.size(); i++)
		mat.col(i) = Eigen::VectorXf::Map(&dataset[i][0], dataset[i].size());

	knncpp::KDTreeMinkowskiX<float, knncpp::EuclideanDistance<float>> kdtree(mat);
	kdtree.setBucketSize(16);
	kdtree.setSorted(true);
	kdtree.setThreads(n_thread);
	kdtree.build();
	knncpp::Matrixi indices;
	Eigen::MatrixXf distances;

	kdtree.query(mat, k, indices, distances);

	res_idx.resize(indices.cols(), std::vector<size_t>(indices.rows()));
	for (int i = 0; i < indices.cols(); i++) {
		res_idx[i] = std::vector<size_t>(indices.col(i).data(), indices.col(i).data() + indices.rows());
	}
	res_dists.resize(distances.cols(), std::vector<float>(distances.rows()));
	for (int i = 0; i < distances.cols(); i++) {
		res_dists[i] = std::vector<float>(distances.col(i).data(), distances.col(i).data() + distances.rows());
	}
}


template <typename T>
void unique_group(std::vector<T>& arr, std::vector<std::vector<T>>& u_group, std::vector<T>& arr_unq, std::vector<T>& ui)
{
	arr_unq.clear();
	ui.clear();
	u_group.clear();

	std::vector<size_t> arr_sorted_idx;
	std::vector<T> arr_sorted;
	sort_indexes(arr, arr_sorted_idx, arr_sorted);

	const size_t n = arr.size();

	ui.push_back(arr_sorted_idx[0]);
	std::size_t counter = 0;

	std::vector<T>* ut = new std::vector<T>();
	ut->push_back(arr_sorted_idx[0]);

	//detect the location (before sorted) where a row in the sorted array is different from the previous row (as ia), and add one for the reverse index as ic
	for (std::size_t i = 1; i < n; ++i) {

		if (arr_sorted[i] != arr_sorted[i - 1]) {
			ui.push_back(arr_sorted_idx[i]);
			arr_unq.push_back(arr_sorted[i]);

			u_group.push_back(*ut);
			ut = new std::vector<T>();
			ut->push_back(arr_sorted_idx[i]);
			counter++;
		}
		else
		{
			ut->push_back(arr_sorted_idx[i]);
		}
	}
	u_group.push_back(*ut);
	return;
}


template <typename T>
void unique_group(std::vector<T>& arr, std::vector<std::vector<T>>& u_group, std::vector<T>& arr_unq)
{

	arr_unq.clear();
	u_group.clear();

	std::vector<size_t> arr_sorted_idx;
	std::vector<T> arr_sorted;
	sort_indexes(arr, arr_sorted_idx, arr_sorted);

	const size_t n = arr.size();
	arr_unq.push_back(arr_sorted[0]);
	std::size_t counter = 0;

	std::vector<T>* ut = new std::vector<T>();
	ut->push_back(arr_sorted_idx[0]);

	//detect the location (before sorted) where a row in the sorted array is different from the previous row (as ia), and add one for the reverse index as ic
	for (std::size_t i = 1; i < n; ++i) {

		if (arr_sorted[i] != arr_sorted[i - 1]) {
			arr_unq.push_back(arr_sorted[i]);

			u_group.push_back(*ut);
			ut = new std::vector<T>();
			ut->push_back(arr_sorted_idx[i]);
			counter++;
		}
		else
		{
			ut->push_back(arr_sorted_idx[i]);
		}
	}
	u_group.push_back(*ut);
	return;
}

template <typename T>
void unique_group(std::vector<T>& arr, std::vector<std::vector<T>>& u_group)
{
	u_group.clear();

	std::vector<size_t> arr_sorted_idx;
	std::vector<T> arr_sorted;
	sort_indexes(arr, arr_sorted_idx, arr_sorted);

	const size_t n = arr.size();
	std::size_t counter = 0;

	std::vector<T>* ut = new std::vector<T>();
	ut->push_back(arr_sorted_idx[0]);

	//detect the location (before sorted) where a row in the sorted array is different from the previous row (as ia), and add one for the reverse index as ic
	for (std::size_t i = 1; i < n; ++i) {

		if (arr_sorted[i] != arr_sorted[i - 1]) {
			u_group.push_back(*ut);
			ut = new std::vector<T>();
			ut->push_back(arr_sorted_idx[i]);
			counter++;
		}
		else
		{
			ut->push_back(arr_sorted_idx[i]);
		}
	}
	u_group.push_back(*ut);

	return;
}

template <typename T1, typename T2>
void get_subset(std::vector<T1>& arr, std::vector<T2>& indices, std::vector<T1>& arr_sub)
{
	arr_sub.clear();
	for (const auto& idx : indices)
	{
		arr_sub.push_back(arr[idx]);
	}
	return;
}


template <typename T1, typename T2>
void get_subset(std::vector<std::vector<T1>>& arr, std::vector<T2>& indices, Eigen::MatrixXf& arr_sub)
{
	arr_sub.setZero();
	arr_sub.resize(arr[0].size(), indices.size());

	for (unsigned int i = 0; i < indices.size(); ++i)
	{
		for (unsigned int j = 0; j < arr[0].size(); ++j)
		{
			arr_sub(j,i) = arr[indices[i]][j];
		}
	}
	return;
}
template <typename T1, typename T2>
void get_subset(std::vector<std::vector<T1>>& arr, std::vector<T2>& indices, std::vector<std::vector<T1>>& arr_sub)
{
	arr_sub.clear();
	arr_sub.resize(indices.size());
	for (unsigned int i = 0; i < indices.size(); ++i)
	{
		arr_sub[i] = arr[indices[i]];
	}

	return;
}
template <typename T1, typename T2>
bool get_subset(ccPointCloud* pcd, std::vector<T2>& indices, std::vector<std::vector<T1>>& arr_sub)
{
	arr_sub.clear();
	arr_sub.resize(indices.size(), std::vector<T1>(3));
	for (unsigned int i = 0; i < indices.size(); ++i)
	{
		const CCVector3* vec = pcd->getPoint(indices[i]);
		arr_sub[i][0] = vec->x;
		arr_sub[i][1] = vec->y;
		arr_sub[i][2] = vec->z;
	}
	return true;
}

ccPointCloud* partial_pcd(ccPointCloud* pc, std::vector<size_t> ia)
{
	ccPointCloud* pc_sub;
	CCCoreLib::ReferenceCloud* pc_sub_ref = new CCCoreLib::ReferenceCloud(pc);
	if (!pc_sub_ref->reserve(ia.size()))
	{
		//not enough memory
		delete pc_sub_ref;
		pc_sub_ref = nullptr;
	}
	for (unsigned i = 0; i < ia.size(); ++i)
	{
		pc_sub_ref->addPointIndex(ia[i]);
	}
	pc_sub = pc->partialClone(pc_sub_ref);
	return pc_sub;
}



bool perform_cut_pursuit(unsigned K, const float regStrength, std::vector<std::vector<float>>& pc, std::vector<float>& edgeWeight, std::vector<uint32_t>& Eu, std::vector<uint32_t>& Ev, std::vector<uint32_t>& in_component, std::vector<std::vector<uint32_t>>& components)
{

	std::vector<std::vector<size_t>> nn_idx;
	std::vector<std::vector<float>> nn_D;
	knn_cpp_nearest_neighbors(pc, K + 1, nn_idx, nn_D, 8);

	const unsigned pointCount = pc.size();


	const uint32_t nNod = pointCount;
	const uint32_t nObs = 3;
	const uint32_t nEdg = pointCount * K;
	const uint32_t cutoff = 0;
	const float mode = 1;
	const float speed = 0;
	const float weight_decay = 0;
	const float verbose = 0;


	std::vector< std::vector<float>> y = pc;
	std::vector<float> nodeWeight;

	std::vector< std::vector<float> > solution;

	y.resize(pointCount, std::vector<float>(nObs));

	if (edgeWeight.size() == 0) {
		edgeWeight.resize(nEdg);
		std::fill(edgeWeight.begin(), edgeWeight.end(), 1.0);
	}

	nodeWeight.resize(pointCount);
	solution.resize(pointCount, std::vector<float>(K));


	//minus average
	std::vector<float> y_avg;
	y_avg.resize(3);
	if (Eu.size() == 0)
	{
		Eu.resize(nEdg);
		Ev.resize(nEdg);
		for (unsigned i = 0; i < pointCount; ++i)
		{
			y_avg[0] += y[i][0];
			y_avg[1] += y[i][1];
			y_avg[2] += y[i][2];

			for (unsigned j = 0; j < K; ++j)
			{
				Eu[i * K + j] = i;
				Ev[i * K + j] = nn_idx[i][j + 1];
			}

		}
	}
	else {
		for (unsigned i = 0; i < pointCount; ++i)
		{
			y_avg[0] += y[i][0];
			y_avg[1] += y[i][1];
			y_avg[2] += y[i][2];
		}
	}

	y_avg[0] = y_avg[0] / pointCount;
	y_avg[1] = y_avg[1] / pointCount;
	y_avg[2] = y_avg[2] / pointCount;

	for (unsigned i = 0; i < pointCount; ++i)
	{
		y[i][0] = y[i][0] - y_avg[0];
		y[i][1] = y[i][1] - y_avg[1];
		y[i][2] = y[i][2] - y_avg[2];
	}

	std::fill(nodeWeight.begin(), nodeWeight.end(), 1.0);
	CP::cut_pursuit<float>(nNod, nEdg, nObs, y, Eu, Ev, edgeWeight, nodeWeight, solution, in_component, components, regStrength, cutoff, mode, speed, weight_decay, verbose);
	return true;

}


void perform_cut_pursuit2d(const unsigned K, const float regStrength, std::vector<std::vector<float>>& pc_vec, std::vector<float>& edgeWeight, std::vector<uint32_t>& Eu, std::vector<uint32_t>& Ev, std::vector<uint32_t>& in_component)
{

	//build graph for cut-pursuit
	const unsigned pointCount = pc_vec.size();

	const uint32_t nNod = pointCount;
	const uint32_t nObs = 2;
	const uint32_t nEdgMax = pointCount * K;
	const uint32_t cutoff = 0;
	const float mode = 1;
	const float speed = 0;
	const float weight_decay = 0;
	const float verbose = 0;


	std::vector< std::vector<float>> y = pc_vec;
	std::vector<float> nodeWeight;

	std::vector< std::vector<float> > solution;

	y.resize(pointCount, std::vector<float>(nObs));

	if (edgeWeight.size() == 0) {
		edgeWeight.resize(nEdgMax);
		std::fill(edgeWeight.begin(), edgeWeight.end(), 1.0);
	}
	const uint32_t nEdg = edgeWeight.size();

	nodeWeight.resize(pointCount);
	solution.resize(pointCount, std::vector<float>(K));


	//minus average
	std::vector<float> y_avg;
	y_avg.resize(nObs);
	if (Eu.size() == 0)
	{
		std::vector<std::vector<size_t>> nn_idx;
		std::vector<std::vector<float>> nn_D;
		knn_cpp_nearest_neighbors(pc_vec, K + 1, nn_idx, nn_D, 8);
		Eu.resize(nEdg);
		Ev.resize(nEdg);
		for (unsigned i = 0; i < pointCount; ++i)
		{
			y_avg[0] += pc_vec[i][0];
			y_avg[1] += pc_vec[i][1];
			for (unsigned j = 0; j < K; ++j)
			{
				Eu[i * K + j] = i;
				Ev[i * K + j] = nn_idx[i][j + 1];
			}
		}
	}
	else {
		for (unsigned i = 0; i < pointCount; ++i)
		{
			y_avg[0] += pc_vec[i][0];
			y_avg[1] += pc_vec[i][1];
		}
	}


	y_avg[0] = y_avg[0] / pointCount;
	y_avg[1] = y_avg[1] / pointCount;

	for (unsigned i = 0; i < pointCount; ++i)
	{
		y[i][0] = pc_vec[i][0] - y_avg[0];
		y[i][1] = pc_vec[i][1] - y_avg[1];
	}

	std::fill(nodeWeight.begin(), nodeWeight.end(), 1.0);
	std::vector< std::vector<uint32_t> > components;
	CP::cut_pursuit<float>(nNod, nEdg, nObs, y, Eu, Ev, edgeWeight, nodeWeight, solution, in_component, components, regStrength, cutoff, mode, speed, weight_decay, verbose);
	return;
}

//convert ccPointCloud to 2D vector array
void toVec2DTranslate(ccPointCloud* pc, std::vector<std::vector<float>>& y) {

	
	const unsigned pointCount = pc->size();
	y.resize(pointCount, std::vector<float>(3));

	for (unsigned i = 0; i < pointCount; ++i)
	{
		const CCVector3* pv = pc->getPoint(i);
		y[i][0] = (float)(pv->x);
		y[i][1] = (float)(pv->y);
		y[i][2] = (float)(pv->z);
	}

	std::vector<float> y_mean;
	mean_col(y, y_mean);
	for (unsigned i = 0; i < pointCount; ++i)
	{
		y[i][0] = y[i][0]-y_mean[0];
		y[i][1] = y[i][1]-y_mean[1];
		y[i][2] = y[i][2]-y_mean[2];
	}
	return;
}
//convert ccPointCloud to 2D vector array
void toVec2D(ccPointCloud* pc, std::vector<std::vector<float>>& y) {

	const unsigned pointCount = pc->size();
	y.resize(pointCount, std::vector<float>(3));

	for (unsigned i = 0; i < pointCount; ++i)
	{
		const CCVector3* pv = pc->getPoint(i);
		y[i][0] = (float)(pv->x);
		y[i][1] = (float)(pv->y);
		y[i][2] = (float)(pv->z);
	}

	return;
}

template <typename T>
size_t arg_min_col(std::vector<T>& arr) {
	auto min_element_it = std::min_element(arr.begin(), arr.end());
	std::size_t min_index = std::distance(arr.begin(), min_element_it);
	return min_index;
}
template <typename T>
size_t arg_max_col(std::vector<T>& arr) {
	auto max_element_it = std::max_element(arr.begin(), arr.end());
	std::size_t max_index = std::distance(arr.begin(), max_element_it);
	return max_index;
}
template <typename T>
void min_col(std::vector<std::vector<T>>& arr, std::vector<T>& min_vals) {

	min_vals.clear();

	min_vals.resize(arr[0].size(), std::numeric_limits<int>::max());
	for (const auto& row : arr) {
		for (size_t i = 0; i < row.size(); i++) {
			min_vals[i] = std::min(min_vals[i], row[i]);
		}
	}
	return;
}

template <typename T>
T min_col(std::vector<T>& arr) {
	auto result_it = std::min_element(arr.begin(), arr.end());
	T result=*result_it;
	return result;
}

template <typename T>
T mean_col(std::vector<T>& arr) {
	float sum = std::accumulate(arr.begin(), arr.end(),0);
	float average = sum / arr.size();
	return average;
}

template <typename T>
T median_col(std::vector<T>& arr) {
	size_t n = arr.size();
	// Sort the vector
	std::sort(arr.begin(), arr.end());
	// Calculate the median
	if (n % 2 == 0) {
		return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
	}
	else {
		return arr[n / 2];
	}	
}




template <typename T>
void max_col(std::vector<std::vector<T>>& arr, std::vector<T>& max_vals) {
	max_vals.clear();
	max_vals.resize(arr[0].size(), std::numeric_limits<int>::min());
	for (const auto& row : arr) {
		for (size_t i = 0; i < row.size(); i++) {
			max_vals[i] = std::max(max_vals[i], row[i]);
		}
	}
	return;
}
template <typename T>
void mean_col(std::vector<std::vector<T>>& arr, std::vector<T>& mean_vals) {
	mean_vals.clear();

	//std::vector<T> mean_vals(arr[0].size());
	mean_vals.resize(arr[0].size());
	for (unsigned i = 0; i < arr[0].size(); i++) {
		std::vector<T> column(arr.size());
		std::transform(arr.begin(), arr.end(), column.begin(), [i](auto& v) { return v[i]; });
		mean_vals[i] = accumulate(column.begin(), column.end(), 0.0) / arr.size();
	}
	return;
}

template <typename T>
void decimate_vec(std::vector<std::vector<T>>& arr, float res, std::vector<std::vector<T>>& vec_dec) {

	vec_dec.clear();

	size_t num_rows = arr.size();
	size_t num_cols = arr[0].size();

	std::vector<T> arr_min;
	min_col(arr, arr_min);
	//std::vector<std::vector<T>> vec_dec;
	vec_dec.resize(num_rows, std::vector<float>(num_cols));

	for (unsigned i = 0; i < num_rows; ++i) {
		for (unsigned j = 0; j < num_cols; ++j) {
			vec_dec[i][j] = floor((arr[i][j] - arr_min[j]) / res) + 1;
		}
	}
	return;
}


//return index
template <typename T>
void sort_indexes_by_row(std::vector<std::vector<T>>& v, std::vector<size_t>& idx, std::vector<std::vector<T>>& v_sorted) {

	idx.clear();
	v_sorted.clear();

	// initialize original index locations
	size_t m = v.size();
	size_t n = v[0].size();

	idx.resize(m);
	std::iota(idx.begin(), idx.end(), 0);

	//std::vector<std::vector<T>> v_sorted;
	v_sorted.resize(m, std::vector<T>(n));

	// sort indexes based on comparing values in v
	// using std::stable_sort instead of std::sort
	// to avoid unnecessary index re-orderings
	// when v contains elements of equal values 
	std::stable_sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {
			//bool vt = true;
			for (int k = 0; k < v[0].size(); ++k)
			{
				if (v[i1][k] == v[i2][k]) {
					continue;
				}
				else {
					return v[i1][k] < v[i2][k];
				}
			}
			return false;
		});

	for (std::size_t i = 0; i < idx.size(); ++i) {
		v_sorted[i].resize(n);
		for (std::size_t k = 0; k < n; ++k)
		{
			v_sorted[i][k] = v[idx[i]][k];
		}
	}

	return;
}

//return index
template <typename T>
void sort_indexes(std::vector<T>& v, std::vector<size_t>& idx, std::vector<T>& v_sorted) {
	idx.clear();
	v_sorted.clear();

	size_t m = v.size();
	idx.resize(m);
	std::iota(idx.begin(), idx.end(), 0);

	v_sorted.resize(m);

	std::stable_sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {
			return v[i1] < v[i2];
		});

	for (std::size_t i = 0; i < idx.size(); ++i) {
		v_sorted[i] = v[idx[i]];
	}

	return;
}

template <typename T>
void unique_index_by_rows(std::vector<std::vector<T>>& arr, std::vector<size_t>& ia, std::vector<size_t>& ic)
{

	ia.clear();
	ic.clear();

	std::vector<std::vector<T>> arr_sorted;
	arr_sorted.resize(arr.size());
	std::vector<size_t> sort_idx;

	//sort array first and get indices
	sort_indexes_by_row(arr, sort_idx, arr_sorted);

	const size_t num_rows = arr_sorted.size();
	const size_t num_cols = arr_sorted[0].size();

	ic.resize(num_rows);
	ia.push_back(sort_idx[0]);
	std::size_t counter = 0;
	ic[sort_idx[0]] = counter;

	//detect the location (before sorted) where a row in the sorted array is different from the previous row (as ia), and add one for the reverse index as ic
	for (std::size_t i = 1; i < num_rows; ++i)
	{
		bool diff = false;
		for (std::size_t k = 0; k < num_cols; ++k) {
			if (arr_sorted[i][k] != arr_sorted[i - 1][k]) {
				diff = true;
				break;
			}
		}
		if (diff) {
			ia.push_back(sort_idx[i]);
			counter++;
		}
		ic[sort_idx[i]] = counter;
	}
	return;
}
