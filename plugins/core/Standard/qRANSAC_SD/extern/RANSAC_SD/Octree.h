#ifndef OCTREE_HEADER
#define OCTREE_HEADER
#include "PointCloud.h"
#include <GfxTL/AACubeTree.h>
#include <GfxTL/LevelBuildInformationTreeStrategy.h>
#include <GfxTL/CellSizeDataTreeStrategy.h>
#include <GfxTL/CellRangeDataTreeStrategy.h>
#include <GfxTL/ImmediateTreeDataKernels.h>
#include <GfxTL/IndexedTreeDataKernels.h>
#include <GfxTL/BucketSizeMaxLevelSubdivisionTreeStrategy.h>
#include <GfxTL/CellLevelTreeStrategy.h>
#include <GfxTL/CellCenterAACubeTreeStrategy.h>
#include <GfxTL/NullTreeStrategy.h>
#include <GfxTL/ShuffleIndicesTreeDataKernelStrategy.h>
#include "ScoreAACubeTreeStrategy.h"
#include "RebuildAACubeTreeStrategy.h"
#include <MiscLib/Vector.h>

typedef GfxTL::AACubeTree
	<
		3, ScoreAACubeTreeStrategy
		<
			3, RebuildAACubeTreeStrategy
			<
				GfxTL::BucketSizeMaxLevelSubdivisionTreeStrategy
				<
					GfxTL::CellLevelTreeStrategy
					<
						GfxTL::CellCenterAACubeTreeStrategy
						<
							3,
							GfxTL::BaseAACubeTreeStrategy
							<
								GfxTL::CellRangeDataTreeStrategy
								<
									GfxTL::NullTreeStrategy,
									GfxTL::ShuffleIndicesTreeDataKernelStrategy
									<
										GfxTL::ImmediateRangeTreeDataKernel< PointCloud >,
										MiscLib::Vector< size_t >
									>
								>
							>
						>
					>
				>
			>
		>
	> ImmediateOctreeType;

typedef GfxTL::AACubeTree
	<
		3, ScoreAACubeTreeStrategy
		<
			3, RebuildAACubeTreeStrategy
			<
				GfxTL::BucketSizeMaxLevelSubdivisionTreeStrategy
				<
					GfxTL::CellLevelTreeStrategy
					<
						GfxTL::CellCenterAACubeTreeStrategy
						<
							3,
							GfxTL::BaseAACubeTreeStrategy
							<
								GfxTL::CellRangeDataTreeStrategy
								<
									GfxTL::NullTreeStrategy,
									GfxTL::IteratedIndexedIteratorTreeDataKernel
									<
										MiscLib::Vector< size_t >::iterator,
										PointCloud::const_iterator
									>
								>
							>
						>
					>
				>
			>
		>
	> IndexedOctreeType;

#endif
