#ifndef __AAKDTREE_HEADER__
#define __AAKDTREE_HEADER__

#include <GfxTL/BaseTree.h>
#include <GfxTL/NullClass.h>
#include <GfxTL/AACube.h>
#include <GfxTL/NearestNeighbors.h>
#include <GfxTL/Intersect.h>
#include <list>
#include <vector>
#include <limits>

namespace GfxTL
{
	template< class Point, class Base >
	class AAKdCell
	: public Base
	{
		public:
			typedef Point PointType;
			typedef Base BaseType;
			typedef AAKdCell< Point, Base > ThisType;
			typedef typename Point::ScalarType ScalarType;
			enum { NChildren = 2 };

			AAKdCell();
			AAKdCell(const AAKdCell< Point, Base > &cell);
			~AAKdCell();
			const ThisType *operator[](unsigned int index) const;
			ThisType *operator[](unsigned int index);
			void Child(unsigned int index, ThisType *child);
			ScalarType Split() const;
			void Split(ScalarType split);
			unsigned int Axis() const;
			void Axis(unsigned int axis);

		private:
			unsigned int _axis;
			ScalarType _splitValue;
			ThisType *_children[2];
	};

	template< class Strategies >
	class AAKdTree
	: public Strategies::StrategyBase
			<
				BaseTree
				<
					AAKdCell
					<
						typename Strategies::PointType,
						typename Strategies::CellData
						<
							typename Strategies::PointType
						>
					>
				>
			>
	{
		public:
			typedef AAKdCell
					<
						typename Strategies::PointType,
						typename Strategies::CellData
						<
							typename Strategies::PointType
						>
					> CellType;
			typedef typename CellType::PointType PointType;
			typedef typename CellType::ScalarType ScalarType;
			typedef typename Strategies::StrategyBase
								<
									BaseTree
									<
										AAKdCell
										<
											typename Strategies::PointType,
											typename Strategies::CellData
											<
												typename Strategies::PointType
											>
										>
									>
								> StrategyBaseType;
			typedef typename StrategyBaseType::HandleType HandleType;
			typedef ::GfxTL::NearestNeighbor< ScalarType > NN;
			enum { Dim = PointType::Dim };

			void Build();
			void PointsInSphere(const PointType &center, ScalarType radius,
				std::vector< size_t > *points) const;
			void PointsInAACube(const AACube< PointType	> &cube,
				std::vector< size_t > *points) const;
			void PointsInAACube(const PointType &center, ScalarType width,
				std::vector< size_t > *points) const;
			void RefreshWithNewTreeData(const AACube< PointType > &bc);
			void NearestNeighbor(const PointType &p, size_t *neighbor,
				ScalarType *dist) const;
			void KNearestNeighbors(const PointType &p, unsigned int k,
				std::vector< NN > *neighbors,
				ScalarType *dist) const;

		private:
			void NearestNeighbor(const CellType &cell,
				const AABox< PointType > &box, const PointType &p,
				size_t *neighbor, ScalarType *dist2) const;
			void KNearestNeighbors(const CellType &cell,
				const AABox< PointType > &box, const PointType &p,
				unsigned int k, std::vector< NN > *neighbors,
				size_t *worstIdx, ScalarType *dist2) const;
			void ReadjustData(CellType *cell);
			void PointsInSphere(const CellType *cell, const PointType &center,
				ScalarType radius, std::vector< size_t > *points) const;
			void PointsInAACube(const CellType *cell, const PointType &center,
				ScalarType radius, std::vector< HandleType > *points) const;
			bool ShouldSubdivide(const CellType *cell) const;
			void Subdivide(CellType *cell);
			ScalarType Absolute(ScalarType f) const;
	};
};

#include "AAKdTree.hpp"

#endif