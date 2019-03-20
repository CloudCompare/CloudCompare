#ifndef GfxTL__KDTREE_HEADER__
#define GfxTL__KDTREE_HEADER__
#include <GfxTL/BaseTree.h>
#include <GfxTL/ScalarTypeDeferer.h>
#include <GfxTL/NullClass.h>
#include <GfxTL/VectorXD.h>
#include <GfxTL/AABox.h>
#include <GfxTL/ScalarTypeConversion.h>
#include <GfxTL/MathHelper.h>
#include <GfxTL/VectorKernel.h>
#include <GfxTL/LimitedHeap.h>
#include <GfxTL/NearestNeighbor.h>
#include <GfxTL/FlatCopyVector.h>
#include <algorithm>
#include <memory>
#include <deque>
#ifndef __APPLE__
#include <malloc.h>
#else
#include <stdlib.h>
#endif

namespace GfxTL
{
	template< class BaseT >
	class KdTreeCell
	: public BaseT
	{
		public:
			typedef KdTreeCell< BaseT > ThisType;
			typedef typename BaseT::value_type value_type;
			typedef typename ScalarTypeDeferer< value_type >::ScalarType ScalarType;
			enum { NChildren = 2 };

			KdTreeCell()
			{
				m_children[0] = m_children[1] = NULL;
			}

			~KdTreeCell()
			{
				if(m_children[0] != (KdTreeCell *)0x1)
					delete m_children[0];
				delete m_children[1];
			}

			ThisType &operator[](unsigned int i)
			{
				return *m_children[i];
			}

			const ThisType &operator[](unsigned int i) const
			{
				return *m_children[i];
			}

			void Child(unsigned int i, ThisType *cell)
			{
				m_children[i] = cell;
			}

			unsigned int &SplitAxis()
			{
				return m_splitAxis;
			}

			const unsigned int SplitAxis() const
			{
				return m_splitAxis;
			}

			ScalarType &SplitValue()
			{
				return m_splitValue;
			}

			const ScalarType SplitValue() const
			{
				return m_splitValue;
			}

			template< class VectorT >
			bool operator()(const VectorT &v) const
			{
				return v[m_splitAxis] <= m_splitValue;
			}

			/*void *operator new(size_t size)
			{
				return _mm_malloc(size, 16);
			}

			void operator delete(void *ptr)
			{
				_mm_free(ptr);
			}*/

		private:
			unsigned int m_splitAxis;
			ScalarType m_splitValue;
			ThisType *m_children[2];
	};

	template< class DataStrategyT >
	struct BaseKdTreeStrategy
	{
		typedef typename DataStrategyT::value_type value_type;

		class CellData
		: public DataStrategyT::CellData
		{};

		template< class BaseT >
		struct StrategyBase
		: public DataStrategyT::template StrategyBase< BaseT >
		{
			typedef typename DataStrategyT::template StrategyBase< BaseT >::CellType
				CellType;
			typedef typename DataStrategyT::template StrategyBase< BaseT > BaseType;
			typedef typename BaseType::DereferencedType DereferencedType;

			template< class BuildInformationT >
			bool ShouldSubdivide(const CellType &cell, BuildInformationT &bi)
			{
				return false;
			}

			template< class BuildInformationT >
			void EnterGlobalBuildInformation(const CellType &cell,
				BuildInformationT *bi)
			{}

			template< class BuildInformationT >
			void LeaveGlobalBuildInformation(const CellType &cell,
				const BuildInformationT &bi)
			{}

			template< class TraversalInformationT >
			void UpdateCellWithBack(const TraversalInformationT &,
				CellType *)
			{}

		protected:
			template< class TraversalBaseT >
			class TraversalInformationBase
			: public DataStrategyT::template StrategyBase< BaseT >
				::template TraversalInformation< TraversalBaseT >
			{};
		};
	};

	template< class StrategiesT, template< class > class MetricT,
		template< class > class VectorKernelT = DynVectorKernel >
	class KdTree
	: public StrategiesT::template StrategyBase
		<
			MetricT
			<
				VectorKernelT
				<
					BaseTree
					<
						KdTreeCell< typename StrategiesT::CellData >
					>
				>
			>
		>
	{
		public:
			typedef KdTreeCell< typename StrategiesT::CellData > CellType;
			typedef typename StrategiesT::template StrategyBase
			<
				MetricT
				<
					VectorKernelT
					<
						BaseTree
						<
							KdTreeCell< typename StrategiesT::CellData >
						>
					>
				>
			> BaseType;
			typedef typename BaseType::value_type value_type;
			typedef typename BaseType::DereferencedType DereferencedType;
			typedef typename ScalarTypeDeferer< value_type >::ScalarType ScalarType;

			class BuildInformation
			: public BaseType::BuildInformation
			{
				public:
					unsigned int &CreateChild()
					{ return m_createChild; }
					const unsigned int CreateChild() const
					{ return m_createChild; }

				private:
					unsigned int m_createChild;
			};

			template< class PointT >
			class BaseGlobalTraversalInformation
			{
				public:
					typedef PointT PointType;

					const PointT &Point() const { return m_point; }
					void Point(const PointT &p) { m_point = p; }

				private:
					PointT m_point;
			};

			template< class GlobalInfoT >
			class BaseTraversalInformation
			{
				public:
					typedef GlobalInfoT GlobalType;
					void Global(const GlobalInfoT *globalInfo)
					{ m_globalInfo = globalInfo; }
					const GlobalInfoT &Global() const
					{ return *m_globalInfo; }

				private:
					const GlobalInfoT *m_globalInfo;
			};

			void Build()
			{
				typedef std::pair< CellType *, BuildInformation > Pair;
				BaseType::Clear();
				if(!BaseType::size())
					return;
				BaseType::Root() = new CellType;
				std::deque< Pair > stack(1);
				// init build information directly on stack to avoid
				// copying.
				stack.back().first = BaseType::Root();
				InitRootBuildInformation(&stack.back().second);
				BaseType::InitRoot(stack.back().second, BaseType::Root());
				while(stack.size())
				{
					Pair &p = stack.back();
					if(p.second.CreateChild() == CellType::NChildren)
					{
						BaseType::LeaveGlobalBuildInformation(*p.first, p.second);
						stack.pop_back();
						continue;
					}
					if(BaseType::IsLeaf(*p.first))
					{
						if(!BaseType::ShouldSubdivide(*p.first, p.second))
						{
							stack.pop_back();
							continue;
						}
						Subdivide(p.second, p.first);
						if(BaseType::IsLeaf(*p.first)) // couldn't subdivide?
						{
							stack.pop_back();
							continue;
						}
					}
					else
						BaseType::LeaveGlobalBuildInformation(*p.first, p.second);
					while(p.second.CreateChild() < CellType::NChildren &&
						!this->ExistChild(*p.first, p.second.CreateChild()))
						++p.second.CreateChild();
					if(p.second.CreateChild() == CellType::NChildren)
					{
						stack.pop_back();
						continue;
					}
					BaseType::EnterGlobalBuildInformation(*p.first, &p.second);
					stack.resize(stack.size() + 1); // create new entry
					stack.back().first = &(*p.first)[p.second.CreateChild()];
					InitBuildInformation(*p.first, p.second,
						p.second.CreateChild(), &stack.back().second);
					InitCell(*p.first, p.second, p.second.CreateChild(),
						stack.back().second, &(*p.first)[p.second.CreateChild()]);
					++p.second.CreateChild();
				}
			}

			void InsertBack()
			{
				typedef typename BaseType::template GlobalTraversalInformation<
					BaseGlobalTraversalInformation< value_type > >
					GlobalInfoType;
				// traverse the tree and insert the point
				typename BaseType::template TraversalInformation<
					BaseTraversalInformation< GlobalInfoType > > ti;
				BaseType::InitRootTraversalInformation(*BaseType::Root(), &ti);
				InsertBack(*BaseType::Root(), ti);
			}

			void Remove(DereferencedType s)
			{
				Remove(*BaseType::Root(), s);				
			}

			template< class PointT, class DistScalarT, class ContainerT >
			void PointsInBall(const PointT &p,
				/*typename BaseType::template DistanceType
				<
					typename ScalarTypeDeferer< PointT >::ScalarType,
					ScalarType
				>::Type*/DistScalarT sqrRadius,
				ContainerT *points) const
			{
				typedef typename BaseType::template GlobalTraversalInformation<
					BaseGlobalTraversalInformation< PointT > >
					GlobalInfoType;
				GlobalInfoType gti;
				typename BaseType::template TraversalInformation<
					BaseTraversalInformation< GlobalInfoType > > ti;
				points->clear();
				gti.Point(p);
				ti.Global(&gti);
				InitRootTraversalInformation(*BaseType::Root(), &ti);
				PointsInBall(sqrRadius, *BaseType::Root(), ti, points);
			}

			template< class PointT >
			struct PointsInBallAuxInfo
			{
				typedef typename BaseType::template GlobalTraversalInformation<
					BaseGlobalTraversalInformation< PointT > > GlobalInfoType;
				typedef typename BaseType::template TraversalInformation<
					BaseTraversalInformation< GlobalInfoType > > TraversalInfoType;
				typedef typename BaseType::template DistanceType<
							typename ScalarTypeDeferer< PointT >::ScalarType,
							ScalarType
						>::Type DistanceType;
				PointsInBallAuxInfo() {}
				PointsInBallAuxInfo(const CellType *cell, const TraversalInfoType &ti)
				: m_cell(cell)
				, m_ti(ti)
				{}
				const CellType *m_cell;
				TraversalInfoType m_ti;
			};
			
			template< class PointT >
			struct PointsInBallAuxData
			: public FlatCopyVector< PointsInBallAuxInfo< PointT > >
			{
				typedef typename PointsInBallAuxInfo< PointT >::TraversalInfoType TraversalInfoType;
				typedef typename PointsInBallAuxInfo< PointT >::DistanceType DistanceType;
			};

			template< class PointT, class DistScalarT, class ContainerT >
			void PointsInBall(const PointT &p,
				DistScalarT sqrRadius,
				ContainerT *points, PointsInBallAuxData< PointT > *auxData) const
			{
				points->clear();
				PrivatePointsInBall(p, sqrRadius, points, auxData);
			}

			template< class PointT, class LimitedHeapT >
			void NearestNeighbors(const PointT &p, unsigned int k,
				LimitedHeapT *neighbors) const
			{
				typedef typename BaseType::template GlobalTraversalInformation<
					BaseGlobalTraversalInformation< PointT > > GlobalInfoType;
				typedef typename BaseType::template TraversalInformation<
					BaseTraversalInformation< GlobalInfoType > > TraversalInformation;
				typedef typename BaseType::template DistanceType<
							typename ScalarTypeDeferer< PointT >::ScalarType,
							ScalarType
						>::Type DistanceType;
				GlobalInfoType gti;
				TraversalInformation ti;
				neighbors->clear();
				neighbors->Limit(k);
				gti.Point(p);
				ti.Global(&gti);
				BaseType::InitRootTraversalInformation(*BaseType::Root(), &ti);
				PrivateNearestNeighbors(*BaseType::Root(), ti, neighbors);
				neighbors->AssertHeap();
			}

			template< class PointT >
			struct NearestNeighborsAuxInfo
			{
				typedef typename BaseType::template GlobalTraversalInformation<
					BaseGlobalTraversalInformation< PointT > > GlobalInfoType;
				typedef typename BaseType::template TraversalInformation<
					BaseTraversalInformation< GlobalInfoType > > TraversalInfoType;
				typedef typename BaseType::template DistanceType<
							typename ScalarTypeDeferer< PointT >::ScalarType,
							ScalarType
						>::Type DistanceType;
				NearestNeighborsAuxInfo() {}
				NearestNeighborsAuxInfo(const CellType *cell, const TraversalInfoType &ti, DistanceType sqrDist)
				: m_cell(cell)
				, m_ti(ti)
				, m_sqrDist(sqrDist)
				{}
				const CellType *m_cell;
				TraversalInfoType m_ti;
				DistanceType m_sqrDist;
			};
			
			template< class PointT >
			struct NearestNeighborsAuxData
			: public FlatCopyVector< NearestNeighborsAuxInfo< PointT > >
			{
				typedef typename NearestNeighborsAuxInfo< PointT >::TraversalInfoType TraversalInfoType;
				typedef typename NearestNeighborsAuxInfo< PointT >::DistanceType DistanceType;
			};

			template< class PointT, class LimitedHeapT >
			void NearestNeighbors(const PointT &p, unsigned int k,
				LimitedHeapT *neighbors, NearestNeighborsAuxData< PointT > *auxData) const
			{
				neighbors->clear();
				neighbors->Limit(k);
				PrivateNearestNeighbors(p, auxData, neighbors);
				neighbors->AssertHeap();
			}

			template< class PointT >
			struct NNTypeHelper {
				typedef NN<
									typename BaseType::template DistanceType<
										typename ScalarTypeDeferer< PointT >::ScalarType,
										ScalarType
									>::Type, typename BaseType::DereferencedType > 
						NNType;
			};
			
			template< class PointT, class EpsilonT, template< class > class ContainerT >
			void ApproximateNearestNeighbors(const PointT &p, unsigned int k,
				EpsilonT epsilon,
				LimitedHeap< typename NNTypeHelper<PointT>::NNType,
					std::less< typename NNTypeHelper<PointT>::NNType >,
					ContainerT > *neighbors) const
			{
				typedef typename BaseType::template GlobalTraversalInformation<
					BaseGlobalTraversalInformation< PointT > > GlobalInfoType;
				GlobalInfoType gti;
				typename BaseType::template TraversalInformation<
					BaseTraversalInformation< GlobalInfoType > > ti;
				neighbors->clear();
				neighbors->Limit(k);
				gti.Point(p);
				ti.Global(&gti);
				BaseType::InitRootTraversalInformation(*BaseType::Root(), &ti);
				ApproximateNearestNeighbors(*BaseType::Root(), ti, 1 + epsilon, neighbors);
				neighbors->AssertHeap();
			}

			template< class PointT >
			bool Contains(const PointT &p, DereferencedType *dref) const
			{
				typedef typename BaseType::template GlobalTraversalInformation<
					BaseGlobalTraversalInformation< PointT > >
					GlobalInfoType;
				GlobalInfoType gti;
				typename BaseType::template TraversalInformation<
					BaseTraversalInformation< GlobalInfoType > > ti;
				gti.Point(p);
				ti.Global(&gti);
				BaseType::InitRootTraversalInformation(*BaseType::Root(), &ti);
				return Contains(*BaseType::Root(), ti, dref);
			}

		private:
			void InitRootBuildInformation(BuildInformation *bi)
			{
				bi->CreateChild() = 0;
				BaseType::InitRootBuildInformation(bi);
			}

			void InitBuildInformation(const CellType &parent,
				const BuildInformation &parentInfo, unsigned int childIdx,
				BuildInformation *bi)
			{
				bi->CreateChild() = 0;
				BaseType::InitBuildInformation(parent, parentInfo, childIdx,
					bi);
			}

			template< class BuildInformationT >
			void InitCell(const CellType &parent,
				const BuildInformationT &parentInfo, unsigned int childIdx,
				const BuildInformationT &bi, CellType *cell)
			{
				BaseType::InitCell(parent, parentInfo, childIdx, bi, cell);
			}

			template< class TraversalInformationT >
			void InitTraversalInformation(const CellType &parent,
				const TraversalInformationT &pTi, unsigned int childIdx,
				TraversalInformationT *ti) const
			{
				ti->Global(&pTi.Global());
				BaseType::InitTraversalInformation(parent, pTi, childIdx, ti);
			}

			template< class TraversalInformationT >
			void InsertBack(CellType &cell, const TraversalInformationT &ti)
			{
				typename BaseType::CellRange range;
				BaseType::GetCellRange(cell, ti, &range);
				if(IsLeaf(cell))
				{
					BaseType::InsertBack(range, &cell);
					return;
				}
				bool l = SplitAndInsert(cell, range, &(cell[0]), &(cell[1]));
				TraversalInformationT cti;
				if(l)
				{
					UpdateCellWithBack(ti, &(cell[0]));
					InitTraversalInformation(cell, ti, 0, &cti);
					InsertBack(cell[0], cti);
				}
				else
				{
					UpdateCellWithBack(ti, &(cell[1]));
					InitTraversalInformation(cell, ti, 1, &cti);
					InsertBack(cell[1], cti);
				}
			}

			void Remove(CellType &cell, DereferencedType s)
			{
				if(IsLeaf(cell))
				{
					BaseType::Remove(s, &cell);
					return;
				}
				if(BaseType::Remove(cell, s))
					Remove(cell[0], s);
				else
					Remove(cell[1], s);
			}

			void Subdivide(BuildInformation &bi, CellType *cell)
			{
				BaseType::ComputeSplit(bi, cell);
				CellType *left, *right;
				left = new CellType;
				right = new CellType;
//				unsigned int splitIter = 1;
//splitAgain:	
				BaseType::SplitData(*cell, *cell, bi, left, right);
				if(left->Size() == cell->Size())
				{
					delete right;
					right = NULL;
				}
				else if(right->Size() == cell->Size())
				{
					delete left;
					left = NULL;
				}
				//if(left->Size() == cell->Size() || right->Size() == cell->Size())
				//{
				//	//if(splitIter < BaseType::m_dim)
				//	//{
				//	//	++splitIter;
				//	//	if(BaseType::AlternateSplit(bi, cell))
				//	//		goto splitAgain;
				//	//}
				//	for(unsigned int i = 0; i < 2; ++i)
				//	{
				//		left->Child(i, NULL);
				//		right->Child(i, NULL);
				//	}
				//	delete left;
				//	delete right;
				//	left = right = NULL;
				//}
				cell->Child(0, left);
				cell->Child(1, right);
				if(BaseType::IsLeaf(*cell))
					cell->Child(0, BaseType::InnerNodeMarker());
			}

			template< class TraversalInformationT, class DistScalarT,
				class ContainerT >
			void PointsInBall(DistScalarT sqrRadius, const CellType &cell,
				const TraversalInformationT &ti, ContainerT *points) const
			{
				if(BaseType::IsLeaf(cell))
				{
					// find all points within ball
					typename BaseType::CellRange range;
					BaseType::GetCellRange(cell, ti, &range);
					for(typename BaseType::HandleType h = range.first;
						h != range.second; ++h)
					{
						DistScalarT d = BaseType::SqrDistance(ti.Global().Point(),
							at(BaseType::Dereference(h)));
						if(d <= sqrRadius)
							points->push_back(typename ContainerT::value_type(
								d, BaseType::Dereference(h)));
					}
					return;
				}
				TraversalInformationT cti;
				// check distance to left box
				if(BaseType::ExistChild(cell, 0))
				{
					InitTraversalInformation(cell, ti, 0, &cti);
					if(BaseType::CellSqrDistance(cell[0], cti) <= sqrRadius)
						PointsInBall(sqrRadius, cell[0], cti, points);
				}
				// check distance to right box
				if(BaseType::ExistChild(cell, 1))
				{
					InitTraversalInformation(cell, ti, 1, &cti);
					if(BaseType::CellSqrDistance(cell[1], cti) <= sqrRadius)
						PointsInBall(sqrRadius, cell[1], cti, points);
				}
			}

			template< class PointT, class DistScalarT, class ContainerT >
			void PrivatePointsInBall(const PointT &p, DistScalarT sqrRadius,
				ContainerT *points, PointsInBallAuxData< PointT > *auxData) const
			{
				typedef PointsInBallAuxData< PointT > AuxDataType;
				typedef typename AuxDataType::TraversalInfoType TraversalInfoType;
				typedef typename TraversalInfoType::GlobalType GlobalInfoType;
				typedef typename AuxDataType::value_type AuxDataValueType;
				typedef typename AuxDataType::DistanceType DistanceType;
				if(!BaseType::Root())
					return;
				auxData->reserve(128);
				GlobalInfoType gti;
				TraversalInfoType ti;
				gti.Point(p);
				ti.Global(&gti);
				BaseType::InitRootTraversalInformation(*BaseType::Root(), &ti);
				const CellType *current = BaseType::Root();
				while(1)
				{
					if(BaseType::IsLeaf(*current))
					{
						typename BaseType::CellRange range;
						BaseType::GetCellRange(*current, ti, &range);
						typename BaseType::HandleType h = range.first;
						// let's find nearest neighbors from points within the cell
						for(; h != range.second; ++h)
						{
							typename ContainerT::value_type nn(
								BaseType::SqrDistance(p, BaseType::at(BaseType::Dereference(h))),
								BaseType::Dereference(h));
							if(nn.sqrDist <= sqrRadius)
								points->push_back(nn);
						}
					}
					else if(!BaseType::ExistChild(*current, 0))
					{
						TraversalInfoType rti;
						InitTraversalInformation(*current, ti, 1, &rti);
						if(BaseType::CellSqrDistance(current[1], rti) <= sqrRadius)
						{
							current = &(*current)[1];
							ti = rti;
							continue;
						}
					}
					else if(!BaseType::ExistChild(*current, 1))
					{
						TraversalInfoType lti;
						InitTraversalInformation(*current, ti, 0, &lti);
						if(BaseType::CellSqrDistance((*current)[0], lti) <= sqrRadius)
						{
							current = &(*current)[0];
							ti = lti;
							continue;
						}
					}
					else
					{
						TraversalInfoType lti, rti;
						DistanceType dl, dr;
						InitTraversalInformation(*current, ti, 0, &lti);
						InitTraversalInformation(*current, ti, 1, &rti);
						// descent into left child first
						dl = BaseType::CellSqrDistance((*current)[0], lti);
						dr = BaseType::CellSqrDistance((*current)[1], rti);
						if(dl <= sqrRadius)
						{
							if(dr <= sqrRadius)
								auxData->push_back(AuxDataValueType(&(*current)[1], rti));
							current = &(*current)[0];
							ti = lti;
							continue;
						}
						else if(dr <= sqrRadius)
						{
							current = &(*current)[1];
							ti = rti;
							continue;
						}
					}
					// pop stack
					if(!auxData->size())
						return;
					current = auxData->back().m_cell;
					ti = auxData->back().m_ti;
					auxData->pop_back();
				}
			}

			template< class TraversalInformationT, class LimitedHeapT >
			void PrivateNearestNeighbors(const CellType &cell,
				const TraversalInformationT &ti,
				LimitedHeapT *neighbors) const
			{
				typedef typename LimitedHeapT::value_type NNType;
				if(BaseType::IsLeaf(cell)
					|| (neighbors->size() + cell.Size() <= neighbors->Limit()))
				{
					typename BaseType::CellRange range;
					BaseType::GetCellRange(cell, ti, &range);
					typename BaseType::HandleType h = range.first;
					// let's find nearest neighbors from points within the cell
					for(; h != range.second; ++h)
					{
						NNType nn(BaseType::SqrDistance(ti.Global().Point(), BaseType::at(BaseType::Dereference(h))),
								BaseType::Dereference(h));
						neighbors->PushHeap(nn);
					}
					return;
				}
				if(!BaseType::ExistChild(cell, 0))
				{
					TraversalInformationT rti;
					typename NNType::ScalarType dr;
					InitTraversalInformation(cell, ti, 1, &rti);
					dr = BaseType::CellSqrDistance(cell[1], rti);
					if((neighbors->size() < neighbors->Limit()
						|| dr <= neighbors->front().sqrDist))
						PrivateNearestNeighbors(cell[1], rti, neighbors);
					return;
				}
				else if(!BaseType::ExistChild(cell, 1))
				{
					TraversalInformationT lti;
					typename NNType::ScalarType dl;
					InitTraversalInformation(cell, ti, 0, &lti);
					dl = BaseType::CellSqrDistance(cell[0], lti);
					if((neighbors->size() < neighbors->Limit()
						|| dl <= neighbors->front().sqrDist))
						PrivateNearestNeighbors(cell[0], lti, neighbors);
					return;
				}
				TraversalInformationT lti, rti;

				typename NNType::ScalarType dl, dr;
				InitTraversalInformation(cell, ti, 0, &lti);
				InitTraversalInformation(cell, ti, 1, &rti);
				// descent into closer child first
				dl = BaseType::CellSqrDistance(cell[0], lti);
				dr = BaseType::CellSqrDistance(cell[1], rti);
				if(dl <= dr)
				{
					if((neighbors->size() < neighbors->Limit()
						|| dl <= neighbors->front().sqrDist))
						PrivateNearestNeighbors(cell[0], lti, neighbors);
					if((neighbors->size() < neighbors->Limit()
						|| dr <= neighbors->front().sqrDist))
						PrivateNearestNeighbors(cell[1], rti, neighbors);
				}
				else
				{
					if((neighbors->size() < neighbors->Limit()
						|| dr <= neighbors->front().sqrDist))
						PrivateNearestNeighbors(cell[1], rti, neighbors);
					if((neighbors->size() < neighbors->Limit()
						|| dl <= neighbors->front().sqrDist))
						PrivateNearestNeighbors(cell[0], lti, neighbors);
				}
			}

			template< class PointT, class LimitedHeapT >
			void PrivateNearestNeighbors(const PointT &p,
				NearestNeighborsAuxData< PointT > *auxData,
				LimitedHeapT *neighbors) const
			{
				typedef NearestNeighborsAuxData< PointT > AuxDataType;
				typedef typename AuxDataType::TraversalInfoType TraversalInfoType;
				typedef typename TraversalInfoType::GlobalType GlobalInfoType;
				typedef typename AuxDataType::value_type AuxDataValueType;
				if(!BaseType::Root())
					return;
				auxData->reserve(128);
				GlobalInfoType gti;
				TraversalInfoType ti;
				gti.Point(p);
				ti.Global(&gti);
				BaseType::InitRootTraversalInformation(*BaseType::Root(), &ti);
				const CellType *current = BaseType::Root();
				while(1)
				{
					typedef typename LimitedHeapT::value_type NNType;
					if(BaseType::IsLeaf(*current)
						|| (neighbors->size() + current->Size() <= neighbors->Limit()))
					{
						typename BaseType::CellRange range;
						BaseType::GetCellRange(*current, ti, &range);
						typename BaseType::HandleType h = range.first;
						// let's find nearest neighbors from points within the cell
						for(; h != range.second; ++h)
						{
							NNType nn(BaseType::SqrDistance(ti.Global().Point(), BaseType::at(BaseType::Dereference(h))),
									BaseType::Dereference(h));
							neighbors->PushHeap(nn);
						}
					}
					else if(!BaseType::ExistChild(*current, 0))
					{
						TraversalInfoType rti;
						typename NNType::ScalarType dr;
						InitTraversalInformation(*current, ti, 1, &rti);
						dr = BaseType::CellSqrDistance(current[1], rti);
						if(neighbors->size() < neighbors->Limit()
							|| dr <= neighbors->front().sqrDist)
						{
							current = &(*current)[1];
							ti = rti;
							continue;
						}
					}
					else if(!BaseType::ExistChild(*current, 1))
					{
						TraversalInfoType lti;
						typename NNType::ScalarType dl;
						InitTraversalInformation(*current, ti, 0, &lti);
						dl = BaseType::CellSqrDistance((*current)[0], lti);
						if(neighbors->size() < neighbors->Limit()
							|| dl <= neighbors->front().sqrDist)
						{
							current = &(*current)[0];
							ti = lti;
							continue;
						}
					}
					else
					{
						TraversalInfoType lti, rti;
						typename NNType::ScalarType dl, dr;
						InitTraversalInformation(*current, ti, 0, &lti);
						InitTraversalInformation(*current, ti, 1, &rti);
						// descent into closer child first
						dl = BaseType::CellSqrDistance((*current)[0], lti);
						dr = BaseType::CellSqrDistance((*current)[1], rti);
						if(dl <= dr)
						{
							if(neighbors->size() < neighbors->Limit()
								|| dl <= neighbors->front().sqrDist)
							{
								auxData->push_back(AuxDataValueType(&(*current)[1], rti, dr));
								current = &(*current)[0];
								ti = lti;
								continue;
							}
							if((neighbors->size() < neighbors->Limit()
								|| dr <= neighbors->front().sqrDist))
							{
								current = &(*current)[1];
								ti = rti;
								continue;
							}
						}
						else
						{
							if(neighbors->size() < neighbors->Limit()
								|| dr <= neighbors->front().sqrDist)
							{
								auxData->push_back(AuxDataValueType(&(*current)[0], lti, dl));
								current = &(*current)[1];
								ti = rti;
								continue;
							}
							if((neighbors->size() < neighbors->Limit()
								|| dl <= neighbors->front().sqrDist))
							{
								current = &(*current)[0];
								ti = lti;
								continue;
							}
						}
					}
					// pop stack
					while(1)
					{
						if(!auxData->size())
							return;
						if(neighbors->size() < neighbors->Limit()
							|| auxData->back().m_sqrDist <= neighbors->front().sqrDist)
						{
							current = auxData->back().m_cell;
							ti = auxData->back().m_ti;
							auxData->pop_back();
							break;
						}
						auxData->pop_back();
					}
				}
			}

			template< class TraversalInformationT, class NNT,
				class EpsilonT, template< class > class ContainerT >
			void ApproximateNearestNeighbors(const CellType &cell,
				const TraversalInformationT &ti, EpsilonT epsilon,
				LimitedHeap< NNT, std::less<NNT>, ContainerT > *neighbors) const
			{
				typedef NNT NNType;
				if(IsLeaf(cell)
					|| (neighbors->size() + cell.Size() <= neighbors->Limit()))
				{
					typename BaseType::CellRange range;
					GetCellRange(cell, ti, &range);
					typename BaseType::HandleType h = range.first,
						hend = range.second;
					// let's find nearest neighbors from points within the cell
					for(; h != hend; ++h)
					{
						neighbors->PushHeap(NNType(
							SqrDistance(ti.Global().Point(), at(Dereference(h))),
							Dereference(h)));
					}
					return;
				}
				TraversalInformationT lti, rti;
				InitTraversalInformation(cell, ti, 0, &lti);
				InitTraversalInformation(cell, ti, 1, &rti);
				// descent into closer child first
				typename LimitedHeap< NNT, std::less<NNT>, ContainerT >::value_type::ScalarType
					dl, dr;
				dl = CellSqrDistance(cell[0], lti);
				dr = CellSqrDistance(cell[1], rti);
				if(dl <= dr)
				{
					if((neighbors->size() < neighbors->Limit()
						|| epsilon * dl <= neighbors->front().sqrDist)
						&& BaseType::ExistChild(cell, 0))
						ApproximateNearestNeighbors(cell[0], lti, epsilon, neighbors);
					if((neighbors->size() < neighbors->Limit()
						|| epsilon * dr <= neighbors->front().sqrDist)
						&& BaseType::ExistChild(cell, 1))
						ApproximateNearestNeighbors(cell[1], rti, epsilon, neighbors);
				}
				else
				{
					if((neighbors->size() < neighbors->Limit()
						|| epsilon * dr <= neighbors->front().sqrDist)
						&& BaseType::ExistChild(cell, 1))
						ApproximateNearestNeighbors(cell[1], rti, epsilon, neighbors);
					if((neighbors->size() < neighbors->Limit()
						|| epsilon * dl <= neighbors->front().sqrDist)
						&& BaseType::ExistChild(cell, 0))
						ApproximateNearestNeighbors(cell[0], lti, epsilon, neighbors);
				}
			}

			template< class TraversalInformationT >
			bool Contains(const CellType &cell, const TraversalInformationT &ti,
				DereferencedType *dref) const
			{
				if(IsLeaf(cell))
				{
					typename BaseType::CellRange range;
					GetCellRange(cell, ti, &range);
					typename BaseType::HandleType h = range.first,
						hend = range.second;
					for(; h != hend; ++h)
						if(at(Dereference(h)) == ti.Global().Point())
						{
							*dref = Dereference(h);
							return true;
						}
					return false;
				}
				if(cell(ti.Global().Point()))
				{
					if(!BaseType::ExistChild(cell, 0)) return false;
					TraversalInformationT lti;
					InitTraversalInformation(cell, ti, 0, &lti);
					return Contains(cell[0], lti, dref);
				}
				else
				{
					if(!BaseType::ExistChild(cell, 1)) return false;
					TraversalInformationT rti;
					InitTraversalInformation(cell, rti, 1, &rti);
					return Contains(cell[1], rti, dref);
				}
			}
	};
};

#endif
