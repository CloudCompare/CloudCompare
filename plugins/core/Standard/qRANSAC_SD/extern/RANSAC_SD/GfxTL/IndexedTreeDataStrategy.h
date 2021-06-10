#ifndef __GfxTL_INDEXEDTREEDATASTRATEGY_HEADER__
#define __GfxTL_INDEXEDTREEDATASTRATEGY_HEADER__

#include <vector>
#include <list>

namespace GfxTL
{

	template< class InheritedStrategy, class Data >
	class IndexedTreeDataStrategy
	{
		private:
			template< class Data >
			class IndexedTreeDataCore
			{
				public:
					typedef Data DataType;
					typedef typename Data::PointType PointType;
					typedef typename PointType::ScalarType ScalarType;

					Data &ContainedData()
					{
						return _data;
					}

					const Data &ContainedData() const
					{
						return _data;
					}

					PointType &PointTranslated(size_t i)
					{
						return _data.Point(i);
					}

					const PointType PointTranslated(size_t i) const
					{
						return _data.Point(i);
					}

					PointType &operator[](size_t i)
					{
						return _data.Point(i);
					}

					const PointType operator[](size_t i) const
					{
						return _data.Point(i);
					}

				private:
					Data _data;
			};

			template< class Data >
			class IndexedTreeDataCore< const Data >
			{
				public:
					typedef const Data DataType;
					typedef typename Data::PointType PointType;
					typedef typename PointType::ScalarType ScalarType;

					Data &NonConstContainedData()
					{
						return _data;
					}

					const Data &ContainedData() const
					{
						return _data;
					}

					const PointType PointTranslated(size_t i) const
					{
						return _data.Point(i);
					}

					const PointType operator[](size_t i) const
					{
						return _data.Point(i);
					}

				private:
					Data _data;
			};

			template< class Data >
			class IndexedTreeDataCore< Data * >
			{
				public:
					typedef Data DataType;
					typedef typename Data::PointType PointType;
					typedef typename PointType::ScalarType ScalarType;

					IndexedTreeDataCore()
					: _data(NULL)
					{}

					Data &ContainedData()
					{
						return *_data;
					}

					const Data &ContainedData() const
					{
						return *_data;
					}

					void ContainedDataPtr(Data *data)
					{
						_data = data;
					}

					PointType &PointTranslated(size_t i)
					{
						return _data->Point(i);
					}

					const PointType PointTranslated(size_t i) const
					{
						return _data->Point(i);
					}

					PointType &operator[](size_t i)
					{
						return _data->Point(i);
					}

					const PointType operator[](size_t i) const
					{
						return _data->Point(i);
					}

				private:
					Data *_data;
			};

			template< class Data >
			class IndexedTreeDataCore< const Data * >
			{
				public:
					typedef const Data DataType;
					typedef typename Data::PointType PointType;
					typedef typename PointType::ScalarType ScalarType;

					const Data &ContainedData() const
					{
						return *_data;
					}

					void ContainedDataPtr(const Data *data)
					{
						_data = data;
					}

					const PointType PointTranslated(size_t i) const
					{
						return _data->Point(i);
					}

					const PointType operator[](size_t i) const
					{
						return _data->Point(i);
					}

				private:
					const Data *_data;
			};

		public:
			typedef size_t HandleType;
			typedef std::vector< HandleType > IndicesType;
			typedef size_t CellIndicesType;
			typedef typename IndexedTreeDataCore< Data >::DataType
				DataType;
			typedef typename DataType::PointType PointType;
			typedef typename PointType::ScalarType ScalarType;
			

			template< class Point >
			class CellData
			: public InheritedStrategy::CellData< Point >
			{
				public:
					typedef CellData< Point > ThisType;
					CellData()
					: _size(0)
					{}

					CellData(const CellIndicesType &indices, size_t size)
					: _indices(indices)
					, _size(size)
					{}

					size_t Size() const
					{
						return _size;
					}

					void Size(size_t size)
					{
						_size = size;
					}

					const CellIndicesType Points() const
					{
						return _indices;
					}

					const CellIndicesType Normals() const
					{
						return _indices;
					}

					void Data(const CellIndicesType indices,
						size_t size)
					{
						_indices = indices;
						_size = size;
					}

					void Data(const ThisType &c)
					{
						_indices = c._indices;
						_size = c._size;
					}

					CellIndicesType Indices() const
					{
						return _indices;
					}

					void Indices(CellIndicesType indices)
					{
						_indices = indices;
					}

				private:
					CellIndicesType _indices;
					size_t _size;
			};

			template< class Base >
			class StrategyBase
			: public InheritedStrategy::StrategyBase< Base >
			, public IndexedTreeDataCore< Data >
			{
				public:
					typedef typename Base::CellType CellType;
					typedef typename IndexedTreeDataCore< Data >::DataType
						DataType;
					typedef typename DataType::PointType PointType;
					typedef typename PointType::ScalarType ScalarType;
					typedef	typename
						IndexedTreeDataStrategy
						<
							InheritedStrategy,
							Data
						>::HandleType HandleType;
					typedef typename
						IndexedTreeDataStrategy
						<
							InheritedStrategy,
							Data
						>::IndicesType IndicesType;
					typedef typename InheritedStrategy::StrategyBase< Base >
						StrategyBaseType;

					StrategyBase()
					{}

					void Clear()
					{
						StrategyBaseType::Clear();
						_indices.clear();
					}

					size_t Translate(HandleType i) const
					{
						return _indices[i];
					}

					size_t Size() const
					{
						return ContainedData().Size();
					}

					void PointsInCell(const CellType &cell,
						std::vector< size_t > *points) const
					{
						points->clear();
						for(size_t i = 0; i < cell.Size(); ++i)
							points->push_back(Translate(cell.Points() + i));
					}

					void BoundingCube(AACube< PointType > *cube)
					{
						cube->Bound(*this, Size());
					}

					template< class BV >
					void BoundingVolume(BV *bv)
					{
						bv->Bound(*this, Size());
					}

					void Add(const DataType &data)
					{
						ContainedData().Add(data);
						AACube< PointType > bc;
						bc.Bound(data.Points(), data.Size());
						RefreshWithNewTreeData(bc);
					}

					template< class T >
					void Transform(const T &t)
					{
						ContainedData().Transform(t);
						AACube< PointType > bc; // is infinite
						RefreshWithNewTreeData(bc);
					}

					template< class T >
					void Transform(size_t i, const T &t)
					{
						ContainedData().Transform(i, t);
						AACube< PointType > bc; // is infinite
						RefreshWithNewTreeData(bc);
					}

					void Translate(const PointType &translation)
					{
						ContainedData().Translate(translation);
						AACube< PointType > bc; // is infinite
						RefreshWithNewTreeData(bc);
					}

					template< class TFunc >
					void TransformFunc(TFunc tfunc)
					{
						ContainedData().TransformFunc(tfunc);
						AACube< PointType > bc; // is infinite
						RefreshWithNewTreeData(bc);
					}

					template< class BoundingVolume >
					void Remove(const BoundingVolume &bv)
					{
						ContainedData().Remove(bv);
						AACube< PointType > bc; // is infinite
						RefreshWithNewTreeData(bc);
					}

					virtual void RefreshWithNewTreeData(
						const AACube< PointType > &bc)
					{
						if(_indices.size() != ContainedData().Size())
						{
							size_t oldSize = _indices.size();
							_indices.resize(ContainedData().Size());
							for(size_t i = oldSize; i < _indices.size(); ++i)
								_indices[i] = i;
						}
					}

				protected:
					template< class BoundingVolume >
					void RootCellData(const BoundingVolume &bv, CellType *cell)
					{
						_indices.resize(ContainedData().Size());
						for(size_t i = 0; i < ContainedData().Size(); ++i)
							_indices[i] = i;
						IndicesType::iterator j = _indices.begin();
						IndicesType::iterator k = _indices.begin() +
							_indices.size() - 1;
						for(; j <= k;)
						{
							if(!bv.IsInside(
								ContainedData().Point(*j)))
							{
								HandleType q = *k;
								*k = *j;
								*j = q;
								--k;
							}
							else
								++j;
						}
						cell->Data(0, j - _indices.begin());
					}

					void SplitAlongAxis(const CellType &cell, unsigned int axis,
						ScalarType s, CellType *left, CellType *right)
					{
						if(cell.Size() == 0)
						{
							right->Data(cell);
							left->Data(cell);
							return;
						}
						IndicesType::iterator j = _indices.begin() +
							cell.Indices();
						IndicesType::iterator k = _indices.begin() +
							cell.Indices() + cell.Size() - 1;
						for(; j <= k;)
						{
							if(ContainedData().Point(*j)[axis] > s)
							{
								HandleType q = *k;
								*k = *j;
								*j = q;
								--k;
							}
							else
								++j;
						}
						left->Indices(cell.Indices());
						left->Size(j - (_indices.begin() + cell.Indices()));
						right->Indices(cell.Indices() + left->Size());
						right->Size((cell.Indices() + cell.Size())
							- right->Indices());
					}

					void BoundingCube(const CellType &data, AACube< PointType > *cube)
					{
						cube->BoundingCube(
							IndexPoints(data, data.Indices(), data.Size()), data.Size());
					}

				private:
					IndicesType _indices;
			};
	};

};

#endif