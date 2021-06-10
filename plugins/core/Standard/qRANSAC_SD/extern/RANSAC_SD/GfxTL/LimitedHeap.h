#ifndef __GfxTL_LIMITEDHEAP_HEADER__
#define __GfxTL_LIMITEDHEAP_HEADER__
#include <algorithm>
#include <functional>
#include <vector>
#include <GfxTL/FlatCopyVector.h>
#include <GfxTL/StdContainerAdaptor.h>
#include <functional>

namespace GfxTL
{
	template< class T, class PredicateT = std::less< T >,
		template< class > class ContainerT = FlatCopyVector >
	class LimitedHeap
	: public ContainerT< T >
	{
		public:
			typedef typename ContainerT< T >::value_type value_type;
			typedef typename ContainerT< T >::size_type size_type;
			typedef PredicateT PredicateType;
			typedef ContainerT< T > BaseType;

			LimitedHeap()
			: _limit(-1)
			, _instances(0)
			{}

			LimitedHeap(size_type limit)
			: _limit(limit)
			, _instances(0)
			{
				BaseType::reserve(limit);
			}

			LimitedHeap(const PredicateType &pred)
			: m_pred(pred)
			, _limit(-1)
			, _instances(0)
			{
				BaseType::reserve(_limit);
			}

			LimitedHeap(size_type limit, const PredicateType &pred)
			: m_pred(pred)
			, _limit(limit)
			, _instances(0)
			{
				BaseType::reserve(limit);
			}

			void Limit(size_type limit)
			{
				_limit = limit;
				BaseType::reserve(limit + 1);
			}

			size_type Limit() const
			{
				return _limit;
			}

			void Predicate(const PredicateType &pred)
			{
				m_pred = pred;
			}

			const PredicateType &Predicate() const
			{
				return m_pred;
			}

			void clear()
			{
				BaseType::clear();
				_instances = 0;
			}

			void MakeHeap()
			{
				size_type iend = BaseType::size();
				for(size_type i = BaseType::size() >> 1; i > 0; --i)
					Heapify(i - 1, iend);
			}

			void SortHeap()
			{
				if(!BaseType::size())
					return;
				for(size_type i = BaseType::size() - 1; i > 0; --i)
				{
					T tmp = BaseType::at(i);
					BaseType::at(i) = BaseType::at(0);
					BaseType::at(0) = tmp;
					Heapify(0, i);
				}
			}

			void AssertHeap()
			{
				if(BaseType::size() < _limit)
					MakeHeap();
			}

			void PushHeap(const value_type &t)
			{
				if(BaseType::size() < _limit)
				{
					BaseType::push_back(t);
					if(BaseType::size() == _limit)
					{
						MakeHeap();
						_instances = CountInstances(0);
					}
				}
				else
				{
					if(m_pred(BaseType::front(), t)) // front < t
						return; // t > front
					if(m_pred(t, BaseType::front())) // t < front?
					{
						if(_instances > 1)
						{
							if(BaseType::size() + 1 - _instances >= _limit)
							{
								// remove instances
								for(size_type i = 0; i < _instances; ++i)
								{
									std::swap(BaseType::front(), BaseType::back());
									BaseType::pop_back();
									Heapify(0, BaseType::size());
								}
								BaseType::push_back(t);
								BubbleDown();
								_instances = CountInstances(0);
								return;
							}
							BaseType::push_back(t);
							BubbleDown();
							return;
						}
						BaseType::front() = t;
						Heapify(0, BaseType::size());
						_instances = CountInstances(0);
						return;
					}
					// t == front;
					BaseType::push_back(t);
					BubbleDown();
					++_instances;
				}
			}

		private:
			void Heapify(size_type i, size_type iend)
			{
				using namespace std;
				size_type max;
				while((max = (i << 1) + 1) < iend)
				{
					if(max + 1 < iend && m_pred(BaseType::at(max), BaseType::at(max + 1)))
						++max;
					if(!m_pred(BaseType::at(i), BaseType::at(max)))
						break;
					std::swap(BaseType::at(i), BaseType::at(max));
					i = max;
				}
			}

			void BubbleDown()
			{
				using namespace std;
				if(BaseType::size() == 1)
					return;
				size_type i = BaseType::size() - 1, j;
				while(i && m_pred(BaseType::at(j = ((i + 1) >> 1) - 1), BaseType::at(i)))
				{
					std::swap(BaseType::at(i), BaseType::at(j));
					i = j;
				}
			}

			size_type CountInstances(size_type i) const
			{
				size_type child = (i << 1) + 1;
				if(child >= BaseType::size())
					return 1;
				size_type instances = 0;
				if(!m_pred(BaseType::at(child), BaseType::at(i)))
					instances = CountInstances(child);
				++child;
				if(child >= BaseType::size())
					return 1 + instances;
				if(!m_pred(BaseType::at(child), BaseType::at(i)))
					instances += CountInstances(child);
				return 1 + instances;
			}

		private:
			PredicateType m_pred;
			size_type _limit;
			size_type _instances;
	};

	template< class T, template< class > class ContainerT = FlatCopyVector >
	class AssumeUniqueLimitedHeap
	: public ContainerT< T >
	{
		public:
			typedef typename ContainerT< T >::value_type value_type;
			typedef typename ContainerT< T >::size_type size_type;
			typedef ContainerT< T > BaseType;

			AssumeUniqueLimitedHeap()
			: _limit(-1)
			{}

			AssumeUniqueLimitedHeap(size_type limit)
			: _limit(limit)
			{
				BaseType::reserve(limit);
			}

			void Limit(size_type limit)
			{
				_limit = limit;
				BaseType::reserve(limit + 1);
			}

			size_type Limit() const
			{
				return _limit;
			}

			void MakeHeap()
			{
				size_type iend = BaseType::size();
				for(size_type i = BaseType::size() >> 1; i > 0; --i)
					Heapify(i - 1, iend);
			}

			void SortHeap()
			{
				if(!BaseType::size())
					return;
				for(size_type i = BaseType::size() - 1; i > 0; --i)
				{
					T tmp = BaseType::at(i);
					BaseType::at(i) = BaseType::at(0);
					BaseType::at(0) = tmp;
					Heapify(0, i);
				}
			}

			void AssertHeap()
			{
				if(BaseType::size() < _limit)
					MakeHeap();
			}

			void PushHeap(const value_type &t)
			{
				if(BaseType::size() < _limit)
				{
					BaseType::push_back(t);
					if(BaseType::size() == _limit)
						MakeHeap();
				}
				else
				{
					if(t >= BaseType::front())
						return;
					BaseType::front() = t;
					Heapify(0, BaseType::size());
				}
			}

		private:
			void Heapify(size_type i, size_type iend)
			{
				size_type max;
				while((max = (i << 1) + 1) < iend)
				{
					if(max + 1 < iend && BaseType::at(max) < BaseType::at(max + 1))
						++max;
					if(BaseType::at(i) >= BaseType::at(max))
						break;
					std::swap(BaseType::at(i), BaseType::at(max));
					i = max;
				}
			}

			void BubbleDown()
			{
				if(BaseType::size() == 1)
					return;
				size_type i = BaseType::size() - 1, j;
				while(i && BaseType::at(i) > BaseType::at(j = ((i + 1) >> 1) - 1))
				{
					std::swap(BaseType::at(i), BaseType::at(j));
					i = j;
				}
			}

		private:
			size_type _limit;
	};
};

#endif
