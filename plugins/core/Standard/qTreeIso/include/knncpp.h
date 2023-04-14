/* knncpp.h
 *
 * Author:     Fabian Meyer
 * Created On: 22 Aug 2021
 * License:    MIT
 */

#ifndef KNNCPP_H_
#define KNNCPP_H_

#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <set>

#ifdef KNNCPP_FLANN

#include <flann/flann.hpp>

#endif

namespace knncpp
{
    /********************************************************
     * Matrix Definitions
     *******************************************************/

    typedef typename Eigen::MatrixXd::Index Index;

    typedef Eigen::Matrix<Index, Eigen::Dynamic, 1> Vectori;
    typedef Eigen::Matrix<Index, 2, 1> Vector2i;
    typedef Eigen::Matrix<Index, 3, 1> Vector3i;
    typedef Eigen::Matrix<Index, 4, 1> Vector4i;
    typedef Eigen::Matrix<Index, 5, 1> Vector5i;

    typedef Eigen::Matrix<Index, Eigen::Dynamic, Eigen::Dynamic> Matrixi;
    typedef Eigen::Matrix<Index, 2, 2> Matrix2i;
    typedef Eigen::Matrix<Index, 3, 3> Matrix3i;
    typedef Eigen::Matrix<Index, 4, 4> Matrix4i;
    typedef Eigen::Matrix<Index, 5, 5> Matrix5i;

    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrixf;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrixd;

    /********************************************************
     * Distance Functors
     *******************************************************/

    /** Manhatten distance functor.
      * This the same as the L1 minkowski distance but more efficient.
      * @see EuclideanDistance, ChebyshevDistance, MinkowskiDistance */
    template <typename Scalar>
    struct ManhattenDistance
    {
        /** Compute the unrooted distance between two vectors.
          * @param lhs vector on left hand side
          * @param rhs vector on right hand side */
        template<typename DerivedA, typename DerivedB>
        Scalar operator()(const Eigen::MatrixBase<DerivedA> &lhs,
            const Eigen::MatrixBase<DerivedB> &rhs) const
        {
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedA>::Scalar,Scalar>::value,
                "distance scalar and input matrix A must have same type");
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedB>::Scalar, Scalar>::value,
                "distance scalar and input matrix B must have same type");

            return (lhs - rhs).cwiseAbs().sum();
        }

        /** Compute the unrooted distance between two scalars.
          * @param lhs scalar on left hand side
          * @param rhs scalar on right hand side */
        Scalar operator()(const Scalar lhs,
            const Scalar rhs) const
        {
            return std::abs(lhs - rhs);
        }

        /** Compute the root of a unrooted distance value.
          * @param value unrooted distance value */
        Scalar operator()(const Scalar val) const
        {
            return val;
        }
    };

    /** Euclidean distance functor.
      * This the same as the L2 minkowski distance but more efficient.
      * @see ManhattenDistance, ChebyshevDistance, MinkowskiDistance */
    template <typename Scalar>
    struct EuclideanDistance
    {
        /** Compute the unrooted distance between two vectors.
          * @param lhs vector on left hand side
          * @param rhs vector on right hand side */
        template<typename DerivedA, typename DerivedB>
        Scalar operator()(const Eigen::MatrixBase<DerivedA> &lhs,
            const Eigen::MatrixBase<DerivedB> &rhs) const
        {
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedA>::Scalar,Scalar>::value,
                "distance scalar and input matrix A must have same type");
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedB>::Scalar, Scalar>::value,
                "distance scalar and input matrix B must have same type");

            return (lhs - rhs).cwiseAbs2().sum();
        }

        /** Compute the unrooted distance between two scalars.
          * @param lhs scalar on left hand side
          * @param rhs scalar on right hand side */
        Scalar operator()(const Scalar lhs,
            const Scalar rhs) const
        {
            Scalar diff = lhs - rhs;
            return diff * diff;
        }

        /** Compute the root of a unrooted distance value.
          * @param value unrooted distance value */
        Scalar operator()(const Scalar val) const
        {
            return std::sqrt(val);
        }
    };

    /** General minkowski distance functor.
      * The infinite version is only available through the chebyshev distance.
      * @see ManhattenDistance, EuclideanDistance, ChebyshevDistance  */
    template <typename Scalar, int P>
    struct MinkowskiDistance
    {
        struct Pow
        {
            Scalar operator()(const Scalar val) const
            {
                Scalar result = 1;
                for(int i = 0; i < P; ++i)
                    result *= val;
                return result;
            }
        };

        /** Compute the unrooted distance between two vectors.
          * @param lhs vector on left hand side
          * @param rhs vector on right hand side */
        template<typename DerivedA, typename DerivedB>
        Scalar operator()(const Eigen::MatrixBase<DerivedA> &lhs,
            const Eigen::MatrixBase<DerivedB> &rhs) const
        {
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedA>::Scalar,Scalar>::value,
                "distance scalar and input matrix A must have same type");
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedB>::Scalar, Scalar>::value,
                "distance scalar and input matrix B must have same type");

            return (lhs - rhs).cwiseAbs().unaryExpr(MinkowskiDistance::Pow()).sum();
        }

        /** Compute the unrooted distance between two scalars.
          * @param lhs scalar on left hand side
          * @param rhs scalar on right hand side */
        Scalar operator()(const Scalar lhs,
            const Scalar rhs) const
        {
            return std::pow(std::abs(lhs - rhs), P);;
        }

        /** Compute the root of a unrooted distance value.
          * @param value unrooted distance value */
        Scalar operator()(const Scalar val) const
        {
            return std::pow(val, 1 / static_cast<Scalar>(P));
        }
    };

    /** Chebyshev distance functor.
      * This distance is the same as infinity minkowski distance.
      * @see ManhattenDistance, EuclideanDistance, MinkowskiDistance */
    template<typename Scalar>
    struct ChebyshevDistance
    {
        /** Compute the unrooted distance between two vectors.
          * @param lhs vector on left hand side
          * @param rhs vector on right hand side */
        template<typename DerivedA, typename DerivedB>
        Scalar operator()(const Eigen::MatrixBase<DerivedA> &lhs,
            const Eigen::MatrixBase<DerivedB> &rhs) const
        {
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedA>::Scalar,Scalar>::value,
                "distance scalar and input matrix A must have same type");
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedB>::Scalar, Scalar>::value,
                "distance scalar and input matrix B must have same type");

            return (lhs - rhs).cwiseAbs().maxCoeff();
        }

        /** Compute the unrooted distance between two scalars.
          * @param lhs scalar on left hand side
          * @param rhs scalar on right hand side */
        Scalar operator()(const Scalar lhs,
            const Scalar rhs) const
        {
            return std::abs(lhs - rhs);
        }

        /** Compute the root of a unrooted distance value.
          * @param value unrooted distance value */
        Scalar operator()(const Scalar val) const
        {
            return val;
        }
    };

    /** Hamming distance functor.
      * The distance vectors have to be of integral type and should hold the
      * information vectors as bitmasks.
      * Performs a XOR operation on the vectors and counts the number of set
      * ones. */
    template<typename Scalar>
    struct HammingDistance
    {
        static_assert(std::is_integral<Scalar>::value,
            "HammingDistance requires integral Scalar type");

        struct XOR
        {
            Scalar operator()(const Scalar lhs, const Scalar rhs) const
            {
                return lhs ^ rhs;
            }
        };

        struct BitCount
        {
            Scalar operator()(const Scalar lhs) const
            {
                Scalar copy = lhs;
                Scalar result = 0;
                while(copy != static_cast<Scalar>(0))
                {
                    ++result;
                    copy &= (copy - 1);
                }

                return result;
            }
        };

        /** Compute the unrooted distance between two vectors.
          * @param lhs vector on left hand side
          * @param rhs vector on right hand side */
        template<typename DerivedA, typename DerivedB>
        Scalar operator()(const Eigen::MatrixBase<DerivedA> &lhs,
            const Eigen::MatrixBase<DerivedB> &rhs) const
        {
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedA>::Scalar,Scalar>::value,
                "distance scalar and input matrix A must have same type");
            static_assert(
                std::is_same<typename Eigen::MatrixBase<DerivedB>::Scalar, Scalar>::value,
                "distance scalar and input matrix B must have same type");

            return lhs.
                binaryExpr(rhs, XOR()).
                unaryExpr(BitCount()).
                sum();
        }

        /** Compute the unrooted distance between two scalars.
          * @param lhs scalar on left hand side
          * @param rhs scalar on right hand side */
        Scalar operator()(const Scalar lhs,
            const Scalar rhs) const
        {
            BitCount cnt;
            XOR xOr;
            return cnt(xOr(lhs, rhs));
        }

        /** Compute the root of a unrooted distance value.
          * @param value unrooted distance value */
        Scalar operator()(const Scalar value) const
        {
            return value;
        }
    };

    /** Efficient heap structure to query nearest neighbours. */
    template<typename Scalar>
    class QueryHeap
    {
    private:
        Index *indices_ = nullptr;
        Scalar *distances_ = nullptr;
        size_t maxSize_ = 0;
        size_t size_ = 0;
    public:
        /** Creates a query heap with the given index and distance memory regions. */
        QueryHeap(Index *indices, Scalar *distances, const size_t maxSize)
            : indices_(indices), distances_(distances), maxSize_(maxSize)
        { }

        /** Pushes a new query data set into the heap with the given
          * index and distance.
          * The index identifies the point for which the given distance
          * was computed.
          * @param idx index / ID of the query point
          * @param dist distance that was computed for the query point*/
        void push(const Index idx, const Scalar dist)
        {
            assert(!full());

            // add new value at the end
            indices_[size_] = idx;
            distances_[size_] = dist;
            ++size_;

            // upheap
            size_t k = size_ - 1;
            size_t tmp = (k - 1) / 2;
            while(k > 0 && distances_[tmp] < dist)
            {
                distances_[k] = distances_[tmp];
                indices_[k] = indices_[tmp];
                k = tmp;
                tmp = (k - 1) / 2;
            }
            distances_[k] = dist;
            indices_[k] = idx;
        }

        /** Removes the element at the front of the heap and restores
          * the heap order. */
        void pop()
        {
            assert(!empty());

            // replace first element with last
            --size_;
            distances_[0] = distances_[size_];
            indices_[0] = indices_[size_];

            // downheap
            size_t k = 0;
            size_t j;
            Scalar dist = distances_[0];
            Index idx = indices_[0];
            while(2 * k + 1 < size_)
            {
                j = 2 * k + 1;
                if(j + 1 < size_ && distances_[j+1] > distances_[j])
                    ++j;
                // j references now greatest child
                if(dist >= distances_[j])
                    break;
                distances_[k] = distances_[j];
                indices_[k] = indices_[j];
                k = j;
            }
            distances_[k] = dist;
            indices_[k] = idx;
        }

        /** Returns the distance of the element in front of the heap. */
        Scalar front() const
        {
            assert(!empty());
            return distances_[0];
        }

        /** Determines if this query heap is full.
          * The heap is considered full if its number of elements
          * has reached its max size.
          * @return true if the heap is full, else false */
        bool full() const
        {
            return size_ >= maxSize_;
        }

        /** Determines if this query heap is empty.
          * @return true if the heap contains no elements, else false */
        bool empty() const
        {
            return size_ == 0;
        }

        /** Returns the number of elements within the query heap.
          * @return number of elements in the heap */
        size_t size() const
        {
            return size_;
        }

        /** Clears the query heap. */
        void clear()
        {
            size_ = 0;
        }

        /** Sorts the elements within the heap according to
          * their distance. */
        void sort()
        {
            size_t cnt = size_;
            for(size_t i = 0; i < cnt; ++i)
            {
                Index idx = indices_[0];
                Scalar dist = distances_[0];
                pop();
                indices_[cnt - i - 1] = idx;
                distances_[cnt - i - 1] = dist;
            }
        }
    };

    /** Class for performing brute force knn search. */
    template<typename Scalar,
        typename Distance=EuclideanDistance<Scalar>>
    class BruteForce
    {
    public:
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
        typedef knncpp::Matrixi Matrixi;
    private:
        Distance distance_ = Distance();
        Matrix dataCopy_ = Matrix();
        const Matrix *data_ = nullptr;

        bool sorted_ = true;
        bool takeRoot_ = true;
        Index threads_ = 1;
        Scalar maxDist_ = 0;

    public:

        BruteForce() = default;

        /** Constructs a brute force instance with the given data.
          * @param data NxM matrix, M points of dimension N
          * @param copy if true copies the data, otherwise assumes static data */
        BruteForce(const Matrix &data, const bool copy = false)
            : BruteForce()
        {
            setData(data, copy);
        }

        /** Set if the points returned by the queries should be sorted
          * according to their distance to the query points.
          * @param sorted sort query results */
        void setSorted(const bool sorted)
        {
            sorted_ = sorted;
        }

        /** Set if the distances after the query should be rooted or not.
          * Taking the root of the distances increases query time, but the
          * function will return true distances instead of their powered
          * versions.
          * @param takeRoot set true if root should be taken else false */
        void setTakeRoot(const bool takeRoot)
        {
            takeRoot_ = takeRoot;
        }

        /** Set the amount of threads that should be used for querying.
          * OpenMP has to be enabled for this to work.
          * @param threads amount of threads, 0 for optimal choice */
        void setThreads(const unsigned int threads)
        {
            threads_ = threads;
        }

        /** Set the maximum distance for querying the tree.
          * The search will be pruned if the maximum distance is set to any
          * positive number.
          * @param maxDist maximum distance, <= 0 for no limit */
        void setMaxDistance(const Scalar maxDist)
        {
            maxDist_ = maxDist;
        }

        /** Set the data points used for this tree.
          * This does not build the tree.
          * @param data NxM matrix, M points of dimension N
          * @param copy if true data is copied, assumes static data otherwise */
        void setData(const Matrix &data, const bool copy = false)
        {
            if(copy)
            {
                dataCopy_ = data;
                data_ = &dataCopy_;
            }
            else
            {
                data_ = &data;
            }
        }

        void setDistance(const Distance &distance)
        {
            distance_ = distance;
        }

        void build()
        { }

        template<typename Derived>
        void query(const Eigen::MatrixBase<Derived> &queryPoints,
            const size_t knn,
            Matrixi &indices,
            Matrix &distances) const
        {
            if(data_ == nullptr)
                throw std::runtime_error("cannot query BruteForce: data not set");
            if(data_->size() == 0)
                throw std::runtime_error("cannot query BruteForce: data is empty");
            if(queryPoints.rows() != dimension())
                throw std::runtime_error("cannot query BruteForce: data and query descriptors do not have same dimension");

            const Matrix &dataPoints = *data_;

            indices.setConstant(knn, queryPoints.cols(), -1);
            distances.setConstant(knn, queryPoints.cols(), -1);

            #pragma omp parallel for num_threads(threads_)
            for(Index i = 0; i < queryPoints.cols(); ++i)
            {
                Index *idxPoint = &indices.data()[i * knn];
                Scalar *distPoint = &distances.data()[i * knn];

                QueryHeap<Scalar> heap(idxPoint, distPoint, knn);

                for(Index j = 0; j < dataPoints.cols(); ++j)
                {
                    Scalar dist = distance_(queryPoints.col(i), dataPoints.col(j));

                    // check if point is in range if max distance was set
                    bool isInRange = maxDist_ <= 0 || dist <= maxDist_;
                    // check if this node was an improvement if heap is already full
                    bool isImprovement = !heap.full() ||
                        dist < heap.front();
                    if(isInRange && isImprovement)
                    {
                        if(heap.full())
                            heap.pop();
                        heap.push(j, dist);
                    }
                }

                if(sorted_)
                    heap.sort();

                if(takeRoot_)
                {
                    for(size_t j = 0; j < knn; ++j)
                    {
                        if(idxPoint[j] < 0)
                            break;
                        distPoint[j] = distance_(distPoint[j]);
                    }
                }
            }
        }

        /** Returns the amount of data points stored in the search index.
          * @return number of data points */
        Index size() const
        {
            return data_ == nullptr ? 0 : data_->cols();
        }

        /** Returns the dimension of the data points in the search index.
          * @return dimension of data points */
        Index dimension() const
        {
            return data_ == nullptr ? 0 : data_->rows();
        }
    };

    // template<typename Scalar>
    // struct MeanMidpointRule
    // {
    //     typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    //     typedef knncpp::Matrixi Matrixi;

    //     void operator(const Matrix &data, const Matrixi &indices, Index split)
    // };

    /** Class for performing k nearest neighbour searches with minkowski distances.
      * This kdtree only works reliably with the minkowski distance and its
      * special cases like manhatten or euclidean distance.
      * @see ManhattenDistance, EuclideanDistance, ChebyshevDistance, MinkowskiDistance*/
    template<typename _Scalar, int _Dimension, typename _Distance>
    class KDTreeMinkowski
    {
    public:
        typedef _Scalar Scalar;
        typedef _Distance Distance;
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
        typedef Eigen::Matrix<Scalar, _Dimension, Eigen::Dynamic> DataMatrix;
        typedef Eigen::Matrix<Scalar, _Dimension, 1> DataVector;
        typedef knncpp::Matrixi Matrixi;
    private:
        typedef Eigen::Matrix<Scalar, 2, 1> Bounds;
        typedef Eigen::Matrix<Scalar, 2, _Dimension> BoundingBox;

        /** Struct representing a node in the KDTree.
          * It can be either a inner node or a leaf node. */
        struct Node
        {
            /** Indices of data points in this leaf node. */
            Index startIdx = 0;
            Index length = 0;

            /** Left child of this inner node. */
            Index left = -1;
            /** Right child of this inner node. */
            Index right = -1;
            /** Axis of the axis aligned splitting hyper plane. */
            Index splitaxis = -1;
            /** Translation of the axis aligned splitting hyper plane. */
            Scalar splitpoint = 0;
            /** Lower end of the splitpoint range */
            Scalar splitlower = 0;
            /** Upper end of the splitpoint range */
            Scalar splitupper = 0;


            Node() = default;

            /** Constructor for leaf nodes */
            Node(const Index startIdx, const Index length)
                : startIdx(startIdx), length(length)
            { }

            /** Constructor for inner nodes */
            Node(const Index splitaxis, const Scalar splitpoint,
                const Index left, const Index right)
                : left(left), right(right),
                splitaxis(splitaxis), splitpoint(splitpoint)
            { }

            bool isLeaf() const
            {
                return !hasLeft() && !hasRight();
            }

            bool isInner() const
            {
                return hasLeft() && hasRight();
            }

            bool hasLeft() const
            {
                return left >= 0;
            }

            bool hasRight() const
            {
                return right >= 0;
            }
        };

        DataMatrix dataCopy_ = DataMatrix();
        const DataMatrix *data_ = nullptr;
        std::vector<Index> indices_ = std::vector<Index>();
        std::vector<Node> nodes_ = std::vector<Node>();

        Index bucketSize_ = 16;
        bool sorted_ = true;
        bool compact_ = false;
        bool balanced_ = false;
        bool takeRoot_ = true;
        Index threads_ = 0;
        Scalar maxDist_ = 0;

        Distance distance_ = Distance();

        BoundingBox bbox_ = BoundingBox();

        Index buildLeafNode(const Index startIdx,
            const Index length,
            BoundingBox &bbox)
        {
            nodes_.push_back(Node(startIdx, length));
            calculateBoundingBox(startIdx, length, bbox);
            return static_cast<Index>(nodes_.size() - 1);
        }

        /** Finds the minimum and maximum values of each dimension (row) in the
          * data matrix. Only respects the columns specified by the index
          * vector.
          * @param startIdx starting index within indices data structure to search for bounding box
          * @param length length of the block of indices*/
        void calculateBoundingBox(const Index startIdx,
            const Index length,
            BoundingBox &bbox) const
        {
            assert(length > 0);
            assert(startIdx >= 0);
            assert(static_cast<size_t>(startIdx + length) <= indices_.size());
            assert(data_->rows() == bbox.cols());

            const DataMatrix &data = *data_;

            // initialize bounds of the bounding box
            Index first = indices_[startIdx];
            for(Index i = 0; i < bbox.cols(); ++i)
            {
                bbox(0, i) = data(i, first);
                bbox(1, i) = data(i, first);
            }

            // search for min / max values in data
            for(Index i = 1; i < length; ++i)
            {
                // retrieve data index
                Index col = indices_[startIdx + i];
                assert(col >= 0 && col < data.cols());

                // check min and max for each dimension individually
                for(Index j = 0; j < data.rows(); ++j)
                {
                    bbox(0, j) = std::min(bbox(0, j), data(j, col));
                    bbox(1, j) = std::max(bbox(1, j), data(j, col));
                }
            }
        }

        /** Calculates the bounds (min / max values) for the given dimension and block of data. */
        void calculateBounds(const Index startIdx,
            const Index length,
            const Index dim,
            Bounds &bounds) const
        {
            assert(length > 0);
            assert(startIdx >= 0);
            assert(static_cast<size_t>(startIdx + length) <= indices_.size());

            const DataMatrix &data = *data_;

            bounds(0) = data(dim, indices_[startIdx]);
            bounds(1) = data(dim, indices_[startIdx]);

            for(Index i = 1; i < length; ++i)
            {
                Index col = indices_[startIdx + i];
                assert(col >= 0 && col < data.cols());

                bounds(0) = std::min(bounds(0), data(dim, col));
                bounds(1) = std::max(bounds(1), data(dim, col));
            }
        }

        void calculateSplittingMidpoint(const Index startIdx,
            const Index length,
            const BoundingBox &bbox,
            Index &splitaxis,
            Scalar &splitpoint,
            Index &splitoffset)
        {
            const DataMatrix &data = *data_;

            // search for axis with longest distance
            splitaxis = 0;
            Scalar splitsize = static_cast<Scalar>(0);
            for(Index i = 0; i < data.rows(); ++i)
            {
                Scalar diff = bbox(1, i) - bbox(0, i);
                if(diff > splitsize)
                {
                    splitaxis = i;
                    splitsize = diff;
                }
            }

            // calculate the bounds in this axis and update our data
            // accordingly
            Bounds bounds;
            calculateBounds(startIdx, length, splitaxis, bounds);
            splitsize = bounds(1) - bounds(0);

            const Index origSplitaxis = splitaxis;
            for(Index i = 0; i < data.rows(); ++i)
            {
                // skip the dimension of the previously found splitaxis
                if(i == origSplitaxis)
                    continue;
                Scalar diff = bbox(1, i) - bbox(0, i);
                // check if the split for this dimension would be potentially larger
                if(diff > splitsize)
                {
                    Bounds newBounds;
                    // update the bounds to their actual current value
                    calculateBounds(startIdx, length, splitaxis, newBounds);
                    diff = newBounds(1) - newBounds(0);
                    if(diff > splitsize)
                    {
                        splitaxis = i;
                        splitsize = diff;
                        bounds = newBounds;
                    }
                }
            }

            // use the sliding midpoint rule
            splitpoint = (bounds(0) + bounds(1)) / static_cast<Scalar>(2);

            Index leftIdx = startIdx;
            Index rightIdx = startIdx + length - 1;

            // first loop checks left < splitpoint and right >= splitpoint
            while(leftIdx <= rightIdx)
            {
                // increment left as long as left has not reached right and
                // the value of the left element is less than the splitpoint
                while(leftIdx <= rightIdx && data(splitaxis, indices_[leftIdx]) < splitpoint)
                    ++leftIdx;

                // decrement right as long as left has not reached right and
                // the value of the right element is greater than the splitpoint
                while(leftIdx <= rightIdx && data(splitaxis, indices_[rightIdx]) >= splitpoint)
                    --rightIdx;

                if(leftIdx <= rightIdx)
                {
                    std::swap(indices_[leftIdx], indices_[rightIdx]);
                    ++leftIdx;
                    --rightIdx;
                }
            }

            // remember this offset from starting index
            const Index offset1 = leftIdx - startIdx;

            rightIdx = startIdx + length - 1;
            // second loop checks left <= splitpoint and right > splitpoint
            while(leftIdx <= rightIdx)
            {
                // increment left as long as left has not reached right and
                // the value of the left element is less than the splitpoint
                while(leftIdx <= rightIdx && data(splitaxis, indices_[leftIdx]) <= splitpoint)
                    ++leftIdx;

                // decrement right as long as left has not reached right and
                // the value of the right element is greater than the splitpoint
                while(leftIdx <= rightIdx && data(splitaxis, indices_[rightIdx]) > splitpoint)
                    --rightIdx;

                if(leftIdx <= rightIdx)
                {
                    std::swap(indices_[leftIdx], indices_[rightIdx]);
                    ++leftIdx;
                    --rightIdx;
                }
            }

            // remember this offset from starting index
            const Index offset2 = leftIdx - startIdx;

            const Index halfLength = length / static_cast<Index>(2);

            // find a separation of points such that is best balanced
            // offset1 denotes separation where equal points are all on the right
            // offset2 denots separation where equal points are all on the left
            if (offset1 > halfLength)
                splitoffset = offset1;
            else if (offset2 < halfLength)
                splitoffset = offset2;
            // when we get here offset1 < halflength and offset2 > halflength
            // so simply split the equal elements in the middle
            else
                splitoffset = halfLength;
        }

        Index buildInnerNode(const Index startIdx,
            const Index length,
            BoundingBox &bbox)
        {
            assert(length > 0);
            assert(startIdx >= 0);
            assert(static_cast<size_t>(startIdx  + length) <= indices_.size());
            assert(data_->rows() == bbox.cols());

            // create node
            const Index nodeIdx = nodes_.size();
            nodes_.push_back(Node());

            Index splitaxis;
            Index splitoffset;
            Scalar splitpoint;
            calculateSplittingMidpoint(startIdx, length, bbox, splitaxis, splitpoint, splitoffset);

            nodes_[nodeIdx].splitaxis = splitaxis;
            nodes_[nodeIdx].splitpoint = splitpoint;

            const Index leftStart = startIdx;
            const Index leftLength = splitoffset;
            const Index rightStart = startIdx + splitoffset;
            const Index rightLength = length - splitoffset;

            BoundingBox bboxLeft = bbox;
            BoundingBox bboxRight = bbox;

            // do left build
            bboxLeft(1, splitaxis) = splitpoint;
            Index left = buildR(leftStart, leftLength, bboxLeft);
            nodes_[nodeIdx].left =  left;

            // do right build
            bboxRight(0, splitaxis) = splitpoint;
            Index right = buildR(rightStart, rightLength, bboxRight);
            nodes_[nodeIdx].right = right;

            // extract the range of the splitpoint
            nodes_[nodeIdx].splitlower = bboxLeft(1, splitaxis);
            nodes_[nodeIdx].splitupper = bboxRight(0, splitaxis);

            // update the bounding box to the values of the new bounding boxes
            for(Index i = 0; i < bbox.cols(); ++i)
            {
                bbox(0, i) = std::min(bboxLeft(0, i), bboxRight(0, i));
                bbox(1, i) = std::max(bboxLeft(1, i), bboxRight(1, i));
            }

            return nodeIdx;
        }

        Index buildR(const Index startIdx,
            const Index length,
            BoundingBox &bbox)
        {
            // check for base case
            if(length <= bucketSize_)
                return buildLeafNode(startIdx, length, bbox);
            else
                return buildInnerNode(startIdx, length, bbox);
        }

        bool isDistanceInRange(const Scalar dist) const
        {
            return maxDist_ <= 0 || dist <= maxDist_;
        }

        bool isDistanceImprovement(const Scalar dist, const QueryHeap<Scalar> &dataHeap) const
        {
            return !dataHeap.full() || dist < dataHeap.front();
        }

        template<typename Derived>
        void queryLeafNode(const Node &node,
            const Eigen::MatrixBase<Derived> &queryPoint,
            QueryHeap<Scalar> &dataHeap) const
        {
            assert(node.isLeaf());

            const DataMatrix &data = *data_;

            // go through all points in this leaf node and do brute force search
            for(Index i = 0; i < node.length; ++i)
            {
                const Index idx = node.startIdx + i;
                assert(idx >= 0 && idx < static_cast<Index>(indices_.size()));

                // retrieve index of the current data point
                const Index dataIdx = indices_[idx];
                const Scalar dist = distance_(queryPoint, data.col(dataIdx));

                // check if point is within max distance and if the value would be
                // an improvement
                if(isDistanceInRange(dist) && isDistanceImprovement(dist, dataHeap))
                {
                    if(dataHeap.full())
                        dataHeap.pop();
                    dataHeap.push(dataIdx, dist);
                }
            }
        }

        template<typename Derived>
        void queryInnerNode(const Node &node,
            const Eigen::MatrixBase<Derived> &queryPoint,
            QueryHeap<Scalar> &dataHeap,
            DataVector &splitdists,
            const Scalar mindist) const
        {
            assert(node.isInner());

            const Index splitaxis = node.splitaxis;
            const Scalar splitval = queryPoint(splitaxis, 0);
            Scalar splitdist;
            Index firstNode;
            Index secondNode;
            // check if right or left child should be visited
            const bool visitLeft = (splitval - node.splitlower + splitval - node.splitupper) < 0;
            if(visitLeft)
            {
                firstNode = node.left;
                secondNode = node.right;
                splitdist = distance_(splitval, node.splitupper);
            }
            else
            {
                firstNode = node.right;
                secondNode = node.left;
                splitdist = distance_(splitval, node.splitlower);
            }

            queryR(nodes_[firstNode], queryPoint, dataHeap, splitdists, mindist);

            const Scalar mindistNew = mindist + splitdist - splitdists(splitaxis);

            // check if node is in range if max distance was set
            // check if this node was an improvement if heap is already full
            if(isDistanceInRange(mindistNew) && isDistanceImprovement(mindistNew, dataHeap))
            {
                const Scalar splitdistOld = splitdists(splitaxis);
                splitdists(splitaxis) = splitdist;
                queryR(nodes_[secondNode], queryPoint, dataHeap, splitdists, mindistNew);
                splitdists(splitaxis) = splitdistOld;
            }
        }

        template<typename Derived>
        void queryR(const Node &node,
            const Eigen::MatrixBase<Derived> &queryPoint,
            QueryHeap<Scalar> &dataHeap,
            DataVector &splitdists,
            const Scalar mindist) const
        {
            if(node.isLeaf())
                queryLeafNode(node, queryPoint, dataHeap);
            else
                queryInnerNode(node, queryPoint, dataHeap, splitdists, mindist);
        }

        /** Recursively computes the depth for the given node. */
        Index depthR(const Node &node) const
        {
            if(node.isLeaf())
                return 1;
            else
            {
                Index left = depthR(nodes_[node.left]);
                Index right = depthR(nodes_[node.right]);
                return std::max(left, right) + 1;
            }
        }

    public:

        /** Constructs an empty KDTree. */
        KDTreeMinkowski()
        { }

        /** Constructs KDTree with the given data. This does not build the
          * the index of the tree.
          * @param data NxM matrix, M points of dimension N
          * @param copy if true copies the data, otherwise assumes static data */
        KDTreeMinkowski(const DataMatrix &data, const bool copy=false)
        {
            setData(data, copy);
        }

        /** Set the maximum amount of data points per leaf in the tree (aka
          * bucket size).
          * @param bucketSize amount of points per leaf. */
        void setBucketSize(const Index bucketSize)
        {
            bucketSize_ = bucketSize;
        }

        /** Set if the points returned by the queries should be sorted
          * according to their distance to the query points.
          * @param sorted sort query results */
        void setSorted(const bool sorted)
        {
            sorted_ = sorted;
        }

        /** Set if the tree should be built as balanced as possible.
          * This increases build time, but decreases search time.
          * @param balanced set true to build a balanced tree */
        void setBalanced(const bool balanced)
        {
            balanced_ = balanced;
        }

        /** Set if the distances after the query should be rooted or not.
          * Taking the root of the distances increases query time, but the
          * function will return true distances instead of their powered
          * versions.
          * @param takeRoot set true if root should be taken else false */
        void setTakeRoot(const bool takeRoot)
        {
            takeRoot_ = takeRoot;
        }

        /** Set if the tree should be built with compact leaf nodes.
          * This increases build time, but makes leaf nodes denser (more)
          * points. Thus less visits are necessary.
          * @param compact set true ti build a tree with compact leafs */
        void setCompact(const bool compact)
        {
            compact_ = compact;
        }

        /** Set the amount of threads that should be used for building and
          * querying the tree.
          * OpenMP has to be enabled for this to work.
          * @param threads amount of threads, 0 for optimal choice */
        void setThreads(const unsigned int threads)
        {
            threads_ = threads;
        }

        /** Set the maximum distance for querying the tree.
          * The search will be pruned if the maximum distance is set to any
          * positive number.
          * @param maxDist maximum distance, <= 0 for no limit */
        void setMaxDistance(const Scalar maxDist)
        {
            maxDist_ = maxDist;
        }

        /** Set the data points used for this tree.
          * This does not build the tree.
          * @param data NxM matrix, M points of dimension N
          * @param copy if true data is copied, assumes static data otherwise */
        void setData(const DataMatrix &data, const bool copy = false)
        {
            clear();
            if(copy)
            {
                dataCopy_ = data;
                data_ = &dataCopy_;
            }
            else
            {
                data_ = &data;
            }
        }

        void setDistance(const Distance &distance)
        {
            distance_ = distance;
        }

        /** Builds the search index of the tree.
          * Data has to be set and must be non-empty. */
        void build()
        {
            if(data_ == nullptr)
                throw std::runtime_error("cannot build KDTree; data not set");

            if(data_->size() == 0)
                throw std::runtime_error("cannot build KDTree; data is empty");

            clear();
            nodes_.reserve((data_->cols() / bucketSize_) + 1);

            // initialize indices in simple sequence
            indices_.resize(data_->cols());
            for(size_t i = 0; i < indices_.size(); ++i)
                indices_[i] = i;

            bbox_.resize(2, data_->rows());
            Index startIdx = 0;
            Index length = data_->cols();

            calculateBoundingBox(startIdx, length, bbox_);

            buildR(startIdx, length, bbox_);
        }

        /** Queries the tree for the nearest neighbours of the given query
          * points.
          *
          * The tree has to be built before it can be queried.
          *
          * The query points have to have the same dimension as the data points
          * of the tree.
          *
          * The result matrices will be resized appropriatley.
          * Indices and distances will be set to -1 if less than knn neighbours
          * were found.
          *
          * @param queryPoints NxM matrix, M points of dimension N
          * @param knn amount of neighbours to be found
          * @param indices KNNxM matrix, indices of neighbours in the data set
          * @param distances KNNxM matrix, distance between query points and
          *        neighbours */
        template<typename Derived>
        void query(const Eigen::MatrixBase<Derived> &queryPoints,
            const size_t knn,
            Matrixi &indices,
            Matrix &distances) const
        {
            if(nodes_.size() == 0)
                throw std::runtime_error("cannot query KDTree; not built yet");

            if(queryPoints.rows() != dimension())
                throw std::runtime_error("cannot query KDTree; data and query points do not have same dimension");

            distances.setConstant(knn, queryPoints.cols(), -1);
            indices.setConstant(knn, queryPoints.cols(), -1);

            Index *indicesRaw = indices.data();
            Scalar *distsRaw = distances.data();

            #pragma omp parallel for num_threads(threads_)
            for(Index i = 0; i < queryPoints.cols(); ++i)
            {

                Scalar *distPoint = &distsRaw[i * knn];
                Index *idxPoint = &indicesRaw[i * knn];

                // create heap to find nearest neighbours
                QueryHeap<Scalar> dataHeap(idxPoint, distPoint, knn);

                Scalar mindist = static_cast<Scalar>(0);
                DataVector splitdists(queryPoints.rows());

                for(Index j = 0; j < splitdists.rows(); ++j)
                {
                    const Scalar value = queryPoints(j, i);
                    const Scalar lower = bbox_(0, j);
                    const Scalar upper = bbox_(1, j);
                    if(value < lower)
                    {
                        splitdists(j) = distance_(value, lower);
                    }
                    else if(value > upper)
                    {
                        splitdists(j) = distance_(value, upper);
                    }
                    else
                    {
                        splitdists(j) = static_cast<Scalar>(0);
                    }

                    mindist += splitdists(j);
                }

                queryR(nodes_[0], queryPoints.col(i), dataHeap, splitdists, mindist);

                if(sorted_)
                    dataHeap.sort();

                if(takeRoot_)
                {
                    for(size_t j = 0; j < knn; ++j)
                    {
                        if(distPoint[j] < 0)
                            break;
                        distPoint[j] = distance_(distPoint[j]);
                    }
                }
            }
        }

        /** Clears the tree. */
        void clear()
        {
            nodes_.clear();
        }

        /** Returns the amount of data points stored in the search index.
          * @return number of data points */
        Index size() const
        {
            return data_ == nullptr ? 0 : data_->cols();
        }

        /** Returns the dimension of the data points in the search index.
          * @return dimension of data points */
        Index dimension() const
        {
            return data_ == nullptr ? 0 : data_->rows();
        }

        /** Returns the maxximum depth of the tree.
          * @return maximum depth of the tree */
        Index depth() const
        {
            return nodes_.size() == 0 ? 0 : depthR(nodes_.front());
        }
    };

    template<typename _Scalar, typename _Distance = EuclideanDistance<_Scalar>> using KDTreeMinkowski2 = KDTreeMinkowski<_Scalar, 2, _Distance>;
    template<typename _Scalar, typename _Distance = EuclideanDistance<_Scalar>> using KDTreeMinkowski3 = KDTreeMinkowski<_Scalar, 3, _Distance>;
    template<typename _Scalar, typename _Distance = EuclideanDistance<_Scalar>> using KDTreeMinkowski4 = KDTreeMinkowski<_Scalar, 4, _Distance>;
    template<typename _Scalar, typename _Distance = EuclideanDistance<_Scalar>> using KDTreeMinkowski5 = KDTreeMinkowski<_Scalar, 5, _Distance>;
    template<typename _Scalar, typename _Distance = EuclideanDistance<_Scalar>> using KDTreeMinkowskiX = KDTreeMinkowski<_Scalar, Eigen::Dynamic, _Distance>;

    /** Class for performing KNN search in hamming space by multi-index hashing. */
    template<typename Scalar>
    class MultiIndexHashing
    {
    public:
        static_assert(std::is_integral<Scalar>::value, "MultiIndexHashing Scalar has to be integral");

        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
        typedef knncpp::Matrixi Matrixi;

    private:
        HammingDistance<Scalar> distance_;

        Matrix dataCopy_;
        const Matrix *data_;

        bool sorted_;
        Scalar maxDist_;
        Index substrLen_;
        Index threads_;
        std::vector<std::map<Scalar, std::vector<Index>>> buckets_;

        template<typename Derived>
        Scalar extractCode(const Eigen::MatrixBase<Derived> &data,
            const Index idx,
            const Index offset) const
        {
            Index leftShift = std::max<Index>(0, static_cast<Index>(sizeof(Scalar)) - offset - substrLen_);
            Index rightShift = leftShift + offset;

            Scalar code = (data(idx, 0) << (leftShift * 8)) >> (rightShift * 8);

            if(static_cast<Index>(sizeof(Scalar)) - offset < substrLen_ && idx + 1 < data.rows())
            {
                Index shift = 2 * static_cast<Index>(sizeof(Scalar)) - substrLen_ - offset;
                code |= data(idx+1, 0) << (shift * 8);
            }

            return code;
        }
    public:
        MultiIndexHashing()
            : distance_(), dataCopy_(), data_(nullptr), sorted_(true),
            maxDist_(0), substrLen_(1), threads_(1)
        { }

        /** Constructs an index with the given data.
          * This does not build the the index.
          * @param data NxM matrix, M points of dimension N
          * @param copy if true copies the data, otherwise assumes static data */
        MultiIndexHashing(const Matrix &data, const bool copy=false)
            : MultiIndexHashing()
        {
            setData(data, copy);
        }

        /** Set the maximum distance for querying the index.
          * Note that if no maximum distance is used, this algorithm performs
          * basically a brute force search.
          * @param maxDist maximum distance, <= 0 for no limit */
        void setMaxDistance(const Scalar maxDist)
        {
            maxDist_ = maxDist;
        }

        /** Set if the points returned by the queries should be sorted
          * according to their distance to the query points.
          * @param sorted sort query results */
        void setSorted(const bool sorted)
        {
            sorted_ = sorted;
        }

        /** Set the amount of threads that should be used for building and
          * querying the tree.
          * OpenMP has to be enabled for this to work.
          * @param threads amount of threads, 0 for optimal choice */
        void setThreads(const unsigned int threads)
        {
            threads_ = threads;
        }

        /** Set the length of substrings (in bytes) used for multi index hashing.
          * @param len lentth of bucket substrings in bytes*/
        void setSubstringLength(const Index len)
        {
            substrLen_ = len;
        }

        /** Set the data points used for the KNN search.
          * @param data NxM matrix, M points of dimension N
          * @param copy if true data is copied, assumes static data otherwise */
        void setData(const Matrix &data, const bool copy = false)
        {
            clear();
            if(copy)
            {
                dataCopy_ = data;
                data_ = &dataCopy_;
            }
            else
            {
                data_ = &data;
            }
        }

        void build()
        {
            if(data_ == nullptr)
                throw std::runtime_error("cannot build MultiIndexHashing; data not set");
            if(data_->size() == 0)
                throw std::runtime_error("cannot build MultiIndexHashing; data is empty");

            const Matrix &data = *data_;
            const Index bytesPerVec = data.rows() * static_cast<Index>(sizeof(Scalar));
            if(bytesPerVec % substrLen_ != 0)
                throw std::runtime_error("cannot build MultiIndexHashing; cannot divide byte count per vector by substring length without remainings");

            buckets_.clear();
            buckets_.resize(bytesPerVec / substrLen_);

            for(size_t i = 0; i < buckets_.size(); ++i)
            {
                Index start = static_cast<Index>(i) * substrLen_;
                Index idx = start / static_cast<Index>(sizeof(Scalar));
                Index offset = start % static_cast<Index>(sizeof(Scalar));
                std::map<Scalar, std::vector<Index>> &map = buckets_[i];

                for(Index c = 0; c < data.cols(); ++c)
                {
                    Scalar code = extractCode(data.col(c), idx, offset);
                    if(map.find(code) == map.end())
                        map[code] = std::vector<Index>();
                    map[code].push_back(c);
                }
            }
        }

        template<typename Derived>
        void query(const Eigen::MatrixBase<Derived> &queryPoints,
            const size_t knn,
            Matrixi &indices,
            Matrix &distances) const
        {
            if(buckets_.size() == 0)
                throw std::runtime_error("cannot query MultiIndexHashing; not built yet");
            if(queryPoints.rows() != dimension())
                throw std::runtime_error("cannot query MultiIndexHashing; data and query points do not have same dimension");

            const Matrix &data = *data_;

            indices.setConstant(knn, queryPoints.cols(), -1);
            distances.setConstant(knn, queryPoints.cols(), -1);

            Index *indicesRaw = indices.data();
            Scalar *distsRaw = distances.data();

            Scalar maxDistPart = maxDist_ / buckets_.size();

            #pragma omp parallel for num_threads(threads_)
            for(Index c = 0; c < queryPoints.cols(); ++c)
            {
                std::set<Index> candidates;
                for(size_t i = 0; i < buckets_.size(); ++i)
                {
                    Index start = static_cast<Index>(i) * substrLen_;
                    Index idx = start / static_cast<Index>(sizeof(Scalar));
                    Index offset = start % static_cast<Index>(sizeof(Scalar));
                    const std::map<Scalar, std::vector<Index>> &map = buckets_[i];

                    Scalar code = extractCode(queryPoints.col(c), idx, offset);
                    for(const auto &x: map)
                    {
                        Scalar dist = distance_(x.first, code);
                        if(maxDistPart <= 0 || dist <= maxDistPart)
                        {
                            for(size_t j = 0; j < x.second.size(); ++j)
                                candidates.insert(x.second[j]);
                        }
                    }
                }

                Scalar *distPoint = &distsRaw[c * knn];
                Index *idxPoint = &indicesRaw[c * knn];
                // create heap to find nearest neighbours
                QueryHeap<Scalar> dataHeap(idxPoint, distPoint, knn);

                for(Index idx: candidates)
                {
                    Scalar dist = distance_(data.col(idx), queryPoints.col(c));

                    bool isInRange = maxDist_ <= 0 || dist <= maxDist_;
                    bool isImprovement = !dataHeap.full() ||
                        dist < dataHeap.front();
                    if(isInRange && isImprovement)
                    {
                        if(dataHeap.full())
                            dataHeap.pop();
                        dataHeap.push(idx, dist);
                    }
                }

                if(sorted_)
                    dataHeap.sort();
            }
        }

        /** Returns the amount of data points stored in the search index.
          * @return number of data points */
        Index size() const
        {
            return data_ == nullptr ? 0 : data_->cols();
        }

        /** Returns the dimension of the data points in the search index.
          * @return dimension of data points */
        Index dimension() const
        {
            return data_ == nullptr ? 0 : data_->rows();
        }

        void clear()
        {
            data_ = nullptr;
            dataCopy_.resize(0, 0);
            buckets_.clear();
        }

    };

    #ifdef KNNCPP_FLANN

    /** Wrapper class of FLANN kdtrees for the use with Eigen3. */
    template<typename Scalar,
        typename Distance=flann::L2_Simple<Scalar>>
    class KDTreeFlann
    {
    public:
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
        typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> Matrixi;

    private:
        typedef flann::Index<Distance> FlannIndex;

        Matrix dataCopy_;
        Matrix *dataPoints_;

        FlannIndex *index_;
        flann::SearchParams searchParams_;
        flann::IndexParams indexParams_;
        Scalar maxDist_;

    public:
        KDTreeFlann()
            : dataCopy_(), dataPoints_(nullptr), index_(nullptr),
            searchParams_(32, 0, false),
            indexParams_(flann::KDTreeSingleIndexParams(15)),
            maxDist_(0)
        {
        }

        KDTreeFlann(Matrix &data, const bool copy = false)
            : KDTreeFlann()
        {
            setData(data, copy);
        }

        ~KDTreeFlann()
        {
            clear();
        }

        void setIndexParams(const flann::IndexParams &params)
        {
            indexParams_ = params;
        }

        void setChecks(const int checks)
        {
            searchParams_.checks = checks;
        }

        void setSorted(const bool sorted)
        {
            searchParams_.sorted = sorted;
        }

        void setThreads(const int threads)
        {
            searchParams_.cores = threads;
        }

        void setEpsilon(const float eps)
        {
            searchParams_.eps = eps;
        }

        void setMaxDistance(const Scalar dist)
        {
            maxDist_ = dist;
        }

        void setData(Matrix &data, const bool copy = false)
        {
            if(copy)
            {
                dataCopy_ = data;
                dataPoints_ = &dataCopy_;
            }
            else
            {
                dataPoints_ = &data;
            }

            clear();
        }

        void build()
        {
            if(dataPoints_ == nullptr)
                throw std::runtime_error("cannot build KDTree; data not set");
            if(dataPoints_->size() == 0)
                throw std::runtime_error("cannot build KDTree; data is empty");

            if(index_ != nullptr)
                delete index_;

            flann::Matrix<Scalar> dataPts(
                dataPoints_->data(),
                dataPoints_->cols(),
                dataPoints_->rows());

            index_ = new FlannIndex(dataPts, indexParams_);
            index_->buildIndex();
        }

        void query(Matrix &queryPoints,
            const size_t knn,
            Matrixi &indices,
            Matrix &distances) const
        {
            if(index_ == nullptr)
                throw std::runtime_error("cannot query KDTree; not built yet");
            if(dataPoints_->rows() != queryPoints.rows())
                throw std::runtime_error("cannot query KDTree; KDTree has different dimension than query data");

            // resize result matrices
            distances.resize(knn, queryPoints.cols());
            indices.resize(knn, queryPoints.cols());

            // wrap matrices into flann matrices
            flann::Matrix<Scalar> queryPts(
                queryPoints.data(),
                queryPoints.cols(),
                queryPoints.rows());
            flann::Matrix<int> indicesF(
                indices.data(),
                indices.cols(),
                indices.rows());
            flann::Matrix<Scalar> distancesF(
                distances.data(),
                distances.cols(),
                distances.rows());

            // if maximum distance was set then use radius search
            if(maxDist_ > 0)
                index_->radiusSearch(queryPts, indicesF, distancesF, maxDist_, searchParams_);
            else
                index_->knnSearch(queryPts, indicesF, distancesF, knn, searchParams_);

            // make result matrices compatible to API
            #pragma omp parallel for num_threads(searchParams_.cores)
            for(Index i = 0; i < indices.cols(); ++i)
            {
                bool found = false;
                for(Index j = 0; j < indices.rows(); ++j)
                {
                    if(indices(j, i) == -1)
                        found = true;

                    if(found)
                    {
                        indices(j, i) = -1;
                        distances(j, i) = -1;
                    }
                }
            }
        }

        Index size() const
        {
            return dataPoints_ == nullptr ? 0 : dataPoints_->cols();
        }

        Index dimension() const
        {
            return dataPoints_ == nullptr ? 0 : dataPoints_->rows();
        }

        void clear()
        {
            if(index_ != nullptr)
            {
                delete index_;
                index_ = nullptr;
            }
        }

        FlannIndex &flannIndex()
        {
            return index_;
        }
    };

    typedef KDTreeFlann<double> KDTreeFlannd;
    typedef KDTreeFlann<float> KDTreeFlannf;

    #endif
}

#endif