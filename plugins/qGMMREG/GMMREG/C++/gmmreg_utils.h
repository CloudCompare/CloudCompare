#ifndef GMMREG_UTILS_H_
#define GMMREG_UTILS_H_

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

namespace gmmreg {

	template<typename T>
	T GaussTransform(const vnl_matrix<T>& A,
		const vnl_matrix<T>& B,
		T scale);

	template<typename T>
	T GaussTransform(const vnl_matrix<T>& A,
		const vnl_matrix<T>& B,
		T scale,
		vnl_matrix<T>& gradient);

	template<typename T>
	//T GaussTransform(const vnl_matrix<T>& A, const vnl_matrix<T>& B, T scale, T* gradient);
	//T GaussTransform(const T* A, const T* B, int m, int n, int dim, T scale, T* grad);
	void ComputeTPSKernel(const vnl_matrix<T>& model,
		const vnl_matrix<T>& ctrl_pts,
		vnl_matrix<T>& U,
		vnl_matrix<T>& K);

	template<typename T>
	void ComputeGaussianKernel(const vnl_matrix<T>& model,
		const vnl_matrix<T>& ctrl_pts,
		vnl_matrix<T>& G,
		vnl_matrix<T>& K,
		T beta);

	template<typename T>
	void ComputeSquaredDistanceMatrix(const vnl_matrix<T>& A,
		const vnl_matrix<T>& B,
		vnl_matrix<T>& D);

	//void normalize(const vnl_matrix<T>& x, vnl_vector<T>& centroid, T& scale, vnl_matrix<T>& normalized_x);
	//void denormalize(const vnl_matrix<T>& x, const vnl_vector<T>& centroid, const T scale, vnl_matrix<T>& denormalized_x);
	template<typename T>
	void Normalize(vnl_matrix<T>& x,
		vnl_vector<T>& centroid,
		T& scale);

	template<typename T>
	void Denormalize(vnl_matrix<T>& x,
		const vnl_vector<T>& centroid,
		const T scale);

	template<typename T>
	void ComputeP(const vnl_matrix<T>& x,
		const vnl_matrix<T>& y,
		vnl_matrix<T>& P, T &E, T sigma, int outliers);

#define SQR(X)  ((X)*(X))

	/*
	*  Note: The input point set containing 'n' points in 'd'-dimensional
	*  space should be arranged in the memory such that the j-th coordinate of i-th point
	*  is at location  (i*d + j), i.e., the input matrix in MATLAB should
	*  be of size (d, n) since MATLAB is column-major while the input array
	*  in NumPy should be of size (n, d) since NumPy is row-major.
	*/
	template<typename T>
	T GaussTransform(const T* A, const T* B,
		int m, int n, int dim, T scale) {
			T cross_term = 0;
			scale = SQR(scale);
			for (int i = 0; i < m; ++i) {
				for (int j = 0; j < n; ++j) {
					T dist_ij = 0;
					for (int d = 0; d < dim; ++d) {
						dist_ij += SQR(A[i * dim + d] - B[j * dim + d]);
					}
					T cost_ij = exp(-1.0 * dist_ij / scale);
					cross_term += cost_ij;
				}
				/* printf("cross_term = %.3f\n", cross_term);  */
			}
			return cross_term / (m * n);
	}

	template<typename T>
	void GaussianAffinityMatrix(const T* A, const T* B,
		int m, int n, int dim, T scale, T* dist) {
			scale = -2.0*SQR(scale);
			for (int i = 0, k = 0; i < m; ++i) {
				for (int j = 0; j < n; ++j, ++k) {
					T dist_ij = 0;
					for (int d = 0; d < dim; ++d) {
						dist_ij +=  SQR(A[i * dim + d] - B[j * dim + d]);
					}
					dist[k] = exp(dist_ij / scale);
				}
			}
	}

	template<typename T>
	T GaussTransform(const T* A, const T* B,
		int m, int n, int dim, T scale, T* grad) {
			T cross_term = 0;
			for (int i = 0; i < m * dim; ++i) {
				grad[i] = 0;
			}

			scale = SQR(scale);
			for (int i = 0; i < m; ++i) {
				for (int j = 0; j < n; ++j) {
					T dist_ij = 0;
					for (int d = 0; d < dim; ++d) {
						dist_ij +=  SQR(A[i * dim + d] - B[j * dim + d]);
					}
					T cost_ij = exp(-1.0 * dist_ij / scale);
					for (int d = 0; d < dim; ++d){
						grad[i * dim + d] -= cost_ij * 2.0 * (A[i * dim + d] - B[j * dim + d]);
					}
					cross_term += cost_ij;
				}
				/* printf("cross_term = %.3f\n", cross_term);  */
			}
			scale *= m * n;
			for (int i = 0; i < m * dim; ++i) {
				grad[i] /= scale;
			}
			return cross_term / (m * n);
	}

	template<typename T>
	T GaussTransform(const vnl_matrix<T>& A,
		const vnl_matrix<T>& B, T scale) {
			// assert A.cols() == B.cols()
			return GaussTransform(A.data_block(), B.data_block(),
				A.rows(), B.rows(), A.cols(), scale);
	}

	template<typename T>
	T GaussTransform(const vnl_matrix<T>& A,
		const vnl_matrix<T>& B, T scale,
		vnl_matrix<T>& gradient) {
			// assert A.cols() == B.cols()
			return GaussTransform(A.data_block(), B.data_block(),
				A.rows(), B.rows(), A.cols(), scale,
				gradient.data_block());
	}

	// todo: add one more version when the model is same as ctrl_pts
	// reference:  Landmark-based Image Analysis, Karl Rohr, p195
	template<typename T>
	void ComputeTPSKernel(const vnl_matrix<T>& model,
		const vnl_matrix<T>& ctrl_pts,
		vnl_matrix<T>& U, vnl_matrix<T>& K)
	{
		int m = model.rows();
		int n = ctrl_pts.rows();
		int d = ctrl_pts.cols();
		//asssert(model.cols()==d==(2|3));
		K.set_size(n, n);
		K.fill(0);
		U.set_size(m, n);
		U.fill(0);

		if (d == 2)
		{
			T eps = 1e-006;

			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					//U
					vnl_vector<T> v_ij = model.get_row(i) - ctrl_pts.get_row(j);
						T r = v_ij.squared_magnitude();
						if (r > eps)
							U(i, j) = r * log(r) / 2;
				}
			}

			for (int i = 0; i < n; ++i)
			{
				for (int j = i + 1; j < n; ++j)
				{
					//K
					vnl_vector<T> v_ij = ctrl_pts.get_row(i) - ctrl_pts.get_row(j);
					T r = v_ij.squared_magnitude();
					if (r > eps)
					{
						K(i, j) = r * log(r) / 2;
					}
				}
			}
		}
		else if (d == 3)
		{
			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					T r = (model.get_row(i) - ctrl_pts.get_row(j)).two_norm();
					U(i, j) = -r;
				}
			}

			for (int i = 0; i < n; ++i)
			{
				for (int j = i + 1; j < n; ++j)
				{
					T r = (ctrl_pts.get_row(i) - ctrl_pts.get_row(j)).two_norm();
					K(i, j) = -r;
				}
			}
		}
		
		for (int i = 0; i < n; ++i)
		{
			for (int j = 0; j < i; ++j)
			{
				K(i, j) = K(j, i);
			}
		}

	}

	/*
	Matlab code in cpd_G.m:
	k=-2*beta^2;
	[n, d]=size(x);  [m, d]=size(y);

	G=repmat(x,[1 1 m])-permute(repmat(y,[1 1 n]),[3 2 1]);
	G=squeeze(sum(G.^2,2));
	G=G/k;
	G=exp(G);
	*/
	template<typename T>
	void ComputeGaussianKernel(const vnl_matrix<T>& model,
		const vnl_matrix<T>& ctrl_pts,
		vnl_matrix<T>& G, vnl_matrix<T>& K,
		T lambda) {
			int m,n,d;
			m = model.rows();
			n = ctrl_pts.rows();
			d = ctrl_pts.cols();
			//asssert(model.cols()==d);
			//assert(lambda>0);

			G.set_size(m,n);
			GaussianAffinityMatrix(model.data_block(), ctrl_pts.data_block(),
				m, n, d, lambda, G.data_block());

			if (model == ctrl_pts) {
				K = G;
			} else {
				K.set_size(n,n);
				GaussianAffinityMatrix(ctrl_pts.data_block(), ctrl_pts.data_block(),
					n, n, d, lambda, K.data_block());
			}
	}

	template<typename T>
	void ComputeSquaredDistanceMatrix(const vnl_matrix<T>& A,
		const vnl_matrix<T>& B, vnl_matrix<T>& D) {
			int m = A.rows();
			int n = B.rows();
			//asssert(A.cols()==B.cols());
			D.set_size(m,n);
			vnl_vector<T> v_ij;
			for (int i = 0; i < m; ++i) {
				for (int j = 0; j < n; ++j) {
					v_ij = A.get_row(i) - B.get_row(j);
					D(i,j) = v_ij.squared_magnitude();
				}
			}
	}

	template<typename T>
	void Normalize(vnl_matrix<T>& x,
		vnl_vector<T>& centroid, T& scale) {
			int n = x.rows();
			if (n==0) return;
			int d = x.cols();
			centroid.set_size(d);

			vnl_vector<T> col;
			for (int i = 0; i < d; ++i) {
				col = x.get_column(i);
				centroid(i) = col.mean();
			}
			for (int i = 0; i < n; ++i) {
				x.set_row(i, x.get_row(i) - centroid);
			}
			scale = x.frobenius_norm() / sqrt(T(n));
			x = x / scale;
	}

	template<typename T>
	void Denormalize(vnl_matrix<T>& x, const vnl_vector<T>& centroid, const T scale) {
		int n = x.rows();
		if (n==0) return;
		int d = x.cols();
		for (int i = 0; i < n; ++i) {
			x.set_row(i, x.get_row(i) * scale + centroid);
		}
	}

	template<typename T>
	void ComputeP(const vnl_matrix<T>& x,const vnl_matrix<T>& y, vnl_matrix<T>& P, T &E, T sigma, int outliers) {
		T k;
		k = -2*sigma*sigma;

		//P.set_size(m,n); P.fill(0);
		//vnl_vector<T> v_ij;

		vnl_vector<T> column_sum;
		int m = x.rows();
		int s = y.rows();
		int d = x.cols();
		column_sum.set_size(s);
		column_sum.fill(0);
		T outlier_term = outliers*pow((2*sigma*sigma*3.1415926),0.5*d);
		for (int i = 0; i < m; ++i) {
			for (int j = 0; j < s; ++j) {
				T r = 0;
				for (int t = 0; t < d; ++t) {
					r += (x(i,t) - y(j,t))*(x(i,t) - y(j,t));
				}
				P(i,j) = exp(r/k);
				column_sum[j]+=P(i,j);
			}
		}


		if (outliers!=0) {
			for (int i = 0; i < s; ++i)
				column_sum[i] += outlier_term;
		}
		if (column_sum.min_value()>(1e-12)) {
			E = 0;
			for (int i = 0; i < s; ++i) {
				for (int j = 0; j < m; ++j){
					P(j,i) = P(j,i)/column_sum[i];
				}
				E -= log(column_sum[i]);
			}
			//vcl_cerr< < s;
			//vcl_cerr<<P.get_column(10);
		}
		else {
			P.empty();
		}
	}

}  // namespace gmmreg

#endif // GMMREG_UTILS_H_
