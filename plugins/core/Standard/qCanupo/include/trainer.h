//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qCANUPO                       #
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
//#      COPYRIGHT: UEB (UNIVERSITE EUROPEENNE DE BRETAGNE) / CNRS         #
//#                                                                        #
//##########################################################################

/** This file is directly inspired of the suggest_classifier_lda.cpp file in the
original CANUPO project, by N. Brodu and D. Lague.
**/

#ifndef QCANUPO_TRAINER_HEADER
#define QCANUPO_TRAINER_HEADER

//system
#include <vector>

//dlib
#include <dlib/matrix.h>
#include <dlib/svm.h>

class LDATrainer
{
public:

	typedef dlib::matrix<float, 0, 1> sample_type;
	typedef dlib::linear_kernel<sample_type> kernel_type;
	typedef dlib::decision_function<kernel_type> trained_function_type;
	//typedef trained_function_type::mem_manager_type mem_manager_type;

	trained_function_type train(const std::vector<sample_type>& samplesvec, const std::vector<float>& labels) const
	{
		size_t fdim = samplesvec[0].size();
		size_t nsamples = samplesvec.size();

		long ndata_class1 = 0, ndata_class2 = 0;
		for (size_t i = 0; i < nsamples; ++i)
		{
			if (labels[i] > 0)
				++ndata_class1;
			else
				++ndata_class2;
		}

		dlib::matrix<sample_type, 0, 1> samples1, samples2;
		samples1.set_size(ndata_class1);
		samples2.set_size(ndata_class2);
		sample_type mu1; mu1.set_size(fdim);
		sample_type mu2; mu2.set_size(fdim);
		for (size_t i = 0; i < fdim; ++i)
		{
			mu1(i) = 0;
			mu2(i) = 0;
		}

		ndata_class1 = 0; ndata_class2 = 0;
		for (size_t i = 0; i < nsamples; ++i)
		{
			if (labels[i] > 0)
			{
				samples1(ndata_class1) = samplesvec[i];
				++ndata_class1;
				mu1 += samplesvec[i];
			}
			else
			{
				samples2(ndata_class2) = samplesvec[i];
				++ndata_class2;
				mu2 += samplesvec[i];
			}
		}
		mu1 /= ndata_class1;
		mu2 /= ndata_class2;

		// if you get a compilation error coming from here (with templates
		// and a 'visual_studio_sucks_cov_helper' structure involved) then
		// you may have to patch the dlib's file 'matrix_utilities.h":
		// 
		// line 1611, replace
		//	const matrix<double,EXP::type::NR,EXP::type::NC, typename EXP::mem_manager_type> avg = mean(m);
		// by
		//  const typename EXP::type avg = mean(m);
		// 
		dlib::matrix<float> sigma1 = covariance(samples1);
		dlib::matrix<float> sigma2 = covariance(samples2);

		sample_type w_vect = pinv(sigma1 + sigma2) * (mu2 - mu1);

		trained_function_type ret;
		//ret.alpha.set_size(fdim);
		//for (int i=0; i<fdim; ++i) ret.alpha(i) = w_vect(i);
		ret.alpha = w_vect;
		ret.b = dot(w_vect, (mu1 + mu2)*0.5);
		// linear kernel idiocy
		ret.basis_vectors.set_size(fdim);
		for (size_t i = 0; i < fdim; ++i)
		{
			ret.basis_vectors(i).set_size(fdim);
			for (size_t j = 0; j < fdim; ++j)
				ret.basis_vectors(i)(j) = 0;
			ret.basis_vectors(i)(i) = 1;
		}
		return ret;
	}

#if 0
	trained_function_type train(const trained_function_type::sample_vector_type& samplesvec, const trained_function_type::scalar_vector_type& labels) const
	{
		int fdim = samplesvec(0).size();
		int nsamples = samplesvec.size();

		int ndata_class1 = 0, ndata_class2 = 0;
		for (int i=0; i<nsamples; ++i)
		{
			if (labels(i)>0)
				++ndata_class1;
			else
				++ndata_class2;
		}

		dlib::matrix<sample_type,0,1> samples1, samples2;
		samples1.set_size(ndata_class1);
		samples2.set_size(ndata_class2);
		sample_type mu1; mu1.set_size(fdim);
		sample_type mu2; mu2.set_size(fdim);
		for (int i=0; i<fdim; ++i)
		{
			mu1(i)=0;
			mu2(i)=0;
		}

		ndata_class1 = 0; ndata_class2 = 0;
		for (int i=0; i<nsamples; ++i)
		{
			if (labels(i)>0)
			{
				samples1(ndata_class1) = samplesvec(i);
				++ndata_class1;
				mu1 += samplesvec(i);
			}
			else
			{
				samples2(ndata_class2) = samplesvec(i);
				++ndata_class2;
				mu2 += samplesvec(i);
			}
		}
		mu1 /= ndata_class1;
		mu2 /= ndata_class2;
		
		// if you get a compilation error coming from here (with templates
		// and a 'visual_studio_sucks_cov_helper' structure involved) then
		// you may have to patch the dlib's file 'matrix_utilities.h":
		// 
		// line 1611, replace
		//	const matrix<double,EXP::type::NR,EXP::type::NC, typename EXP::mem_manager_type> avg = mean(m);
		// by
		//  const typename EXP::type avg = mean(m);
		// 
		dlib::matrix<float> sigma1 = covariance(samples1);
		dlib::matrix<float> sigma2 = covariance(samples2);

		sample_type w_vect = pinv(sigma1+sigma2) * (mu2 - mu1);

		trained_function_type ret;
		//ret.alpha.set_size(fdim);
		//for (int i=0; i<fdim; ++i) ret.alpha(i) = w_vect(i);
		ret.alpha = w_vect;
		ret.b = dot(w_vect,(mu1+mu2)*0.5);
		// linear kernel idiocy
		ret.basis_vectors.set_size(fdim);
		for (int i=0; i<fdim; ++i)
		{
			ret.basis_vectors(i).set_size(fdim);
			for (int j=0; j<fdim; ++j)
				ret.basis_vectors(i)(j)=0;
			ret.basis_vectors(i)(i) = 1;
		}
		return ret;
		/*LinearPredictor classifier;
		classifier.weights.resize(fdim+1);
		for (int i=0; i<fdim; ++i) classifier.weights[i] = w_vect(i);
		classifier.weights[fdim] = -dot(w_vect,(mu1+mu2)*0.5);
		return classifier;*/
	}
#endif

	void train(int nfolds, const std::vector<sample_type>& samples, const std::vector<float>& labels)
	{
		dlib::probabilistic_decision_function<kernel_type> pdecfun = dlib::train_probabilistic_decision_function(*this, samples, labels, nfolds);
		dlib::decision_function<kernel_type>& decfun = pdecfun.decision_funct;
		int dim = samples.back().size();
		// see comments in linearSVM.hpp
		m_weights.clear();
		m_weights.resize(dim + 1, 0);
		dlib::matrix<float> w(dim, 1);
		w = 0;
		for (int i = 0; i < decfun.alpha.nr(); ++i)
		{
			w += decfun.alpha(i) * decfun.basis_vectors(i);
		}
		for (int i = 0; i < dim; ++i)
			m_weights[i] = w(i);
		m_weights[dim] = -decfun.b;
		for (int i = 0; i <= dim; ++i)
			m_weights[i] *= pdecfun.alpha;
		m_weights[dim] += pdecfun.beta;

		// TODO: check if necessary here
		for (int i = 0; i <= dim; ++i)
			m_weights[i] = -m_weights[i];
	}

	double predict(const sample_type& data) const
	{
		assert(!m_weights.empty());
		double ret = m_weights.back();
		for (size_t d = 0; d < m_weights.size() - 1; ++d)
			ret += static_cast<double>(m_weights[d]) * data(d);
		return ret;
	}

	//! Classifier weights
	std::vector<float> m_weights;

};

//! Gram-Schmidt process to re-orthonormalise the basis
static void GramSchmidt(dlib::matrix<LDATrainer::sample_type,0,1>& basis, LDATrainer::sample_type& newX)
{
	// goal: find a basis so that the given vector is the new X
	// principle: at least one basis vector is not orthogonal with newX (except if newX is null but we suppose this is not the case)
	// => use the max dot product vector, and replace it by newX. this forms a set of
	// linearly independent vectors.
	// then apply the Gram-Schmidt process
	long dim = basis.size();
	double maxabsdp = -1.0;
	long selectedCoord = 0;
	for (long i=0; i<dim; ++i)
	{
		double absdp = fabs(dot(basis(i),newX));
		if (absdp > maxabsdp)
		{
			absdp = maxabsdp;
			selectedCoord = i;
		}
	}
	// swap basis vectors to use the selected coord as the X vector, then replaced by newX
	basis(selectedCoord) = basis(0);
	basis(0) = newX;
	// Gram-Schmidt process to re-orthonormalise the basis.
	// Thanks Wikipedia for the stabilized version
	for (long j = 0; j < dim; ++j)
	{
		for (long i = 0; i < j; ++i)
			basis(j) -= (dot(basis(j),basis(i)) / dot(basis(i),basis(i))) * basis(i);

		basis(j) /= sqrt(dot(basis(j),basis(j)));
	}
}

//! Compute pos. and neg. reference points
static void ComputeReferencePoints(	Classifier::Point2D& refpt_pos,
									Classifier::Point2D& refpt_neg,
									const std::vector<float>& proj1,
									const std::vector<float>& proj2,
									const std::vector<float>& labels,
									unsigned* _npos = 0,
									unsigned* _nneg = 0)
{
	assert(proj1.size() == proj2.size() && proj1.size() == labels.size());
	
	refpt_neg = refpt_pos = Classifier::Point2D(0,0);

	size_t npos = 0;
	size_t nneg = 0;
	for (size_t i=0; i<labels.size(); ++i)
	{
		if (labels[i] < 0)
		{
			refpt_neg += Classifier::Point2D(proj1[i], proj2[i]);
			++nneg;
		}
		else
		{
			refpt_pos += Classifier::Point2D(proj1[i], proj2[i]);
			++npos;
		}
	}
	if (npos)
		refpt_pos /= static_cast<float>(npos);
	if (nneg)
		refpt_neg /= static_cast<float>(nneg);

	if (_npos)
		*_npos = npos;
	if (_nneg)
		*_nneg = nneg;
}

//! Experimental (same as Brodu's code): dilatation to highlight the internal data structure
static bool DilateClassifier(	Classifier& classifier,
								std::vector<float>& proj1,
								std::vector<float>& proj2,
								const std::vector<float>& labels,
								const std::vector<LDATrainer::sample_type>& samples,
								LDATrainer& trainer,
								LDATrainer& orthoTrainer)
{
	//m_app->dispToConsole("[Cloud dilatation]");
	Classifier::Point2D e1 = classifier.refPointPos - classifier.refPointNeg;
	e1.normalize();
	Classifier::Point2D e2(-e1.y, e1.x);
	Classifier::Point2D ori = (classifier.refPointPos + classifier.refPointNeg) / 2;

	float m11=0, m21=0, m12=0, m22=0; // m12, m22 null by construction
	float v11=0, v12=0, v21=0, v22=0;
	size_t nsamples1 = 0;
	size_t nsamples2 = 0;
	size_t nsamples = proj1.size();
	assert(proj1.size() == proj2.size());
	for (size_t i=0; i<nsamples; ++i)
	{
		Classifier::Point2D p(proj1[i], proj2[i]);
		p -= ori;
		float p1 = p.dot(e1);
		float p2 = p.dot(e2);
		if (labels[i] < 0)
		{
			m11 += p1; v11 += p1*p1;
			m12 += p2; v12 += p2*p2;
			++nsamples1;
		}
		else
		{
			m21 += p1; v21 += p1*p1;
			m22 += p2; v22 += p2*p2;
			++nsamples2;
		}
	}
	m11 /= nsamples1;
	v11 = (v11 - m11*m11*nsamples1) / (nsamples1-1);
	m21 /= nsamples2;
	v21 = (v21 - m21*m21*nsamples2) / (nsamples2-1);
	m12 /= nsamples1;
	v12 = (v12 - m12*m12*nsamples1) / (nsamples1-1);
	m22 /= nsamples2;
	v22 = (v22 - m22*m22*nsamples2) / (nsamples2-1);

	float d1 = sqrt(v11/v12);
	float d2 = sqrt(v21/v22);
	classifier.axisScaleRatio = sqrt(d1*d2);

	float bdValues[4] = {e1.x, e1.y, e2.x/classifier.axisScaleRatio, e2.y/classifier.axisScaleRatio};
	dlib::matrix<float,2,2> bd(bdValues);

	float biValues[4] = {e1.x, e2.x, e1.y, e2.y};
	dlib::matrix<float,2,2> bi(biValues);
	dlib::matrix<float,2,2> c = inv(trans(bd)) /* bi * bd */;

	std::vector<float>& w1 = trainer.m_weights;
	std::vector<float>& w2 = orthoTrainer.m_weights;
	assert(w1.size() == w2.size());

	std::vector<float> wn1, wn2;
	try
	{
		wn1.resize(w1.size());
		wn2.resize(w2.size());
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	// first shift so the center of the figure is at the midpoint
	w1.back() -= ori.x;
	w2.back() -= ori.y;
	// now transform / scale along e2
	{
		for (size_t i=0; i<w1.size(); ++i)
		{
			wn1[i] = c(0,0) * w1[i] + c(0,1) * w2[i];
			wn2[i] = c(1,0) * w1[i] + c(1,1) * w2[i];
		}
	}

	trainer.m_weights = wn1;
	orthoTrainer.m_weights = wn2;

	// reset projections
	{
		for (size_t i=0; i<nsamples; ++i)
		{
			proj1[i] = trainer.predict(samples[i]);
			proj2[i] = orthoTrainer.predict(samples[i]);
		}
	}

	classifier.weightsAxis1 = wn1;
	classifier.weightsAxis2 = wn2;

	//update reference points
	ComputeReferencePoints(	classifier.refPointPos,
							classifier.refPointNeg,
							proj1,
							proj2,
							labels);

	return true;
}

#endif //QCANUPO_CLASSIFIER_HEADER
