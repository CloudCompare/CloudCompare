#ifndef LEVMARFUNC_HEADER
#define LEVMARFUNC_HEADER

template< class ScalarT >
struct LevMarFunc {
	virtual ScalarT operator()(const ScalarT *param) const = 0;
	virtual void operator()(const ScalarT *param, ScalarT *gradient) const = 0;
};

#endif
