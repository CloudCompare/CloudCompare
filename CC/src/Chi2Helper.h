//############################################################################
//#                                                                          #
//#                            CLOUDCOMPARE                                  #
//#                                                                          #
//#  AUTEUR   : Daniel Girardeau-Montaut, doctorant (2003-2006)              #
//#  COPYRIGHT 2006 : EDF R&D / TELECOM PARISTECH (TSI)                      #
//#                                                                          #
//#  ATTENTION, CE CODE CORRESPOND A L'IMPLEMENTATION D'ALGORITHMES ET       #
//#  DE TRAITEMENTS SPECIFIQUES DE DONNEES DEVELOPPEES DANS LE CADRE         #
//#  D'UNE THESE (CONVENTION CIFRE) FINANCEE PAR EDF R&D ET ENCADREE PAR     #
//#  TELCOM PARISTECH (LABORATOIRE TSI). SA CONSULTATION ET SON UTILISATION  #
//#  SONT LIMITEES AUX SEULES PERSONNES AUTORISEES D'EDF R&D ET TELECOM      #
//#  PARISTECH. DE PLUS, SA DIFFUSION EST FORTEMENT DECONSEILLEE ETANT DONNE #
//#  SON CARACTERE EXPERIMENTAL ET PARFOIS INCOMPLET. CE N'EST QU'UN PRO-    #
//#  TOTYPE ATTENDANT UNE STRUCTURATION ET UNE INSTRUMENTATION PROPRE,       #
//#  EN PARTICULIER AU NIVEAU DU MODELE OBJET ET DE LA DOCUMENTATION.        #
//#  LE COMPORTEMENT DES ALGORITHMES N'EST PAS GARANTI EN L'ETAT.            #
//#                                                                          #
//############################################################################

#ifndef CHI2_HELPER_HEADER
#define CHI2_HELPER_HEADER

//system
#include <math.h>

//! Package of methods to compute Chi2 related stuff
/**  The following JavaScript functions for calculating normal and
chi-square probabilities and critical values were adapted by
John Walker from C implementations
written by Gary Perlman of Wang Institute, Tyngsboro, MA
01879.  Both the original C code and this JavaScript edition
are in the public domain.
**/
class Chi2Helper
{
public:
	//! Probability of normal z value
	/** Adapted from a polynomial approximation in:
		Ibbetson D, Algorithm 209
		Collected Algorithms of the CACM 1963 p. 616
		Note:
		This routine has six digit accuracy, so it is only useful for absolute
		z values < 6.  For z values >= to 6.0, poz() returns 0.0.
	**/
	static double poz(double z)
	{
		double x;

		if (z == 0.0)
		{
			x = 0.0;
		}
		else
		{
			double y = 0.5 * fabs(z);
			if (y >= 3.0) /* Maximum meaningful z value (6) divided by 2 */
			{
				x = 1.0;
			}
			else if (y < 1.0)
			{
				double w = y * y;
				x = ((((((((0.000124818987 * w
					- 0.001075204047) * w + 0.005198775019) * w
					- 0.019198292004) * w + 0.059054035642) * w
					- 0.151968751364) * w + 0.319152932694) * w
					- 0.531923007300) * w + 0.797884560593) * y * 2.0;
			}
			else
			{
				y -= 2.0;
				x = (((((((((((((-0.000045255659 * y
					+ 0.000152529290) * y - 0.000019538132) * y
					- 0.000676904986) * y + 0.001390604284) * y
					- 0.000794620820) * y - 0.002034254874) * y
					+ 0.006549791214) * y - 0.010557625006) * y
					+ 0.011630447319) * y - 0.009279453341) * y
					+ 0.005353579108) * y - 0.002141268741) * y
					+ 0.000535310849) * y + 0.999936657524;
			}
		}
		return z > 0.0 ? ((x + 1.0) * 0.5) : ((1.0 - x) * 0.5);
	}

	//! Value above which exp(EXP_MAX_A_VALUE) diverges
	static inline double EXP_MAX_A_VALUE() { return 50.0; }

	//! Probability of chi-square value
	/** Adapted from:
	Hill, I. D. and Pike, M. C.  Algorithm 299
	Collected Algorithms for the CACM 1967 p. 243
	Updated for rounding errors based on remark in
	ACM TOMS June 1985, page 185
	**/
	static double pochisq(double x, int df)
	{
#ifndef LOG_SQRT_PI
#define LOG_SQRT_PI 0.5723649429247000870717135 /* log(sqrt(pi)) */
#endif
#ifndef I_SQRT_PI
#define I_SQRT_PI 0.5641895835477562869480795   /* 1 / sqrt(pi) */
#endif

		if (x <= 0.0 || df < 1)
			return 1.0;

		double a = 0.5 * x;
		bool even = !(df & 1); /* True if df is an even number */
		double y = 0;
		if (df > 1) {
			y = exp(-a);
		}
		double s = (even ? y : (2.0 * poz(-sqrt(x))));
		if (df > 2) {
			x = 0.5 * (df - 1.0);
			double z = (even ? 1.0 : 0.5);
			if (a > EXP_MAX_A_VALUE())
			{
				double e = (even ? 0.0 : LOG_SQRT_PI);
				double c = log(a);
				while (z <= x)
				{
					e = log(z)+e;
					s += exp(c*z-a-e);
					z += 1.0;
				}
				return s;
			}
			else
			{
				double e = (even ? 1.0 : (I_SQRT_PI / sqrt(a)));
				double c = 0.0;
				while (z <= x)
				{
					e = e*(a/z);
					c = c+e;
					z += 1.0;
				}
				return c*y+s;
			}
		}
		else return s;
	}

	//! Compute critical chi-square value toproduce given p.
	/** We just do a bisection search for a value within CHI_EPSILON,
	relying on the monotonicity of pochisq().
	**/
	static double critchi(double p, int df)
	{
		double CHI_EPSILON = 0.000001;   /* Accuracy of critchi approximation */
		double CHI_MAX = 99999.0;        /* Maximum chi-square value */
		double minchisq = 0.0;
		double maxchisq = CHI_MAX;
		double chisqval;

		if (p <= 0.0)
			return maxchisq;
		else if (p >= 1.0)
			return 0.0;

		chisqval = df / sqrt(p);    /* fair first value */
		while ((maxchisq - minchisq) > CHI_EPSILON)
		{
			if (pochisq(chisqval, df) < p)
				maxchisq = chisqval;
			else
				minchisq = chisqval;

			chisqval = (maxchisq + minchisq) * 0.5;
		}

		return chisqval;
	}

};

#endif //CHI2_HELPER_HEADER
