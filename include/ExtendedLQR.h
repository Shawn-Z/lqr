#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include "matrix.h"

template <size_t xDim>
void regularize(SymmetricMatrix<xDim>& Q) {
	SymmetricMatrix<xDim> D;
	Matrix<xDim,xDim> V;
	jacobi(Q, V, D);
	for (size_t i = 0; i < xDim; ++i) {
		if (D(i,i) < 0) {
			D(i,i) = 0;
		}
	}
	Q = SymProd(V,D*~V);
}

static const double DEFAULTSTEPSIZE = 0.0009765625;

template <size_t aDim, typename T>
inline Matrix<aDim> jacobian(const Matrix<aDim>& a, const T& b, double (*f)(const Matrix<aDim>&, const T&), double jStep = DEFAULTSTEPSIZE) {
	Matrix<aDim> A;
	Matrix<aDim> ar(a), al(a);
	for (size_t i = 0; i < aDim; ++i) {
		ar[i] += jStep; al[i] -= jStep;
		A[i] = (f(ar, b) - f(al, b)) / (2*jStep);
		ar[i] = al[i] = a[i];
	}
	return A;
}

template <size_t aDim>
inline Matrix<aDim> jacobian(const Matrix<aDim>& a, double (*f)(const Matrix<aDim>&), double jStep = DEFAULTSTEPSIZE) {
	Matrix<aDim> A;
	Matrix<aDim> ar(a), al(a);
	for (size_t i = 0; i < aDim; ++i) {
		ar[i] += jStep; al[i] -= jStep;
		A[i] = (f(ar) - f(al)) / (2*jStep);
		ar[i] = al[i] = a[i];
	}
	return A;
}


template <size_t aDim, typename T, size_t yDim>
inline Matrix<yDim,aDim> jacobian1(const Matrix<aDim>& a, const T& b, Matrix<yDim> (*f)(const Matrix<aDim>&, const T&), double jStep = DEFAULTSTEPSIZE) {
	Matrix<yDim,aDim> A;
	Matrix<aDim> ar(a), al(a);
	for (size_t i = 0; i < aDim; ++i) {
		ar[i] += jStep; al[i] -= jStep;
		A.insert(0,i, (f(ar, b) - f(al, b)) / (2*jStep));
		ar[i] = al[i] = a[i];
	}
	return A;
}

template <size_t aDim, typename T1, typename T2>
inline Matrix<aDim> jacobian1(const Matrix<aDim>& a, const T1& b, const T2& c, double (*f)(const Matrix<aDim>&, const T1&, const T2&), double jStep = DEFAULTSTEPSIZE) {
	Matrix<aDim> A;
	Matrix<aDim> ar(a), al(a);
	for (size_t i = 0; i < aDim; ++i) {
		ar[i] += jStep; al[i] -= jStep;
		A[i] = (f(ar, b, c) - f(al, b, c)) / (2*jStep);
		ar[i] = al[i] = a[i];
	}
	return A;
}


template <typename T1, size_t bDim, size_t yDim>
inline Matrix<yDim,bDim> jacobian2(const T1& a, const Matrix<bDim>& b, Matrix<yDim> (*f)(const T1&, const Matrix<bDim>&), double jStep = DEFAULTSTEPSIZE) {
	Matrix<yDim,bDim> B;
	Matrix<bDim> br(b), bl(b);
	for (size_t i = 0; i < bDim; ++i) {
		br[i] += jStep; bl[i] -= jStep;
		B.insert(0,i, (f(a, br) - f(a, bl)) / (2*jStep));
		br[i] = bl[i] = b[i];
	}
	return B;
}

template <typename T1, size_t bDim, typename T2>
inline Matrix<bDim> jacobian2(const T1& a, const Matrix<bDim>& b, const T2& c, double (*f)(const T1&, const Matrix<bDim>&, const T2&), double jStep = DEFAULTSTEPSIZE) {
	Matrix<bDim> B;
	Matrix<bDim> br(b), bl(b);
	for (size_t i = 0; i < bDim; ++i) {
		br[i] += jStep; bl[i] -= jStep;
		B[i] = (f(a, br, c) - f(a, bl, c)) / (2*jStep);
		br[i] = bl[i] = b[i];
	}
	return B;
}


template <size_t aDim, typename T>
inline SymmetricMatrix<aDim> hessian(const Matrix<aDim>& a, const T& t, double (*f)(const Matrix<aDim>&, const T&), double jStep = DEFAULTSTEPSIZE) {
	SymmetricMatrix<aDim> Q;

	double p = f(a, t);

	// diag(Q)
	Matrix<aDim> ar(a), al(a);
	for (size_t i = 0; i < aDim; ++i) {
		ar[i] += jStep; al[i] -= jStep;
		Q(i,i) = (f(al, t) - 2.0*p + f(ar, t)) / (jStep * jStep);
		ar[i] = al[i] = a[i];
	}

	Matrix<aDim> atr(a), atl(a), abr(a), abl(a);
	for (size_t i = 1; i < aDim; ++i) {
		atr[i] += jStep; atl[i] -= jStep; abr[i] += jStep; abl[i] -= jStep;
		for (size_t j = 0; j < i; ++j) {
			atr[j] += jStep; atl[j] += jStep; abr[j] -= jStep; abl[j] -= jStep;
			Q(i,j) = (f(abl, t) + f(atr, t) - f(atl, t) - f(abr, t)) / (4.0*jStep*jStep);
			atr[j] = atl[j] = abr[j] = abl[j] = a[j];
		}
		atr[i] = atl[i] = abr[i] = abl[i] = a[i];
	}
	return Q;
}

template <size_t aDim>
inline SymmetricMatrix<aDim> hessian(const Matrix<aDim>& a, double (*f)(const Matrix<aDim>&), double jStep = DEFAULTSTEPSIZE) {
	SymmetricMatrix<aDim> Q;

	double p = f(a);

	// diag(Q)
	Matrix<aDim> ar(a), al(a);
	for (size_t i = 0; i < aDim; ++i) {
		ar[i] += jStep; al[i] -= jStep;
		Q(i,i) = (f(al) - 2.0*p + f(ar)) / (jStep * jStep);
		ar[i] = al[i] = a[i];
	}

	Matrix<aDim> atr(a), atl(a), abr(a), abl(a);
	for (size_t i = 1; i < aDim; ++i) {
		atr[i] += jStep; atl[i] -= jStep; abr[i] += jStep; abl[i] -= jStep;
		for (size_t j = 0; j < i; ++j) {
			atr[j] += jStep; atl[j] += jStep; abr[j] -= jStep; abl[j] -= jStep;
			Q(i,j) = (f(abl) + f(atr) - f(atl) - f(abr)) / (4.0*jStep*jStep);
			atr[j] = atl[j] = abr[j] = abl[j] = a[j];
		}
		atr[i] = atl[i] = abr[i] = abl[i] = a[i];
	}
	return Q;
}

template <size_t aDim, typename T1, typename T2>
inline SymmetricMatrix<aDim> hessian1(const Matrix<aDim>& a, const T1& t1, const T2& t2, double (*f)(const Matrix<aDim>&, const T1&, const T2&), double jStep = DEFAULTSTEPSIZE) {
	SymmetricMatrix<aDim> Q;

	double p = f(a, t1, t2);

	// diag(Q)
	Matrix<aDim> ar(a), al(a);
	for (size_t i = 0; i < aDim; ++i) {
		ar[i] += jStep; al[i] -= jStep;
		Q(i,i) = (f(al, t1, t2) - 2.0*p + f(ar, t1, t2)) / (jStep * jStep);
		ar[i] = al[i] = a[i];
	}

	Matrix<aDim> atr(a), atl(a), abr(a), abl(a);
	for (size_t i = 1; i < aDim; ++i) {
		atr[i] += jStep; atl[i] -= jStep; abr[i] += jStep; abl[i] -= jStep;
		for (size_t j = 0; j < i; ++j) {
			atr[j] += jStep; atl[j] += jStep; abr[j] -= jStep; abl[j] -= jStep;
			Q(i,j) = (f(abl, t1, t2) + f(atr, t1, t2) - f(atl, t1, t2) - f(abr, t1, t2)) / (4.0*jStep*jStep);
			atr[j] = atl[j] = abr[j] = abl[j] = a[j];
		}
		atr[i] = atl[i] = abr[i] = abl[i] = a[i];
	}
	return Q;
}

template <typename T1, size_t aDim, typename T2>
inline SymmetricMatrix<aDim> hessian2(const T1& t1, const Matrix<aDim>& a, const T2& t2, double (*f)(const T1&, const Matrix<aDim>&, const T2&), double jStep = DEFAULTSTEPSIZE) {
	SymmetricMatrix<aDim> Q;

	double p = f(t1, a, t2);

	// diag(Q)
	Matrix<aDim> ar(a), al(a);
	for (size_t i = 0; i < aDim; ++i) {
		ar[i] += jStep; al[i] -= jStep;
		Q(i,i) = (f(t1, al, t2) - 2.0*p + f(t1, ar, t2)) / (jStep * jStep);
		ar[i] = al[i] = a[i];
	}

	Matrix<aDim> atr(a), atl(a), abr(a), abl(a);
	for (size_t i = 1; i < aDim; ++i) {
		atr[i] += jStep; atl[i] -= jStep; abr[i] += jStep; abl[i] -= jStep;
		for (size_t j = 0; j < i; ++j) {
			atr[j] += jStep; atl[j] += jStep; abr[j] -= jStep; abl[j] -= jStep;
			Q(i,j) = (f(t1, abl, t2) + f(t1, atr, t2) - f(t1, atl, t2) - f(t1, abr, t2)) / (4.0*jStep*jStep);
			atr[j] = atl[j] = abr[j] = abl[j] = a[j];
		}
		atr[i] = atl[i] = abr[i] = abl[i] = a[i];
	}
	return Q;
}

template <size_t aDim, size_t bDim, typename T2>
inline Matrix<aDim, bDim> hessian12(const Matrix<aDim>& a, const Matrix<bDim>& b, const T2& t2, double (*f)(const Matrix<aDim>&, const Matrix<bDim>&, const T2&), double jStep = DEFAULTSTEPSIZE) {
	Matrix<aDim, bDim> Q;

	Matrix<aDim> atr(a), atl(a), abr(a), abl(a);
	Matrix<bDim> btr(b), btl(b), bbr(b), bbl(b);

	for (size_t i = 0; i < aDim; ++i) {
		atr[i] += jStep; atl[i] -= jStep; abr[i] += jStep; abl[i] -= jStep;
		for (size_t j = 0; j < bDim; ++j) {
			btr[j] += jStep; btl[j] += jStep; bbr[j] -= jStep; bbl[j] -= jStep;
			Q(i,j) = (f(abl, bbl, t2) + f(atr, btr, t2) - f(atl, btl, t2) - f(abr, bbr, t2)) / (4.0*jStep*jStep);
			btr[j] = btl[j] = bbr[j] = bbl[j] = b[j];
		}
		atr[i] = atl[i] = abr[i] = abl[i] = a[i];
	}
	return Q;
}


template <size_t xDim, size_t uDim>
inline void iterativeLQR(const size_t& ell, const Matrix<xDim>& initState, const Matrix<uDim>& uNominal, Matrix<xDim> (*g)(const Matrix<xDim>&, const Matrix<uDim>&), void (*quadratizeFinalCost)(const Matrix<xDim>&, SymmetricMatrix<xDim>&, Matrix<xDim>&, const size_t&), double (*cell)(const Matrix<xDim>&), void (*quadratizeCost)(const Matrix<xDim>&, const Matrix<uDim>&, const size_t&, Matrix<uDim,xDim>&, SymmetricMatrix<xDim>&, SymmetricMatrix<uDim>&, Matrix<xDim>&, Matrix<uDim>&, const size_t&), double (*ct)(const Matrix<xDim>&, const Matrix<uDim>&, const size_t&), std::vector<Matrix<uDim,xDim> >& L, std::vector<Matrix<uDim> >& l, bool vis, size_t& iter) {
	size_t maxIter = 1000;

	L.resize(ell, zeros<uDim,xDim>());
	l.resize(ell, uNominal);

	std::vector<Matrix<xDim> > xHat(ell + 1, zero<xDim>());
	std::vector<Matrix<xDim> > xHatNew(ell + 1, zero<xDim>());
	std::vector<Matrix<uDim> > uHat(ell);
	std::vector<Matrix<uDim> > uHatNew(ell);

	double oldCost = -log(0.0);

	for (iter = 0; iter < maxIter; ++iter) {
		double newCost;
		double alpha = 1.0;

		do {
			newCost = 0;

			// init trajectory
			xHatNew[0] = initState;
			for (size_t t = 0; t < ell; ++t) {
				uHatNew[t] = (1.0 - alpha)*uHat[t] + L[t]*(xHatNew[t] - (1.0 - alpha)*xHat[t]) + alpha*l[t];
				xHatNew[t+1] = g(xHatNew[t], uHatNew[t]);

				newCost += ct(xHatNew[t], uHatNew[t], t);
			}
			newCost += cell(xHatNew[ell]);

			alpha *= 0.5;
		} while (!(newCost < oldCost || abs((oldCost - newCost) / newCost) < 1e-4));

		xHat = xHatNew;
		uHat = uHatNew;

		if (vis) {
			std::cerr << "Iter: " << iter << " Alpha: " << 2*alpha << " Rel. progress: " << (oldCost - newCost) / newCost << " Cost: " << newCost << " Time step: " << exp(xHat[0][xDim-1]) << std::endl;
		}

		if (abs((oldCost - newCost) / newCost) < 1e-4) {
			return;
		}

		oldCost = newCost;

		// pass
		SymmetricMatrix<xDim> S;
		Matrix<xDim> s;

		quadratizeFinalCost(xHat[ell], S, s, iter);

		for (size_t t = ell-1; t != -1; --t) {
			const Matrix<xDim, xDim> A = jacobian1(xHat[t], uHat[t], g);
			const Matrix<xDim, uDim> B = jacobian2(xHat[t], uHat[t], g);
			const Matrix<xDim> c = xHat[t+1] - A*xHat[t] - B*uHat[t];

			Matrix<uDim, xDim> P;
			SymmetricMatrix<xDim> Q;
			SymmetricMatrix<uDim> R;
			Matrix<xDim> q;
			Matrix<uDim> r;

			quadratizeCost(xHat[t], uHat[t], t, P, Q, R, q, r, iter);

			const Matrix<uDim,xDim> C = ~B*S*A + P;
			const SymmetricMatrix<xDim> D = SymProd(~A,S*A) + Q;
			const SymmetricMatrix<uDim> E = SymProd(~B,S*B) + R;
			const Matrix<xDim> d = ~A*(s + S*c) + q;
			const Matrix<uDim> e = ~B*(s + S*c) + r;

			L[t] = -(E%C);
			l[t] = -(E%e);

			S = D + SymProd(~C, L[t]);
			s = d + ~C*l[t];
		}
	}
}


template <size_t xDim, size_t uDim>
inline double extendedLQR(const size_t& ell,
                          const Matrix<xDim>& startState,
                          const Matrix<uDim>& uNominal,
                          Matrix<xDim> (*g)(const Matrix<xDim>&, const Matrix<uDim>&),
                          Matrix<xDim> (*gBar)(const Matrix<xDim>&, const Matrix<uDim>&),
                          void (*quadratizeFinalCost)(const Matrix<xDim>&, SymmetricMatrix<xDim>&, Matrix<xDim>&, const size_t&),
                          double (*cell)(const Matrix<xDim>&),
                          void (*quadratizeCost)(const Matrix<xDim>&, const Matrix<uDim>&, const size_t&, Matrix<uDim,xDim>&, SymmetricMatrix<xDim>&, SymmetricMatrix<uDim>&, Matrix<xDim>&, Matrix<uDim>&, const size_t&),
                          double (*ct)(const Matrix<xDim>&, const Matrix<uDim>&, const size_t&),
                          std::vector<Matrix<uDim,xDim> >& L,
                          std::vector<Matrix<uDim> >& l,
                          bool vis,
                          size_t& iter) {
	size_t maxIter = 1000;

	// Initialization
	L.resize(ell, zeros<uDim,xDim>());
	l.resize(ell, uNominal);

	std::vector<SymmetricMatrix<xDim> > S(ell + 1, zeros<xDim>());
	std::vector<Matrix<xDim> > s(ell + 1, zero<xDim>());
	std::vector<SymmetricMatrix<xDim> > SBar(ell + 1);
	std::vector<Matrix<xDim> > sBar(ell + 1);

	double oldCost = -log(0.0);
	Matrix<xDim> xHat = startState;

	SBar[0] = zeros<xDim>();
	sBar[0] = zero<xDim>();

	for (iter = 0; iter < maxIter; ++iter) {
		// forward pass
		for (size_t t = 0; t < ell; ++t) {
			const Matrix<uDim> uHat = L[t]*xHat + l[t];
			const Matrix<xDim> xHatPrime = g(xHat, uHat);

			const Matrix<xDim, xDim> ABar = jacobian1(xHatPrime, uHat, gBar);
			const Matrix<xDim, uDim> BBar = jacobian2(xHatPrime, uHat, gBar);
			const Matrix<xDim> cBar = xHat - ABar*xHatPrime - BBar*uHat;

			Matrix<uDim, xDim> P;
			SymmetricMatrix<xDim> Q;
			SymmetricMatrix<uDim> R;
			Matrix<xDim> q;
			Matrix<uDim> r;

			quadratizeCost(xHat, uHat, t, P, Q, R, q, r, iter);

			const SymmetricMatrix<xDim> SBarQ = SBar[t] + Q;
			const Matrix<xDim> sBarqSBarQcBar = sBar[t] + q + SBarQ*cBar;

			const Matrix<uDim,xDim> CBar = ~BBar*SBarQ*ABar + P*ABar;
			const SymmetricMatrix<xDim> DBar = SymProd(~ABar,SBarQ*ABar);
			const SymmetricMatrix<uDim> EBar = SymProd(~BBar,SBarQ*BBar) + R + SymSum(P*BBar);
			const Matrix<xDim> dBar = ~ABar*sBarqSBarQcBar;
			const Matrix<uDim> eBar = ~BBar*sBarqSBarQcBar + r + P*cBar;

			L[t] = -(EBar%CBar);
			l[t] = -(EBar%eBar);

			SBar[t+1] = DBar + SymProd(~CBar, L[t]);
			sBar[t+1] = dBar + ~CBar*l[t];

			xHat = -((S[t+1] + SBar[t+1])%(s[t+1] + sBar[t+1]));
		}

		// backward pass
		quadratizeFinalCost(xHat, S[ell], s[ell], iter);
		xHat = -((S[ell] + SBar[ell])%(s[ell] + sBar[ell]));

		for (size_t t = ell - 1; t != -1; --t) {
			const Matrix<uDim> uHat = L[t]*xHat + l[t];
			const Matrix<xDim> xHatPrime = gBar(xHat, uHat);

			const Matrix<xDim, xDim> A = jacobian1(xHatPrime, uHat, g);
			const Matrix<xDim, uDim> B = jacobian2(xHatPrime, uHat, g);
			const Matrix<xDim> c = xHat - A*xHatPrime - B*uHat;

			Matrix<uDim, xDim> P;
			SymmetricMatrix<xDim> Q;
			SymmetricMatrix<uDim> R;
			Matrix<xDim> q;
			Matrix<uDim> r;

			quadratizeCost(xHatPrime, uHat, t, P, Q, R, q, r, iter);

			const Matrix<uDim,xDim> C = ~B*S[t+1]*A + P;
			const SymmetricMatrix<xDim> D = SymProd(~A,S[t+1]*A) + Q;
			const SymmetricMatrix<uDim> E = SymProd(~B,S[t+1]*B) + R;
			const Matrix<xDim> d = ~A*(s[t+1] + S[t+1]*c) + q;
			const Matrix<uDim> e = ~B*(s[t+1] + S[t+1]*c) + r;

			L[t] = -(E%C);
			l[t] = -(E%e);

			S[t] = D + SymProd(~C, L[t]);
			s[t] = d + ~C*l[t];

			xHat = -((S[t] + SBar[t])%(s[t] + sBar[t]));
		}

		// compute cost
		double newCost = 0;
		Matrix<xDim> x = xHat;
		for (size_t t = 0; t < ell; ++t) {
			Matrix<uDim> u = L[t]*x + l[t];
			newCost += ct(x, u, t);
			x = g(x, u);
		}
		newCost += cell(x);

		if (vis) {
			std::cerr << "Iter: " << iter << " Rel. progress: " << (oldCost - newCost) / newCost << " Cost: " << newCost << " Time step: " << exp(xHat[xDim-1]) << std::endl;
		}
		if (abs((oldCost - newCost) / newCost) < 1e-4) {
			++iter;
			return exp(xHat[xDim-1]);
		}
		oldCost = newCost;
	}

	return exp(xHat[xDim-1]);
}