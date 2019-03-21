//#include <SDKDDKVer.h>
#include <stdio.h>
//#include <tchar.h>

#include "ExtendedLQR.h"

// Set dimensions
#define X_DIM 3
#define U_DIM 2
#define DIM 2

struct Obstacle {
	Matrix<DIM> pos;
	double radius;
	size_t dim;
};

size_t ell;
SymmetricMatrix<X_DIM> Q;
double rotCost;
Matrix<X_DIM> xGoal, xStart;
SymmetricMatrix<U_DIM> R;
Matrix<U_DIM> uNominal;
double obstacleFactor;
double scaleFactor;

std::vector<Obstacle> obstacles;
Matrix<DIM> bottomLeft;
Matrix<DIM> topRight;
double robotRadius;
double dt;

// Obstacle-cost term in local cost function
inline double obstacleCost(const Matrix<X_DIM>& x) {
	double cost = 0;

	for (size_t i = 0; i < obstacles.size(); ++i) {
		Matrix<DIM> d = x.subMatrix<DIM>(0,0) - obstacles[i].pos;
		double distr = sqrt(scalar(~d*d));
		double dist = distr - robotRadius - obstacles[i].radius;
		cost += obstacleFactor * exp(-scaleFactor*dist);
	}
	for (size_t i = 0; i < DIM; ++i) {
		double dist = (x[i] - bottomLeft[i]) - robotRadius;
		cost += obstacleFactor * exp(-scaleFactor*dist);
	}
	for (size_t i = 0; i < DIM; ++i) {
		double dist = (topRight[i] - x[i]) - robotRadius;
		cost += obstacleFactor * exp(-scaleFactor*dist);
	}
	return cost;	
}

inline void quadratizeObstacleCost(const Matrix<X_DIM>& x, SymmetricMatrix<X_DIM>& Q, Matrix<X_DIM>& q) {
	SymmetricMatrix<DIM> QObs = zeros<DIM>();
	Matrix<DIM> qObs = zero<DIM>();

	for (size_t i = 0; i < obstacles.size(); ++i) {
		Matrix<DIM> d = x.subMatrix<DIM>(0,0) - obstacles[i].pos;
		double distr = sqrt(scalar(~d*d));
		d /= distr;
		double dist = distr - robotRadius - obstacles[i].radius;

		Matrix<DIM> d_ortho;
		d_ortho[0] = d[1];
		d_ortho[1] = -d[0];

		double a0 = obstacleFactor * exp(-scaleFactor*dist);
		double a1 = -scaleFactor*a0;
		double a2 = -scaleFactor*a1;

		double b2 = a1 / distr;

		QObs += a2*SymProd(d,~d) + b2*SymProd(d_ortho,~d_ortho);
		qObs += a1*d;
	}
	for (size_t i = 0; i < DIM; ++i) {
		double dist = (x[i] - bottomLeft[i]) - robotRadius;

		Matrix<DIM> d = zero<DIM>();
		d[i] = 1.0;

		double a0 = obstacleFactor * exp(-scaleFactor*dist);
		double a1 = -scaleFactor*a0;
		double a2 = -scaleFactor*a1;

		QObs += a2*SymProd(d,~d);
		qObs += a1*d;
	}
	for (size_t i = 0; i < DIM; ++i) {
		double dist = (topRight[i] - x[i]) - robotRadius;

		Matrix<DIM> d = zero<DIM>();
		d[i] = -1.0;

		double a0 = obstacleFactor * exp(-scaleFactor*dist);
		double a1 = -scaleFactor*a0;
		double a2 = -scaleFactor*a1;

		QObs += a2*SymProd(d,~d);
		qObs += a1*d;
	}

	regularize(QObs);
	Q.insert(0, QObs + Q.subSymmetricMatrix<DIM>(0));
	q.insert(0,0, qObs - QObs*x.subMatrix<DIM>(0,0) + q.subMatrix<DIM>(0,0));
}

// Local cost-function c_t(x_t, u_t)
inline double ct(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u, const size_t& t) {
	double cost = 0;
	if (t == 0) {
		cost += scalar(~(x - xStart)*Q*(x - xStart));
	}
	cost += scalar(~(u - uNominal)*R*(u - uNominal));
	cost += obstacleCost(x);
	return cost;
}

inline void quadratizeCost(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u, const size_t& t, Matrix<U_DIM,X_DIM>& Pt, SymmetricMatrix<X_DIM>& Qt, SymmetricMatrix<U_DIM>& Rt, Matrix<X_DIM>& qt, Matrix<U_DIM>& rt, const size_t& iter) {
	/*Qt = hessian1(x, u, t, c); 
	Pt = ~hessian12(x, u, t, c);
	Rt = hessian2(x, u, t, c);
	qt = jacobian1(x, u, t, c) - Qt*x - ~Pt*u;
	rt = jacobian2(x, u, t, c) - Pt*x - Rt*u;*/

	if (t == 0) {
		Qt = Q;
		qt = -(Q*xStart);
	} else {
		Qt.reset(); 
		qt.reset();

		if (iter < 2) {
			Qt(2,2) = rotCost;
			qt[2] = -rotCost*(M_PI/2);
		}
	}
	Rt = R;
	rt = -(R*uNominal);
	Pt.reset();

	quadratizeObstacleCost(x, Qt, qt);
}

// Final cost function c_\ell(x_\ell)
inline double cell(const Matrix<X_DIM>& x) {
	double cost = 0;
	cost += scalar(~(x - xGoal)*Q*(x - xGoal));
	return cost;
}

inline void quadratizeFinalCost(const Matrix<X_DIM>& x, SymmetricMatrix<X_DIM>& Qell, Matrix<X_DIM>& qell, const size_t& iter) {
	/*Qell = hessian(x, cell); 
	qell = jacobian(x, cell) - Qell*x;*/
	Qell = Q;
	qell = -(Q*xGoal);
}

// Continuous-time dynamics \dot{x} = f(x,u)
inline Matrix<X_DIM> f(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u) {
	Matrix<X_DIM> xDot;

	// Differential-drive
	xDot[0] = 0.5*(u[0] + u[1])*cos(x[2]);
	xDot[1] = 0.5*(u[0] + u[1])*sin(x[2]);
	xDot[2] = (u[1] - u[0])/2.58;

	return xDot;
}

// Discrete-time dynamics x_{t+1} = g(x_t, u_t)
inline Matrix<X_DIM> g(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u) {
	Matrix<X_DIM> k1 = f(x, u);
	Matrix<X_DIM> k2 = f(x + 0.5*dt*k1, u);
	Matrix<X_DIM> k3 = f(x + 0.5*dt*k2, u);
	Matrix<X_DIM> k4 = f(x + dt*k3, u);
	return x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

// Discrete-time inverse dynamics x_t = \bar{g}(x_{t+1}, u_t)
inline Matrix<X_DIM> gBar(const Matrix<X_DIM>& x, const Matrix<U_DIM>& u) {
	Matrix<X_DIM> k1 = f(x, u);
	Matrix<X_DIM> k2 = f(x - 0.5*dt*k1, u);
	Matrix<X_DIM> k3 = f(x - 0.5*dt*k2, u);
	Matrix<X_DIM> k4 = f(x - dt*k3, u);
	return x - (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

int main(int argc, char* argv[])
{
	// Control parameters
	ell = 150;
	dt = 1.0/6.0;

	Q = 50*identity<X_DIM>();
	rotCost = 0.4;

	xGoal = zero<X_DIM>();
	xGoal[0] = 0;
	xGoal[1] = 25;
	xGoal[2] = M_PI; 

	xStart = zero<X_DIM>();
	xStart[0] = 0;
	xStart[1] = -25;
	xStart[2] = M_PI;

	R = 0.6*identity<U_DIM>();

	uNominal[0] = 2.5; 
	uNominal[1] = 2.5; 

	obstacleFactor = 1.0;
	scaleFactor = 1.0;

	// Environment settings
	robotRadius = 3.35/2.0;
	bottomLeft[0] = -20; bottomLeft[1] = -30;
	topRight[0] = 20; topRight[1] = 30;

	Obstacle obstacle;
	obstacle.pos[0] = 0; obstacle.pos[1] = -13.5;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = 10; obstacle.pos[1] = -5;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = -9.5; obstacle.pos[1] = -5;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = -2; obstacle.pos[1] = 3;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = 8; obstacle.pos[1] = 7;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = 11; obstacle.pos[1] = 20;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = -12; obstacle.pos[1] = 8;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = -11; obstacle.pos[1] = 21;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = -1; obstacle.pos[1] = 16;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = -11; obstacle.pos[1] = -19;
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	obstacle.pos[0] = 10 + sqrt(2.0); obstacle.pos[1] = -15 - sqrt(2.0);
	obstacle.radius = 2.0;
	obstacles.push_back(obstacle);

	// Run iLQR and Extended LQR
	std::vector<Matrix<U_DIM, X_DIM> > L;
	std::vector<Matrix<U_DIM> > l;

	size_t numIter;

	clock_t beginTime = clock();

	iterativeLQR(ell, xStart, zero<U_DIM>(), g, quadratizeFinalCost, cell, quadratizeCost, ct, L, l, true, numIter);

	clock_t endTime = clock();
	std::cerr << "Iterative LQR: NumIter: " << numIter << " Time: " << (endTime - beginTime) / (double) CLOCKS_PER_SEC << std::endl;

	L.clear();
	l.clear();

	beginTime = clock();

	extendedLQR(ell, xStart, zero<U_DIM>(), g, gBar, quadratizeFinalCost, cell, quadratizeCost, ct, L, l, true, numIter);

	endTime = clock();
	std::cerr << "Extended LQR: NumIter: " << numIter << " Time: " << (endTime - beginTime) / (double) CLOCKS_PER_SEC << std::endl;

	// Execute control policy
	Matrix<X_DIM> x = xStart;
	for (size_t t = 0; t < ell; ++t) {
		std::cerr << t << ": " << ~x;
		x = g(x, L[t]*x + l[t]);
	}
	std::cerr << ell << ": " << ~x;

	int k;
	std::cin >> k;

	return 0;
}

