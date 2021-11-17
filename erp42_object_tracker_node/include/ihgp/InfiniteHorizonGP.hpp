#ifndef IHGP_H_
#define IHGP_H_

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <unsupported/Eigen/MatrixFunctions>

class InfiniteHorizonGP {

public:
    
    /* Constructor */
    InfiniteHorizonGP(const double dt, const Eigen::MatrixXd &F, const Eigen::MatrixXd &HH, const Eigen::MatrixXd &Pinf, const double &R, const std::vector<Eigen::MatrixXd> &dF, const std::vector<Eigen::MatrixXd> &dPinf, const std::vector<double> &dR);

    /* Destructor */
    ~InfiniteHorizonGP();

    /* Constructor(initialize) */
    
    void init_step();

    /* Pass new data point */
    void update(const double &y);
    
    /* Get log marginal likelihood */
    double getLik();
    
    /* Get marginal likelihood gradient */
    Eigen::VectorXd getLikDeriv();

    /* Get the marginal posterior mean */
    std::vector<double> getEft();

    /* Get the marginal posterior variance */
    double getVarft();

private:

    /* Constants for DARE solver */
    static const double DARE_EPS;
    static const int DARE_MAXIT;
    
    /* Model matrices */
    Eigen::MatrixXd A;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd H;
    Eigen::VectorXd HA;
    Eigen::VectorXd K;
    Eigen::MatrixXd AKHA;
    double Rc;
    double S;
    
    /* Model derivatives */
    int nparam;
    Eigen::VectorXd* HdA;
    Eigen::VectorXd* dK;
    Eigen::MatrixXd* dAKHA;
    double* dS;
    
    /* State mean and covariance */
    Eigen::VectorXd m;
    Eigen::MatrixXd P;
    Eigen::MatrixXd PP_update;
    Eigen::MatrixXd PF;
    Eigen::VectorXd* dm;
    std::vector<Eigen::VectorXd> MF;
    
    /* Energy and gradient */
    double edata;
    Eigen::VectorXd gdata;

    /* Custom discrete Riccati equation solver */
    Eigen::MatrixXd DARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q, const double &R);
    
};

#endif
