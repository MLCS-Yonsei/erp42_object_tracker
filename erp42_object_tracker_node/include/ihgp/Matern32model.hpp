#ifndef MAT32_H_
#define MAT32_H_

#include <vector>
#include <cmath>
#include <Eigen/Core>

class Matern32model 
{
public:
    
    /* Hyperparameters */
    double magnSigma2;
    double lengthScale;
    double sigma2;
    
    /* Model matrices */
    Eigen::MatrixXd F;
    Eigen::MatrixXd Pinf;
    Eigen::MatrixXd H;
    double R;
    
    /* Derivatives */
    std::vector< Eigen::MatrixXd > dF;
    std::vector< Eigen::MatrixXd > dPinf;
    std::vector< double > dR;
    
    /* Constructor */
    Matern32model();
    
    /* Update hyperparameters */
    void setMagnSigma2(const double &val);
    void setLengthScale(const double &val);
    void setSigma2(const double &val);
    
    /* Get model matrices */
    Eigen::MatrixXd getF();
    Eigen::MatrixXd getPinf();
    Eigen::MatrixXd getH();
    double getR();
    std::vector< Eigen::MatrixXd > getdF();
    std::vector< Eigen::MatrixXd > getdPinf();
    std::vector< double > getdR();
    
    /* Get hyperparameters */
    double getMagnSigma2();
    double getLengthScale();
    double getSigma2();
    
private:
    
    void updateModel();
    
};

#endif
