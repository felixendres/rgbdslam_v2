#include "covariance_estimation.h"
#include <iostream>
#include <fstream>


Vector6d toVectorScaledAxis(Vector6d vecMQT)
{
    return vecMQT;
    /*
    double sqr_nrm = vecMQT.tail<3>().squaredNorm();
    if(!(sqr_nrm > 0.0 && sqr_nrm <= 1.0)){ //No (valid) rotation
      vecMQT.tail<3>().setZero();
      //std::cerr << "\nSquared Norm is " << sqr_nrm << ", setting rotational part to zero\n";
    } else {
      double qw = std::sqrt(1 - sqr_nrm);
      double alpha = 2*std::acos(qw);
      double factor = alpha / std::sqrt(sqr_nrm); //to normalize axis and multiply with angle
      vecMQT.tail<3>() = vecMQT.tail<3>() * factor;
      //std::cout << "VecScAx: " << vecMQT.transpose() << "\t";
      //std::cout << "sqr_nrm: " << sqr_nrm << "\t";
      //std::cout << "qw:      " << qw << "\t";
      //std::cout << "alpha:   " << alpha << "\n";
    }
    return vecMQT;
    */
}

Vector6d toVectorScaledAxis(const Eigen::Isometry3d& tf)
{
    Vector6d vecMQT = g2o::internal::toVectorMQT(tf);
    return toVectorScaledAxis(vecMQT);
}

inline Vector6d absManhattanDistance(const Eigen::Isometry3d& tf1,
                                          const Eigen::Isometry3d& tf2)
{
  return (toVectorScaledAxis(tf1) - toVectorScaledAxis(tf2)).cwiseAbs();
}


InfoMatType computeEmpiricalInformationMatrix(const Matrix6Xd& measurements,
                                              const Matrix6Xd& errors,
                                              const Vector6d& currentMeasurement,
                                              const Vector6d& stdDeviation)
{
  ScopedTimer s(__FUNCTION__);
  //g2o::EdgeSE3::InformationType covarianceMatrix;
  //This function assumes the DOFs to be independent and therefore works
  //with vectors that represent the diagonal matrices (covariances)
  Vector6d covarianceMatrixDiag = Vector6d::Zero();
  Vector6d sum_of_weights = Vector6d::Zero();

  //Precomputation of weights and weighted errors
  Matrix6Xd distanceMat = (measurements.colwise() - currentMeasurement).cwiseAbs();
  //Vector6d stdDistance = distanceMat.rowwise().mean(); //Now use stddeviation from global mean, not from this measurement -> outlier transformation should get lower weights for variance of other measurements and therefore more or less speak for themself
  distanceMat = stdDeviation.cwiseInverse().asDiagonal() * distanceMat; ///FIXME: Why are the results worse than without inverse, which is wrong?
  distanceMat = distanceMat.cwiseProduct(distanceMat);

  size_t size = measurements.cols();
  Vector6d weight, weightedError;
  for(size_t i = 0; i < size; ++i)
  {
    for(size_t r = 0; r < 6; ++r){ //Weighting by similarity per dimension
      weight(r) = exp(-0.5*distanceMat(r,i)); // Gaussian model, with precomputed exponent
    }
    weightedError = weight.cwiseProduct(errors.col(i));
    covarianceMatrixDiag += weightedError.cwiseProduct(weightedError);
    sum_of_weights += weight;
  }

  //Note: Empirical covariance should be computed with division by N-1.
  //Since each of the N components is weighted, an open question is how much is the "-1"
  covarianceMatrixDiag = covarianceMatrixDiag.cwiseQuotient(sum_of_weights); 
  InfoMatType informationMatrix = covarianceMatrixDiag.cwiseInverse().asDiagonal();
  
  return informationMatrix;
}

