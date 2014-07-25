#ifndef RGBDSLAM_COVARIANCE_ESTIMATION_H
#define RGBDSLAM_COVARIANCE_ESTIMATION_H
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "scoped_timer.h"
//Compute (inverse) covariances from edge errors, optionally weighted by inverse distance to given measurement
typedef Eigen::Matrix<double,6,6> InfoMatType;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;
typedef std::set<g2o::HyperGraph::Edge*>::iterator  EdgeSetIt;

InfoMatType computeEmpiricalInformationMatrix(const Matrix6Xd& measurements,
                                              const Matrix6Xd& errors,
                                              const Vector6d& currentMeasurement,
                                              const Vector6d& stdDeviation);

///Convert Isometry to vector6d of which the last 3 parameters represent the
//rotation by a unit axis scaled with the angle
//Vector6d toVectorScaledAxis(const Eigen::Isometry3d& tf);
///Convert Isometry to vector6d of which the last 3 parameters represent the
//rotation by a unit axis scaled with the angle
//Vector6d toVectorScaledAxis(Vector6d vecMQT);
//g2o::EdgeSE3::InformationType computeEmpiricalInformationMatrix(const Eigen::Matrix<double, 6, Eigen::Dynamic>& measurements,
//                                                                const Eigen::Matrix<double, 6, Eigen::Dynamic>& errors,
//                                                                const g2o::EdgeSE3::Measurement& similarToThisTransformation);


//Fill matrix columns with VectorMQTs from edge measurements
//void edgesToMeasurementMatrix(const g2o::SparseOptimizer* optimizer, Matrix6Xd& output);
template <class EDGETYPE>
void edgesToMeasurementMatrix(const EdgeSet& edges, Matrix6Xd& output)
{
  ScopedTimer s(__FUNCTION__);
  output.resize(Eigen::NoChange, edges.size());
  
  EdgeSetIt it = edges.begin();
  for(int i = 0; it != edges.end(); ++i, ++it)
  {
    EDGETYPE* edge = dynamic_cast<EDGETYPE*>( *it );
    if(edge) {
      output.col(i) = g2o::internal::toVectorMQT(edge->measurement());
    } else {
      output.col(i) = Vector6d::Zero();
      std::cerr << "Encountered invalid edge: " << i << std::endl;
    }
  }
}
//Fill matrix columns with VectorMQTs from edge errors
//void edgesToErrorMatrix(const g2o::SparseOptimizer* optimizer, Matrix6Xd& output);
template <class EDGETYPE>
void edgesToErrorMatrix(const EdgeSet& edges, Matrix6Xd& output)
{
  ScopedTimer s(__FUNCTION__);
  output.resize(Eigen::NoChange, edges.size());
  
  EdgeSetIt it = edges.begin();
  for(int i = 0; it != edges.end(); ++i, ++it)
  {
    EDGETYPE* edge = dynamic_cast<EDGETYPE*>( *it );
    if(edge) {
      output.col(i) = edge->error();
    } else {
      output.col(i) = Vector6d::Zero();
      std::cerr << "Encountered invalid edge: " << i << std::endl;
    }
  }
}
#endif
