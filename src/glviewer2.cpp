/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "glviewer.h"

GLViewer::GLViewer(QWidget *parent) : QWidget(parent) { }
GLViewer::~GLViewer() { }
void GLViewer::initialPosition() { }
QSize GLViewer::minimumSizeHint() const { return QSize(0, 0); }
QSize GLViewer::sizeHint() const { return QSize(0, 0); }
void GLViewer::setXRotation(int angle) { } 
void GLViewer::setYRotation(int angle) { } 
void GLViewer::setZRotation(int angle) { } 
void GLViewer::setRotationGrid(double rot_step_in_degree) { } 
void GLViewer::setStereoShift(double shift) { } 
void GLViewer::initializeGL() { }
void GLViewer::drawTriangle(const point_type& p1, const point_type& p2, const point_type& p3){ }
void GLViewer::drawGrid(){ }
void GLViewer::drawAxes(float scale, float thickness){ }
void GLViewer::makeCurrent(){ }
void GLViewer::paintGL() { }
void GLViewer::drawRenderable() { }
void GLViewer::drawOneCloud(int i) { }
void GLViewer::drawClouds(float xshift) { }
void GLViewer::resizeGL(int width, int height) { }
void GLViewer::mouseDoubleClickEvent(QMouseEvent *event) { }
void GLViewer::toggleStereo(bool flag){ }
void GLViewer::toggleBackgroundColor(bool flag){ }
void GLViewer::toggleShowFeatures(bool flag){ }
void GLViewer::toggleShowOctoMap(bool flag){ }
void GLViewer::toggleShowClouds(bool flag){ }
void GLViewer::toggleShowTFs(bool flag){ }
void GLViewer::toggleShowGrid(bool flag){ }
void GLViewer::toggleShowIDs(bool flag){ }
void GLViewer::toggleShowEdges(bool flag){ }
void GLViewer::toggleShowPoses(bool flag){ }
void GLViewer::toggleFollowMode(bool flag){ }
void GLViewer::mouseReleaseEvent(QMouseEvent *event) { }
void GLViewer::mousePressEvent(QMouseEvent *event) { } 
void GLViewer::wheelEvent(QWheelEvent *event) { }
void GLViewer::mouseMoveEvent(QMouseEvent *event) { } 
void GLViewer::updateTransforms(QList<QMatrix4x4>* transforms){ }
void GLViewer::addPointCloud(pointcloud_type * pc, QMatrix4x4 transform){ }
void GLViewer::addFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* feature_locations_3d) { }
void GLViewer::pointCloud2GLStrip(pointcloud_type * pc){ }
void GLViewer::deleteLastNode(){ }
void GLViewer::pointCloud2GLEllipsoids(pointcloud_type * pc){ }
void GLViewer::pointCloud2GLPoints(pointcloud_type * pc){ }
void GLViewer::pointCloud2GLTriangleList(pointcloud_type const * pc){ }
void GLViewer::reset(){ }
QImage GLViewer::renderList(QMatrix4x4 transform, int list_id){ return QImage(); }
void GLViewer::setEdges(const QList<QPair<int, int> >* edge_list){ } 
void GLViewer::drawEdges(){ } 
void GLViewer::setViewPoint(QMatrix4x4 new_vp){ } 
bool GLViewer::setClickedPosition(int x, int y) { } 
void GLViewer::toggleTriangulation() { } 
void GLViewer::drawToPS(QString filename){ }
void GLViewer::clearAndUpdate(){ }
void GLViewer::drawNavigationAxis(int axis_idx, float scale, QString text){ }
void GLViewer::setRenderable(Renderable* r){ }

