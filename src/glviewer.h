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


#ifndef GLVIEWER_H
#define GLVIEWER_H
#include <QGLWidget>
#include <QList>
#include <QPair>
#include <QMatrix4x4>
#include "parameter_server.h"
#include "renderable.h"

//!OpenGL based display of the 3d model 
class GLViewer : public QGLWidget {
    Q_OBJECT

public:
    GLViewer(QWidget *parent = 0);
    ~GLViewer();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

public Q_SLOTS:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void setStereoShift(double shift);
    void setRotationGrid(double rot_step_in_degree);
    void toggleFollowMode(bool on);
    void toggleShowEdges(bool on);
    void toggleShowIDs(bool on);
    void toggleShowGrid(bool on);
    void toggleShowTFs(bool on);
    void toggleShowPoses(bool on);
    void toggleShowClouds(bool on);
    void toggleShowOctoMap(bool on);
    void toggleShowFeatures(bool on);
    void toggleBackgroundColor(bool on);
    void toggleStereo(bool on);

    //Formerly called trough qt_gui
    void addPointCloud(pointcloud_type * pc, QMatrix4x4 transform);
    void addFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* feature_locations_3d);
    void updateTransforms(QList<QMatrix4x4>* transforms);
    void setEdges(const QList<QPair<int, int> >* edge_list);
    void deleteLastNode();
    void reset();
    void toggleTriangulation();
    void drawToPS(QString filname);
    void setRenderable(Renderable* r);
Q_SIGNALS:
    void cloudRendered(pointcloud_type *);
    void clickedPosition(float x,float y,float z);
    //void xRotationChanged(int angle);
    //void yRotationChanged(int angle);
    //void zRotationChanged(int angle);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    //!Resets to certain perspectives
    ///Double clicks onto an object part will make that part the pivot of 
    ///The camera movement. Depending on the button the position of the camera
    ///will be the most recent frame (left button) or the first frame (right button).
    ///If the background is clicked, the pivot will be the camera position itself
    void mouseDoubleClickEvent(QMouseEvent *event);
    ///Draw colored axis, scale long
    void drawAxes(float scale, float thickness = 4.0);
    //!Draw pose graph edges in opengl 
    void drawEdges();
    //!Draw size x size grid into world coordinate system
    void drawGrid();
    //!Draw a triangle in opengl 
    ///Assumes gl mode for triangles
    void drawTriangle(const point_type& p1, const point_type& p2, const point_type& p3);
    //bool startTriangleStrip(pointcloud_type const * pc, int x, int y, int w, int h, bool& flip);
    ///Compile the pointcloud to a GL Display List using GL_TRIANGLES
    void pointCloud2GLTriangleList(pointcloud_type const * pc);
    ///Compile the pointcloud to a GL Display List using GL_POINT 
    void pointCloud2GLPoints(pointcloud_type * pc);
    ///Compile the pointcloud to a GL Display List using GL_TRIANGLE_STRIPs
    void pointCloud2GLStrip(pointcloud_type * pc);
    ///Draw ellipsoids instead of points, that represent the depth std deviation of each point
    void pointCloud2GLEllipsoids(pointcloud_type * pc);
    QImage renderList(QMatrix4x4 transform, int list_id);
    ///Draw one of the clouds with the given list index
    void drawOneCloud(int i) ;
    //! Draw the scene. Xshift allows for a camera shift in x direction for the stereo view
    void drawClouds(float xshift);
    float bg_col_[4];
    ///Only switch to own context if necessary
    void makeCurrent();

    void initialPosition();
    void drawNavigationAxis(int axis_idx, float scale, QString text);
    void drawRenderable() ;

private:
    void clearAndUpdate();
    int xRot, yRot, zRot;
    float xTra, yTra, zTra;
    QPoint lastPos;
    GLenum polygon_mode;
    QList<GLuint> cloud_list_indices;
    QList<GLuint> feature_list_indices;
    QList<QPair<int, int> > edge_list_;
    QList<QMatrix4x4>* cloud_matrices;
    QMatrix4x4 viewpoint_tf_;
    //QMatrix4x4 world_tf_;//< To change the center of the world, i.e. about what are we rotating
    //!cam_pose_mat transforms the viewpoint from the origin
    ///cam_pose_mat is inverted, i.e. it should describes the transformation
    ///of the camera itself
    void setViewPoint(QMatrix4x4 cam_pose_mat);
    bool setClickedPosition(int x, int y);
    bool show_poses_;
    bool show_ids_;
    bool show_grid_;
    bool show_tfs_;
    bool show_edges_;
    bool show_clouds_;
    bool show_octomap_;
    bool show_features_;
    bool follow_mode_;
    bool stereo_;
    bool black_background_;
    int width_, height_;
    double stereo_shift_, fov_; //field of view
    double rotation_stepping_;
    QWidget* myparent;
    bool button_pressed_;
    bool non_interactive_update_;
    unsigned int fast_rendering_step_;
    Renderable* external_renderable;
};

void drawEllipsoid(float fA, float fB, float fC, const Eigen::Vector4f& p);
#endif
