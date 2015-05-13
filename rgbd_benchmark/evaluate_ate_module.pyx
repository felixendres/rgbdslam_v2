#!/usr/bin/python
#
# Requirements: 
# sudo apt-get install python-argparse

import sys
import numpy
import argparse
import pyximport; pyximport.install()
import associate_module as associate

def align_first(model,data):
    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = numpy.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh
    print data.shape
    trans = data[:,0] - model[:,0]
    
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error

def align(model,data):
    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)
    
    W = numpy.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh
    trans = data.mean(1) - rot * model.mean(1)
    
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data
    
    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error

def plot_traj3d(ax,stamps,traj,style,color,label, linewidth):
    from matplotlib import cm
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    z = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 5*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
            z.append(traj[i][2])
        elif len(x)>0:
            ax.plot(x,y, style,zs=z,color=color,label=label, linewidth=linewidth)
            label=""
            x=[]
            y=[]
            z=[]
        last= stamps[i]
    if len(x)>0:
      ax.plot(x,y,style, zs=z,color=color,label=label, linewidth=linewidth)
      ax.scatter(x,y,zs=z,c=color,label=label, marker='o', cmap=cm.jet)
            
def plot_traj(ax,stamps,traj,style,color,label, linewidth):
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 5*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label, linewidth=linewidth)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label, linewidth=linewidth)
            
def plot2d(rmse, first_stamps, first_xyz_full, second_stamps, second_xyz_full_aligned, matches, first_xyz, second_xyz_aligned, filename):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  import matplotlib.pylab as pylab
  from matplotlib.patches import Ellipse
  font = {'family' : 'serif', 'weight' : 'normal', 'size'   : 16}
  matplotlib.rc('font', **font)

  fig = plt.figure(figsize=(10,10))
  ax = fig.add_subplot(111)
  plt.title("ATE RMSE: %0.2f m"%(rmse))
  plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","Ground Truth", linewidth=2)
  plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"green","Estimate", linewidth=2)

  label="Difference"
  i = 0
  for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
      i+=1
      if i % 10 == 0:
        ax.plot([x1,x2],[y1,y2],'-',color="red",label=label, linewidth=1)
        label=""
      
  ax.legend(loc="upper right")
      
  ax.set_xlabel('x [m]')
  ax.set_ylabel('y [m]')
  fig.subplots_adjust(left=0.1, bottom=0.1, right=0.98, top=0.95)
  plt.savefig(filename,dpi=300)

def plot3d(rmse, first_stamps, first_xyz_full, second_stamps, second_xyz_full_aligned, matches, first_xyz, second_xyz_aligned):
  import matplotlib
  matplotlib.use('Qt4Agg')
  from mpl_toolkits.mplot3d import Axes3D
  import matplotlib.pyplot as plt
  import matplotlib.pylab as pylab
  from matplotlib.patches import Ellipse
  font = {'family' : 'serif', 'weight' : 'normal', 'size'   : 16}
  matplotlib.rc('font', **font)

  fig = plt.figure(figsize=(10,10))
  ax = fig.gca(projection='3d')
  plt.title("ATE RMSE: %0.2f m"%(rmse))
  plot_traj3d(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","Ground Truth", linewidth=2)
  plot_traj3d(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"green","Estimate", linewidth=2)

  label="Difference"
  i = 0
  for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
    i+=1
    if i % 30 == 0:
      ax.plot([x1,x2],[y1,y2],zs=[z1,z2],color="red",label=label, linewidth=1)
      label=""

  ax.legend(loc="upper right")

  ax.set_xlabel('x [m]')
  ax.set_ylabel('y [m]')
  ax.set_zlabel('z [m]')
  plt.show()


def main():
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image. Not compatible with --plot3d')
    parser.add_argument('--plot3d', action="store_true", help='plot interactively in 3d. Not compatible with --plot')
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()

    first_list = associate.read_file_list(args.first_file)
    second_list = associate.read_file_list(args.second_file)

    matches = associate.associate(first_list, second_list,float(args.offset),float(args.max_difference))    
    if len(matches)<2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")


    first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    second_xyz = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()
    rot,trans,trans_error = align(second_xyz,first_xyz)
    #rot,trans,trans_error = align_first(second_xyz,first_xyz)
    
    second_xyz_aligned = rot * second_xyz + trans
    
    first_stamps = first_list.keys()
    first_stamps.sort()
    first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()
    
    second_stamps = second_list.keys()
    second_stamps.sort()
    second_xyz_full = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
    second_xyz_full_aligned = rot * second_xyz_full + trans
    z_coords = numpy.array(second_xyz_full_aligned[2])
    rmse = numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
    if args.verbose:
        print "Z Min: ", numpy.min(z_coords);
        print "Z Max: ", numpy.max(z_coords);
        print "Z Std: ", numpy.std(z_coords);
        print "RMSE: ", numpy.sqrt(numpy.sum(z_coords*z_coords) / z_coords.shape[1])

        print "compared_pose_pairs %d pairs"%(len(trans_error))

        print "absolute_translational_error.rmse %f m"%rmse
        print "absolute_translational_error.mean %f m"%numpy.mean(trans_error)
        print "absolute_translational_error.median %f m"%numpy.median(trans_error)
        print "absolute_translational_error.std %f m"%numpy.std(trans_error)
        print "absolute_translational_error.min %f m"%numpy.min(trans_error)
        print "absolute_translational_error.max %f m"%numpy.max(trans_error)
    else:
        print "%f"%rmse
        
    if args.save_associations:
        file = open(args.save_associations,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
        file.close()
        
    if args.save:
        file = open(args.save,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_full_aligned.transpose().A)]))
        file.close()

    if args.plot:
      if args.plot and args.plot3d:
        args.plot3d = False
        print "Plotting interactively (--plot3d) and to file (--plot <filename>) is not compatible. Plotting to file."
            
      plot2d(rmse, first_stamps, first_xyz_full, second_stamps, second_xyz_full_aligned, matches, first_xyz, second_xyz_aligned, args.plot)
        
    if args.plot3d:
      plot3d(rmse, first_stamps, first_xyz_full, second_stamps, second_xyz_full_aligned, matches, first_xyz, second_xyz_aligned)
