#!/usr/bin/python
import argparse
import random
import matplotlib
from matplotlib.pyplot import figure, text, show, xticks, xlabel, ylabel, gcf, close
from numpy import arange, pi, cos, sin, pi, asarray, median


#If new parameter found, append a list for its values to the error and duration collections
def check_for_new_value(value, parameter_value_list, error_value_list, duration_list):
      if value not in parameter_value_list:
        parameter_value_list.append(value);
        error_value_list.append([])
        duration_list.append([])


def significance_test(a, b):
  #from scipy import stats
  from scipy.stats import wilcoxon
  pre = asarray(a)
  post = asarray(b)
  #How to make sure that a[i] is related to b[i]? Sortieren der Datei?

  #(t,p) = stats.ttest_rel(a,b)
  z_statistic, p_value = wilcoxon(post - pre)
  if global_verbose:
    print "paired wilcoxon-test", p_value, z_statistic
  return p_value

def myboxplotpdfpage(pdf_pages, data, labelx, ticks, labely, title = None, do_significance_test=True):
  if len(ticks) >= 1:
    myboxplot(data, labelx, ticks, labely, title, do_significance_test)
    if pdf_pages != None:
      pdf_pages.savefig()
      close() # close figure, so it is not show()'ed
  else:
    print "Not generating plot", repr(title), "because it has only one category"

def myboxplot(data, labelx, ticks, labely, title = None, do_significance_test=True):
  font = {'family' : 'sans serif', 'weight' : 'normal', 'size'   : 12}
  matplotlib.rc('font', **font)

  import copy
  ticks = copy.deepcopy(ticks)
  for (dataset, tick) in zip(data, ticks):
    if global_verbose:
      print "["+str(tick)+": "+str(len(dataset))+"] ",
  print
  fig = figure()
  if title == None:
    title = labelx + " vs. " + labely
  fig.canvas.set_window_title(title)
  ax0 = fig.add_subplot(111)
  #ax0.boxplot(data,sym='', widths=0.9, whis=1.9)# 95% confidence interval
  bp = ax0.boxplot(data,sym='', widths=0.9, whis=1.9, patch_artist=True)# 95% confidence interval, no outliers
  import pylab
  #print bp.keys()
  boxlines = bp['boxes']
  for line in boxlines:
      line.set_color('#AABBFF')
      #line.set_color('r')
      #print line.get_linewidth()
  medlines = bp['medians']
  for line in medlines:
    line.set_color('r')
    line.set_linewidth(2)
  whiskers = bp['whiskers']
  for line in whiskers:
    line.set_color('k')
    line.set_linewidth(2)
  ax0.set_title(title)
  xticks( range(1,1+len(ticks)), ticks )
  ylabel(labely)
  xlabel(labelx)

  try:  #Depends on asarray, which only works for same amount of data
    if global_verbose:
      print labely
      for category, name in zip(data, ticks):
        print "Median of", str(name)+":", median( asarray(category)), ", Min:", min( asarray(category)), ", Max:", max( asarray(category)), ", Number of Values: ", len(category)
  
      #print "Asarray:", asarray(data)
      print "Overall Median:", median( asarray(data))
      print
    ndata = asarray(data)
    mean_over_categories = ndata.mean(1)
    std_over_categories = ndata.std(1, ddof=1)
    #fig = figure()
    #fig.canvas.set_window_title(title + "(Bar)")
    #ax0 = fig.add_subplot(111)
    ax0.errorbar(arange(1,1+len(mean_over_categories)), mean_over_categories, fmt='mD')
    #ax0.errorbar(arange(1,1+len(mean_over_categories)), mean_over_categories, fmt='mD', yerr = 2* std_over_categories, ecolor="#CCCC00", linewidth=10, capsize=20)
    fig.suptitle("Each category contains " + str(len(data[0])) + " samples", y=0.03, x=0.4)
    if do_significance_test:
      text2 = "Significance Level\n(p-value of paired\nWilcoxon Test):\n"
      for indx1 in range(len(data)-1):
        for indx2 in range(indx1+1,len(data)):
          text2 += "%s vs %s: %.3f\n" % (ticks[indx1], ticks[indx2], significance_test(data[indx1], data[indx2]))
      fig.suptitle(text2, x= 0.72, y=0.50, horizontalalignment="left")#, fontsize="large")
  except ValueError:
    print "Warning: Categories have varying sample size"
    for i in range(len(ticks)):
      ticks[i] = str(ticks[i]) + "\n("+str(len(data[i]))+" Smpls)"
  except TypeError:
    print "Warning: Categories have varying sample size"
    for i in range(len(ticks)):
      ticks[i] = str(ticks[i]) + "\n("+str(len(data[i]))+" Smpls)"

  ax0.set_title(title)
  #ax0.set_ticks_position('both') # ticks left and right
  #if len(data) > 1:
    #ax0.minorticks_on()
  xticks( range(1,1+ len(ticks)), ticks)#, fontsize = 'large' )
  ylabel(labely)
  matplotlib.rc('text', usetex=True)
  xlabel(labelx)
  matplotlib.rc('text', usetex=False)



  #Generate legend manualy
  from matplotlib.lines import Line2D
  #With mean and std.dev. #rectangles = [ Line2D((0,0), (1, 0), marker=markr, c=color, linestyle=ls, lw=2) for (color, ls, markr) in zip(['m', '#CCCC00', 'r', '#AABBFF', 'k' ], [' ', '-', '-', '-' , '-'], ['D', None, None, None, None])]
  rectangles = [ Line2D((0,0), (1, 0), marker=None, c=color, linestyle=ls, linewidth=lw) for (color, ls, lw) in zip(['r', '#AABBFF', 'k' ], ['-', '-' , '--'], [2, 10, 2])]
  #rectangles = [ Line2D((0,0), (1, 0), marker=None, c=color, linestyle='-', lw=2) for color in ['r', '#AABBFF', 'k' ]]

  from matplotlib.font_manager import FontProperties
  fontP = FontProperties()
  fontP.set_size('medium')
  
  #With mean and std.dev. #
  ax0.legend(rectangles, ["Median", "50% Qnt.", "95% Qnt."], loc='upper left', bbox_to_anchor=(1.0, 1.0), fancybox=True, shadow=True, ncol=1, prop = fontP)
  #ax0.legend(rectangles, ["Median", "50% Qnt.", "95% Qnt."], loc='upper right', bbox_to_anchor=(0.38, 1.05), fancybox=True, shadow=True, ncol=1, prop = fontP)
  #Without mean and std.dev. # ax0.legend(rectangles, ["Median", "$\pm$25% Quart.", "$\pm$47.5% Quart."], loc='upper left', bbox_to_anchor=(1.0, 1.0), fancybox=True, shadow=True, ncol=1, prop = fontP)
  fig.subplots_adjust(bottom=0.2, right=0.78)
  #fig.subplots_adjust(left=0.15, bottom=0.17, right=0.7)



  


# Create three plots that show all results
def overviews(y, y_duration, x1, x2, x3, x4, c1, feature_types, sequence_names):
  fig = figure()
  import itertools
  chain = itertools.chain(*y_duration)
  #ndata = asarray(y_duration)
  maxy_duration = max(chain) #ndata.max()
  #ndata = asarray(y)
  chain = itertools.chain(*y)
  maxy = max(chain) #ndata.max()

  from matplotlib import colors
  manycolors = ['black', 'green', 'blue', 'red', 'cyan', 'yellow'];
  colorlist = manycolors[0:len(feature_types)]
  cmap = colors.ListedColormap(colorlist)

  #Generate legend manualy
  from matplotlib.patches import Rectangle
  rectangles = [ Rectangle((0,0), 1, 1, fc=color) for color in colorlist ]

  marker = ['^', 'v', 's', 'o', 'h', '>', '<', 'v', 's', 'o', 'h', '>', '<']

  ax0 = fig.add_subplot(211)
  for i in range(0,len(x1)):
    scat = ax0.scatter(x1[i],y[i],65,c1[i],marker=marker[i], edgecolors='none', cmap=cmap)
  mytitle = "Minor Separation: Max Feature Count"
  ax0.set_title(mytitle)
  fig.canvas.set_window_title(mytitle)
  ax0.set_ylim((0,maxy))
  ax0.set_xlim((-.5,len(sequence_names)))
  ylabel("RMSE (m)")
  xticks( range(len(sequence_names)), sequence_names )

  ax1 = fig.add_subplot(212)
  for i in range(0,len(x1)):
    scat = ax1.scatter(x1[i],y_duration[i],65,c1[i],marker=marker[i], edgecolors='none', cmap=cmap)

  ax1.set_title(mytitle)
  ax1.set_ylim((0,maxy_duration))
  ax1.set_xlim((-.5,len(sequence_names)))
  ylabel("Processing Time per Frame (s)")

  fig.legend(rectangles, feature_types)
  xticks( range(len(sequence_names)), sequence_names )
  fig.suptitle("Up-Pointing Triangle: No Observation Model, Down-Pointing Triangle: With Observation Model", y=0.04)



  fig = figure()
  ax2 = fig.add_subplot(111)
  for i in range(0,len(x2)):
    scat = ax2.scatter(x2[i],y[i],65,c1[i],marker=marker[i], edgecolors='none', cmap=cmap)
  xticks( range(len(sequence_names)), sequence_names )
  ylabel("RMSE (m)")
  fig.legend(rectangles, feature_types)
  ax2.set_title("Minor Separation: Connectivity Values")
  ax2.set_ylim((0,maxy))
  ax2.set_xlim((-.5,len(sequence_names)))
  
  #ax3 = fig.add_subplot(212)
  #for i in range(0,len(x3)):
  #  scat = ax3.scatter(x3[i],y[i],65,c1[i],marker=marker[i], edgecolors='none', cmap=cmap)
  #xticks( range(len(sequence_names)), sequence_names )
  #ylabel("RMSE (m)")
  #fig.legend(rectangles, feature_types)
  #ax3.set_title("Minor Separation: Minimal Nearest Neighbour Distance")
  #ax3.set_ylim((0,maxy))
  #ax3.set_xlim((-0.5,len(sequence_names)))
  #fig.suptitle("Up-Pointing Triangle: No Observation Model, Down-Pointing Triangle: With Observation Model", y=0.04)

  fig = figure()
  ax4 = fig.add_subplot(111)
  for i in range(0,len(x3)):
    scat = ax4.scatter(x4[i],y[i],65,c1[i],marker=marker[i], edgecolors='none', cmap=cmap)
  xticks( range(len(sequence_names)), sequence_names )
  ylabel("RMSE (m)")
  fig.legend(rectangles, feature_types)
  ax4.set_title("Minor Separation: Observation Model On(right)/Off(left)")
  ax4.set_ylim((0,maxy))
  ax4.set_xlim((-0.5,len(sequence_names)))
  fig.suptitle("Up-Pointing Triangle: Pose Graph, Down-Pointing Triangle: Pose+Landmark Graph", y=0.04)








if __name__ == '__main__':
  # parse command line
  parser = argparse.ArgumentParser(description='''
  Generate plots from evaluation data
  ''')
  parser.add_argument('inputcsv', help='input csv file', nargs='+')
  parser.add_argument('--timings', '-t', help='plot timings', action='store_true')
  parser.add_argument('--verbose', '-v', help='plot timings', action='store_true')
  parser.add_argument('--show', '-s', help='show plots instead of saving pdf', action='store_true')
  parser.add_argument('--wilcoxon', '-w', help='Test significance between distributions using the paired wilcoxon text', action='store_true')
  args = parser.parse_args()
  #Global
  global_verbose = False
  global_verbose = args.verbose

  #Data Acquisition
  #Example line:
  #obs/eval/-1/CANDIDATES/2/RANSAC/500/SOLVER/cholmod/NN/0.5/MIN/MATCHES/010/GFTT/0600/Features/ate/evaluation/1.csv:FR1 360; absolute translational error.rmse; 0.080126; m;Duration; 34.640214762;s;Optimizer Runtime; 0.677306;s;Number of Nodes/Edges; 720;2392;
  #Extract parameters from path
  pm = {} #parameter map for convenience and code readability
  pmoffset = 2
  pm['obs_eval_type'] = 2 
  pm['connectivity_values'] = 2 + pmoffset
  pm['ransac_values'] = 4 + pmoffset
  pm['solver_values'] = 6 + pmoffset
  pm['min_nn_ratio'] = 8 + pmoffset
  pm['min_matches'] = 11 + pmoffset
  pm['feature_type'] = 12 + pmoffset
  pm['max_features'] = 13 + pmoffset
  pm['graph_type'] = 17 + pmoffset
  x1 = [] #Sequence for distinguishing max feature
  x2 = [] #Sequence for distinguishing min_matches
  x3 = [] #Sequence for distinguishing min_nn_ratio
  x4 = [] #Sequence for distinguishing min_nn_ratio
  sequence_names = []
  y_seq = [] #Error/Duration by feature type
  y_seq_duration = [] #Error/Duration by feature type

  c1 = [] #
  

  # Evaluation separated by different categories
  y = [] #Error by optimization type
  y_optimizer_duration = [] #Duration by optimization type
  y_duration = [] #Duration by optimization type
  y_duration_all = []

  graph_types = []
  y_graph = [] #Error/Duration by feature type
  y_graph_duration = [] #Error/Duration by feature type

  nn_ratio_values = []
  y_nn = [] #Error/Duration by feature type
  y_nn_duration = [] #Error/Duration by feature type

  min_matches_values = []
  y_mm = [] #Error/Duration by feature type
  y_mm_duration = [] #Error/Duration by feature type

  ransac_values = []
  y_ransac = [] #Error/Duration by feature type
  y_ransac_duration = [] #Error/Duration by feature type


  feature_types = []
  y_feat = [] #Error/Duration by feature type
  y_feat_duration = [] #Error/Duration by feature type


  max_features_values = []
  y_max_feat = [] #Error/Duration
  y_max_feat_duration = [] #Error/Duration

  obs_eval_type = []
  obs_eval_names = []
  y_obs_eval_error = [] #Error/Duration
  y_obs_eval_duration = [] #Error/Duration

  solver_values = []
  y_solver_error = [] #Error/Duration
  y_solver_duration = [] #Error/Duration

  connectivity_values = []
  y_connectivity_error = [] #Error/Duration
  y_connectivity_duration = [] #Error/Duration

  for filename in args.inputcsv:
    f = open(filename)
  try:
      for line in f:
        if line[0] == '#' or len(line) <= 1:
          continue

        cols = line.replace(".csv:", ";").split(";")
        cols[1] = str.title(cols[1])
        if feature_types == []:
          if global_verbose:
            print cols
        cols[0] = cols[0].replace(" ","\n"); #unify seperators
        params = cols[0].replace("_","/"); #unify seperators
        params = params.split("/");
        cols[1] = cols[1].replace(" ","\n"); #unify seperators
        #print cols[1]

        # For newly seen input categories, add them to the corresponding lists
        check_for_new_value(cols[1], sequence_names, y_seq, y_seq_duration)
        check_for_new_value(params[pm['feature_type']], feature_types, y_feat, y_feat_duration)
        check_for_new_value(params[pm['ransac_values']], ransac_values, y_ransac, y_ransac_duration)
        check_for_new_value(params[pm['graph_type']], graph_types, y_graph, y_graph_duration)
        check_for_new_value(params[pm['min_matches']], min_matches_values, y_mm, y_mm_duration)
        check_for_new_value(params[pm['min_nn_ratio']], nn_ratio_values, y_nn, y_nn_duration)

        if params[pm['connectivity_values']] not in connectivity_values:
          connectivity_values.append(params[pm['connectivity_values']]);
          y_connectivity_error.append([]);
          y_connectivity_duration.append([]);

        if params[pm['solver_values']] not in solver_values:
          solver_values.append(params[pm['solver_values']]);
          y_solver_error.append([]);
          y_solver_duration.append([]);

        obs_eval_value = params[pm['obs_eval_type']]
        if params[pm['obs_eval_type']] not in obs_eval_type:
          #tmp = {"Without\nObservation Model", "With\nObservation Model"]
          obs_eval_type.append(obs_eval_value);
          y_obs_eval_error.append([]);
          y_obs_eval_duration.append([]);
          y.append([])
          y_duration.append([])
          y_optimizer_duration.append([])
          x1.append([])
          x2.append([])
          x3.append([])
          x4.append([])
          c1.append([])
          obs_eval_names.append(obs_eval_value)

        if params[pm['max_features']] not in max_features_values:
          max_features_values.append(params[pm['max_features']]);
          y_max_feat.append([]);
          y_max_feat_duration.append([]);

        marker_separated_category = int(obs_eval_type.index(obs_eval_value)) # 

        #Mapping Parameters to the X axis for first plot. These are used for the overviews()
        x1[marker_separated_category].append(sequence_names.index(cols[1])  + float(params[pm['max_features']])/1200-0.5 + float(params[pm['min_matches']])/100-0.05) #major categories + separation by param
        x2[marker_separated_category].append(sequence_names.index(cols[1])  + float(params[pm['connectivity_values']])/20-0.1) #major categories + separation by param
        x3[marker_separated_category].append(sequence_names.index(cols[1])  + (float(params[pm['min_matches']])-0.65)*2) #major categories + separation by param
        x3[marker_separated_category].append(sequence_names.index(cols[1])  + (marker_separated_category/6.0 )) #major categories + separation by param
        x4[marker_separated_category].append(sequence_names.index(cols[1])  + (float(params[pm['obs_eval_type']])-0.5)/2  + float(params[pm['max_features']])/2400-0.05) #major categories + separation by param

        try:
          num_nodes = int(cols[12])
        except ValueError:
          num_nodes = 2727 
        #Result collections
        y_duration_value = float(cols[6]) / num_nodes
        y_duration[marker_separated_category].append(y_duration_value)
        y_duration_all.append(y_duration_value)
        try:
          optimizer_runtime =float(cols[9])
        except ValueError:
          optimizer_runtime = 0

        y_optimizer_duration[marker_separated_category].append(optimizer_runtime)
        y_error = float(cols[3])
        y[marker_separated_category].append(y_error) #Y values, e.g. rmse distinguished by optimization type
        y_graph[graph_types.index(params[pm['graph_type']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_graph_duration[graph_types.index(params[pm['graph_type']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        y_mm[min_matches_values.index(params[pm['min_matches']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_mm_duration[min_matches_values.index(params[pm['min_matches']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        y_nn[nn_ratio_values.index(params[pm['min_nn_ratio']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_nn_duration[nn_ratio_values.index(params[pm['min_nn_ratio']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        y_ransac[ransac_values.index(params[pm['ransac_values']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_ransac_duration[ransac_values.index(params[pm['ransac_values']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        y_feat[feature_types.index(params[pm['feature_type']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_feat_duration[feature_types.index(params[pm['feature_type']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        y_seq[sequence_names.index(cols[1])].append(y_error) #Y values, e.g. rmse, distinguished by sequence
        y_seq_duration[sequence_names.index(cols[1])].append(y_duration_value) #Y values, e.g. rmse, distinguished by sequence
        y_max_feat[max_features_values.index(params[pm['max_features']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_max_feat_duration[max_features_values.index(params[pm['max_features']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        y_obs_eval_error[obs_eval_type.index(params[pm['obs_eval_type']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_obs_eval_duration[obs_eval_type.index(params[pm['obs_eval_type']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        y_solver_error[solver_values.index(params[pm['solver_values']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_solver_duration[solver_values.index(params[pm['solver_values']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        y_connectivity_error[connectivity_values.index(params[pm['connectivity_values']])].append(y_error) #Y values, e.g. rmse, distinguished by feature type
        y_connectivity_duration[connectivity_values.index(params[pm['connectivity_values']])].append(y_duration_value) #Y values, e.g. rmse, distinguished by feature type
        c1[marker_separated_category].append(graph_types.index(params[pm['graph_type']]))
  except IndexError:
    print line
    print params
    print cols
    raise
  finally:
    if global_verbose:
      print "Last line:"
      print line
      print "Last columns:"
      print cols
      print "Last params:"
      print params
    f.close()
    #print feature_types


  if len(y) == 0:
    print "Empty File?"
  else: 
    if args.show:
      pp = None
    else:
      from matplotlib.backends.backend_pdf import PdfPages
      pp = PdfPages(args.inputcsv[0]+'.pdf')

    #myboxplot(y[1:3], "Graph Content", ["Poses Only", "Poses and\nLandmarks"], "ATE RMSE (m)")
    #myboxplot(y_optimizer_duration[1:3], "Graph Content", ["Poses Only", "Poses and\nLandmarks"], "ATE RMSE (m)")
    #myboxplot(y_duration[0:3], "Graph Content", ["Poses Only", "Poses and\nLandmarks"], "ATE RMSE (m)")

    if len(graph_types) > 1:
      myboxplotpdfpage(pp, y_graph, "Graph Pruning", graph_types, "ATE RMSE Error (m)", "Error w.r.t. Pruned Edges", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_graph_duration, "Graph Pruning", graph_types, "Processing Time per Frame (s)", "Processing Time w.r.t. Pruned Edges", do_significance_test = args.wilcoxon)

    if len(ransac_values) > 1:
      myboxplotpdfpage(pp, y_ransac, "RANSAC Iterations", ransac_values, "ATE RMSE Error (m)", "Error w.r.t. Sequence", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_ransac_duration, "RANSAC Iterations", ransac_values, "Processing Time per Frame (s)", "Processing Time w.r.t. RANSAC Iterations", do_significance_test = args.wilcoxon)

    myboxplotpdfpage(pp, y_seq, "Sequence", sequence_names, "ATE RMSE Error (m)", "Error w.r.t. Sequence", do_significance_test = args.wilcoxon)
    if args.timings:
      myboxplotpdfpage(pp, y_seq_duration, "Sequence", sequence_names, "Processing Time per Frame (s)", "Processing Time w.r.t. Sequence", do_significance_test = args.wilcoxon)

    if len(feature_types) > 1:
      myboxplotpdfpage(pp, y_feat, "Feature Types", feature_types, "ATE RMSE Error (m)", "Error w.r.t. Feature Type", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_feat_duration, "Feature Types", feature_types, "Processing Time per Frame (s)", "Processing Time w.r.t. Feature Type", do_significance_test = args.wilcoxon)

    if len(obs_eval_names) > 1:
      myboxplotpdfpage(pp, y_obs_eval_error, "Required Inlier Fraction", obs_eval_names, "ATE RMSE Error (m)",  "Error w.r.t. Usage\nof Observation Model", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_obs_eval_duration, "Required Inlier Fraction", obs_eval_names, "Processing Time per Frame (s)", "Processing Time w.r.t.\nUsage of Observation Model", do_significance_test = args.wilcoxon)

    if len(solver_values) > 1:
      myboxplotpdfpage(pp, y_solver_error, "Keypoint Detection Type", solver_values, "ATE RMSE (m)", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_solver_duration, "Keypoint Detection Type", solver_values, "Processing Time (s)", do_significance_test = args.wilcoxon)

    if len(connectivity_values) > 1:
      myboxplotpdfpage(pp, y_connectivity_error, "Frames to Compare to", [ int(val)*2+int(val)*2.5 for val in connectivity_values], "ATE RMSE (m)", "Error w.r.t.\nFrame Comparisons", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_connectivity_duration, "Frames to Compare to", [ int(val)*2+int(val)*2.5 for val in connectivity_values], "Processing Time per Frame (s)", "Processing Time w.r.t.\nFrame Comparisons", do_significance_test = args.wilcoxon)

    if len(max_features_values) > 1:
      myboxplotpdfpage(pp, y_max_feat, "Maximum Feature Count", max_features_values, "ATE RMSE (m)", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_max_feat_duration, "Maximum Feature Count", max_features_values, "Processing Time per Frame (s)", "Processing Time w.r.t.\nMaximum Feature Count", do_significance_test = args.wilcoxon)

    if len(nn_ratio_values) > 1:
      myboxplotpdfpage(pp, y_nn, "Feature Matching NN Ratio", nn_ratio_values, "ATE RMSE Error (m)", "Error w.r.t. NN Ratio", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_nn_duration, "Feature NN Ratio Matches", nn_ratio_values, "Processing Time per Frame (s)", "Processing Time w.r.t. NN Ratio", do_significance_test = args.wilcoxon)

    if len(min_matches_values) > 1:
      myboxplotpdfpage(pp, y_mm, "Feature Matching Min Matches", min_matches_values, "ATE RMSE Error (m)", "Error w.r.t. Min Matches", do_significance_test = args.wilcoxon)
      if args.timings:
        myboxplotpdfpage(pp, y_mm_duration, "Feature Matching Min Matches", min_matches_values, "Processing Time per Frame (s)", "Processing Time w.r.t. Min Matches", do_significance_test = args.wilcoxon)

    if pp != None:
      pp.close() # close pdf

    #overviews(y, y_duration, x1, x2, x3, x4, c1, graph_types, sequence_names)
    show()


