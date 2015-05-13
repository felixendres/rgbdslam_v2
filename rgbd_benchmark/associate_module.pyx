#!/usr/bin/python
#
# Requirements: 
# sudo apt-get install python-argparse

import argparse
import sys
import os
import numpy


def read_file_list(filename):
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)

def associate(first_dict, second_dict, double offset, double max_difference):
    first_keys = first_dict.keys()
    second_keys = second_dict.keys()
    cdef double t1, t2
    potential_matches = [(abs(t1 - (t2 + offset)), t1, t2) 
                         for t1 in first_keys 
                         for t2 in second_keys 
                         if abs(t1 - (t2 + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, t1, t2 in potential_matches:
        if t1 in first_keys and t2 in second_keys:
            first_keys.remove(t1)
            second_keys.remove(t2)
            matches.append((t1, t2))
    
    matches.sort()
    return matches

def main():    
    # parse command line
    print "helloe"
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    parser.add_argument('--first_only', help='only output associated lines from first file', action='store_true')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    args = parser.parse_args()

    first_dict = read_file_list(args.first_file)
    second_dict = read_file_list(args.second_file)

    matches = associate(first_dict, second_dict,float(args.offset),float(args.max_difference))    

    if args.first_only:
        for a,b in matches:
            print("%f %s"%(a," ".join(first_dict[a])))
    else:
        for a,b in matches:
            print("%f %s %f %s"%(a," ".join(first_dict[a]),b-float(args.offset)," ".join(second_dict[b])))
            

if __name__ == '__main__':
  main()
