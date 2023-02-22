#!/usr/bin/env python3
import os.path
import sys
import argparse
import stretch_body.hello_utils as hu
import click
from colorama import Style
import glob
hu.print_stretch_re_use()
import yaml
from datetime import datetime


parser = argparse.ArgumentParser(description='Tool to load and view the robot trace files.', )
args = parser.parse_args()

class TraceMgmt:
    """
    Manage trace data
    """
    def __init__(self):
        self.trace_directory = hu.get_stretch_directory()+'log/trace'


    def get_int(self,range):
        while True:
            result = input("Enter value in range %d to %d:"%(range[0],range[1]))
            if result.isdigit() and range[0] <= int(result) <= range[1]:
                return int(result)
            print("Error Invalid Input")

    def run_menu(self):
        segs=self.get_trace_segments()
        active_seg_id=0
        if len(segs)==0:
            click.secho('No trace data found in %s'%self.trace_directory, fg="yellow")
            return

        while True:
            print(Style.BRIGHT + '############### MENU ################' + Style.RESET_ALL)

            self.pretty_print_segments(segs, active_seg_id)
            print('Enter command. (q to quit)')
            print('s: set active segment ID')
            print('f: print available fields')
            print('v: plot bus voltage')
            print('-------------------------------------')
            try:
                r = input()
                if r == 'q' or r == 'Q':
                    return
                elif r == 's':
                    active_seg_id=self.get_int([0,len(segs)-1])
                else:
                    print('Invalid entry')
            except(TypeError, ValueError):
                print('Invalid entry')

    def pretty_print_fields(self,segs):
        click.secho('---- Trace Segments ----', fg="green")
        for i in range(len(segs)):
            dt_start = datetime.fromtimestamp(segs[i]['ts_start'])
            dt_end = datetime.fromtimestamp(segs[i]['ts_end'])
            print('%d: \t Duration: %f \t Start: %s \t End: %s'%(i, segs[i]['ts_end']-segs[i]['ts_start'],dt_start,dt_end))


    def get_segment_dict(self,seg):
        """
        Assemble a dict with each field as a list of values
        """
        data={}
        data['ts_start']=seg['ts_start']
        data['ts_end']=seg['ts_end']
        data['samples']=None
        for f in seg['filename']:
            with open(f, 'r') as s:
                trace=yaml.load(s,Loader=yaml.FullLoader)
                x = list(trace.keys())
                x.sort() #Chronological list of samples eg [trace_0000, trace_0001,...], oldest ID 0
                if data['samples'] is None:
                    data['samples']

    def get_trace_segments(self):
        #Return a list of all segments found in the trace dir
        #ID 0 is most recent, ID -1 is oldes
        #A segment is a timespan of samples (that may bridge multiple files)
        #Segment boundaries are defined as having a time gap of over 0.2s
        segments=[]
        all_files=self.get_all_files_sorted()
        seg_thresh_s=0.2
        seg_curr={}
        for f in all_files:
            with open(f, 'r') as s:
                trace=yaml.load(s,Loader=yaml.FullLoader)
                x=list(trace.keys())
                x.sort() #Chronological list of samples eg [trace_0000, trace_0001,...], oldest ID 0
                sub_seg = {'ts_start':trace[x[0]]['timestamp'],'ts_end':trace[x[-1]]['timestamp'],'filename':[f]}
                if len(segments)==0:
                    segments=[sub_seg]
                else:
                    #See if this extends current segment or is a new one
                    ts_last = segments[-1]['ts_end']
                    if sub_seg['ts_start']-ts_last<seg_thresh_s:
                        segments[-1]['ts_end']=sub_seg['ts_end']
                        segments[-1]['filename'].append(f)
                    else:
                        segments.append(sub_seg)
        segments.reverse()
        return segments

    def pretty_print_segments(self,segs,active_seg_id):
        click.secho('---- Trace Segments ----', fg="white")
        for i in range(len(segs)):
            dt_start = datetime.fromtimestamp(segs[i]['ts_start'])
            dt_end = datetime.fromtimestamp(segs[i]['ts_end'])
            ln='%d: \t Duration: %f \t Start: %s \t End: %s'%(i, segs[i]['ts_end']-segs[i]['ts_start'],dt_start,dt_end)
            if i==active_seg_id:
                click.secho(ln, fg="green")
            else:
                click.secho(ln, fg="white")

    def get_all_files_sorted(self):
        # Retrieve sorted list of all trace files
        # Sorted chronological (id 0 is oldest)
        try:
            all_files = glob.glob(self.trace_directory + '/*.yaml')
            all_files.sort()
            return all_files
        except:
            return []


mgmt=TraceMgmt()
mgmt.run_menu()