#!/usr/bin/env python3
import argparse
import stretch_body.hello_utils as hu
import click
from colorama import Style
import glob
hu.print_stretch_re_use()
import yaml
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description='Tool to load and view the robot trace files.', )
args = parser.parse_args()

class TraceMgmt:
    """
    Manage trace data
    """
    def __init__(self):
        self.trace_directory = hu.get_stretch_directory()+'log/trace'


    def get_int(self,range,msg='value'):
        while True:
            result = input("Enter %s (range %d to %d):  "%(msg,range[0],range[1]))
            if result.isdigit() and range[0] <= int(result) <= range[1]:
                return int(result)
            print("Error Invalid Input")

    def run_menu(self):
        print('Loading trace data. This will take a minute...')
        print('')
        segs=self.get_segments()
        if len(segs)==0:
            click.secho('No trace data found in %s'%self.trace_directory, fg="yellow")
            return
        active_seg_id=0
        trace_data=self.get_trace_data(segs[active_seg_id])


        while True:
            print(Style.BRIGHT + '############### AVAILABLE ################' + Style.RESET_ALL)
            self.pretty_print_segments(segs, active_seg_id)
            self.pretty_print_fields(trace_data)
            print(Style.BRIGHT + '############### MENU ################' + Style.RESET_ALL)
            print('Enter command. (q to quit)')
            print('s: set active trace')
            print('p: plot')
            print('-------------------------------------')
            try:
                r = input()
                if r == 'q' or r == 'Q':
                    return
                elif r == 's':
                    active_seg_id=self.get_int([0,len(segs)-1])
                    trace_data = self.get_trace_data(segs[active_seg_id])
                elif r == 'p':
                    self.do_plot(trace_data)
                else:
                    print('Invalid entry')
            except(TypeError, ValueError):
                print('Invalid entry')

    def do_plot(self,trace_data):
        print(Style.BRIGHT + '############### Plotting ################' + Style.RESET_ALL)
        self.pretty_print_fields(trace_data)
        kk = list(trace_data['trace'].keys())
        kk.sort()
        id1=self.get_int([0,len(kk)-1],msg='FIELD ID')
        print('')
        yval = trace_data['trace'][kk[id1]]
        xval = np.array(trace_data['trace']['timestamp'])
        xval = (xval-xval[0])/60.0
        plt.ion()  # enable interactivity

        fig, axes = plt.subplots(1, 1, figsize=(15.0, 8.0), sharex=True)
        fig.canvas.set_window_title('TRACE')
        axes.set_yscale('linear')
        axes.set_xlabel('Time (m)')
        axes.set_ylabel(kk[id1].upper())
        axes.grid(True)
        axes.plot(xval, yval, 'b')
        fig.canvas.draw_idle()
    def pretty_print_fields(self,trace_data):
        i=0
        click.secho('%s | %s |  %s ' % ('ID'.ljust(8), 'FIELD'.ljust(25), 'SAMPLES'.ljust(25)), fg="cyan",bold=True)
        click.secho('-' * 110, fg="cyan", bold=True)
        kk=list(trace_data['trace'].keys())
        kk.sort()
        for k in kk:
            click.secho('%s | %s |  %s ' % (str(i).ljust(8), str(k).ljust(25), str(len(trace_data['trace'][k])).ljust(25)), fg="cyan",bold=True)
            i=i+1
        print('')

    def pretty_print_segments(self,segs,active_seg_id):
        click.secho('%s | %s |  %s |  %s' % ('TRACE'.ljust(8), 'DURATION(m)'.ljust(25), 'START'.ljust(30), 'END'.ljust(30)), fg="cyan",bold=True)
        click.secho('-' * 110, fg="cyan", bold=True)
        for i in range(len(segs)):
            duration = (segs[i]['ts_end'] - segs[i]['ts_start']) / 60
            dt_start = datetime.fromtimestamp(segs[i]['ts_start'])
            dt_end = datetime.fromtimestamp(segs[i]['ts_end'])

            ln='%s | %s |  %s |  %s' % (str(i).ljust(8), ('%.2f'%duration).ljust(25), str(dt_start).ljust(30), str(dt_end).ljust(30))
            if i==active_seg_id:
                click.secho(ln, fg="yellow")
            else:
                click.secho(ln, fg="cyan")
        print('')
    def get_trace_data(self,seg):
        """
        Assemble a dict with each field as a list of values
        """
        data={}
        data['ts_start']=seg['ts_start']
        data['ts_end']=seg['ts_end']
        data['trace']= {}
        for f in seg['filename']:
            with open(f, 'r') as s:
                trace=yaml.load(s,Loader=yaml.FullLoader)
                x = list(trace.keys())
                x.sort() #Chronological list of samples eg [trace_0000, trace_0001,...], oldest ID 0
                for xx in x: #trace_000,...
                    if len(data['trace'])==0:
                        #Build initial dict of {pimu.voltage:[],pimu.current:[],...}
                        for k in trace[x[0]].keys():
                            data['trace'][k]=[]
                    for k in trace[xx].keys():
                        data['trace'][k].append(float(trace[xx][k])) #All data should be a float so can scope
        return data

    def get_segments(self):
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