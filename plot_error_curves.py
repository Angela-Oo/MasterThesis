import os
import re
from itertools import islice

def readLogFile(log):
    lines = []
    with open(log, 'r') as f:
        lines = f.readlines()
    return lines

def getErrorLines(lines):
    error_lines = []
    for line in lines:
        if re.search('error: mean', line):
            error_lines.append(line)
            #print(line)
        m = re.search('(frame \d+) solved', line)
        if m:
            error_lines.append(m.group(1))
            #print(m.group(1))
    return error_lines


def parseError(lines):
    log_dict = dict()

    float_pattern = '[-+]?(?:(?:\d*\.\d+)|(?:\d+\.?))(?:[Ee][+-]?\d+)?'

    #m = re.search('error: mean (\d+(\.\d*)?|\.\d+)', lines)
    m = re.search('error: mean (' + float_pattern + ')', lines)
    log_dict["error mean"] = float(m.group(1)) if m else "unknown"

    m = re.search('variance (' + float_pattern + ')', lines)
    log_dict["error variance"] = float(m.group(1)) if m else "unknown"

    m = re.search('median (' + float_pattern + ')', lines)
    log_dict["error median"] = float(m.group(1)) if m else "unknown"

    m = re.search('max (' + float_pattern + ')', lines)
    log_dict["error max"] = float(m.group(1)) if m else "unknown"
    return log_dict

def getErrorForEachFrame(lines):
    from itertools import groupby
    split_at_frames = []
    for match, group in groupby(lines, lambda x : re.search("(frame \d+)", x)):
        if not match:
            split_at_frames.append(list(group))
        else:
            split_at_frames[-1].append(match.group(1))

    error_dict = dict()
    for frame in split_at_frames:
        error_dict[frame[-1]] = parseError(frame[-2])

    from collections import defaultdict
    test_error_dict = defaultdict(list)
    for frame in split_at_frames:
        error = parseError(frame[-2])
        for key in error:
            test_error_dict[key].append(error[key])
        m = re.search('frame (\d+)', frame[-1])
        if m:
            test_error_dict['frame'].append(int(m.group(1)))

    return test_error_dict
    #return error_dict

def parseLogFile(log):
    lines = readLogFile(log)
    error_lines = getErrorLines(lines)
    return getErrorForEachFrame(error_lines)

def getAllLogFiles(path):
    log_files = []
    # r=root, d=directories, f = files
    for r, d, f in os.walk(path):
        for file in f:
            if re.search('log.*.txt', file):
                log_files.append(os.path.join(r, file))
    return log_files



path = "images/run_2019_09_13"
log_files = getAllLogFiles(path)

for x in log_files:
    print(x)

dicts = dict()
for log in log_files:
    key = log.replace(path + '\\', '')
    key = key.replace('\\', '_')
    log_dict = parseLogFile(log)
    dicts[key] = log_dict


import matplotlib.pyplot as plt
import sys
import os

#data = {'velocity' : [2,4,6,8,12,50],
#        'time' : [12, 12, 12, 16, 18, 210]}

plt.figure(1)

#data = next(iter(dicts.values()))
for key in dicts:
    #if "hand" in key:
    if True:
        data = dicts[key]
        if len(data['frame']) == len(data['error mean']):
            plt.plot(data['frame'] ,data['error mean'])
            plt.gcf().autofmt_xdate()

plt.title('frame vs mean error')
plt.show()

