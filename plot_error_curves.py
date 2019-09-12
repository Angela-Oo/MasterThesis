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
            print(line)
        m = re.search('(frame \d+) solved', line)
        if m:
            error_lines.append(m.group(1))
            print(m.group(1))


def parseError(log):
    lines = readLogFileEnd(log, 80)
    log_dict = dict()
    m = re.search('error: mean (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["error mean"] = m.group(1) if m else "unknown"

    m = re.search('variance (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["error variance"] = m.group(1) if m else "unknown"

    m = re.search('median (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["error median"] = m.group(1) if m else "unknown"

    m = re.search('number of deformation graph nodes (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["nodes"] = m.group(1) if m else ""

    m = re.search('time: \(h:min:s\) (\d+:\d+:\d+)   \-   finished registration', lines)
    log_dict["time"] = m.group(1) if m else ""
    return log_dict

def parseLogFile(log):
    lines = readLogFile(log)
    return getErrorLines(lines)


def getAllLogFiles(path):
    log_files = []
    # r=root, d=directories, f = files
    for r, d, f in os.walk(path):
        for file in f:
            if re.search('log.*.txt', file):
                log_files.append(os.path.join(r, file))
    return log_files



path = "images/results_10_09"
log_files = getAllLogFiles(path)

for x in log_files:
    print(x)

dicts = []
for log in log_files:
    log_dict = parseLogFile(log)
    dicts.append(log_dict)

