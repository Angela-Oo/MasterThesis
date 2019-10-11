import os
import re
from itertools import islice

def readLogFile(log):
    lines = []
    with open(log, 'r') as f:
        lines = f.readlines()
    return lines

def readNLinesOfLogFile(log, n):
    lines = []
    with open(log, 'r') as file:
        lines_gen = islice(file, n)
        for line in lines_gen:
            lines.append(line)
    return "".join(lines)

def readLogFileEnd(file, n, bs=1024):
    f = open(file)
    f.seek(0,2)
    l = 1-f.read(1).count('\n')
    B = f.tell()
    while n >= l and B > 0:
            block = min(bs, B)
            B -= block
            f.seek(B, 0)
            l += f.read(block).count('\n')
    f.seek(B, 0)
    l = min(l,n)
    lines = f.readlines()[-l:]
    f.close()
    return "".join(lines)

# parser

def timestringToDeltaTime(timestring):
    import datetime
    x = datetime.datetime.strptime(timestring, '%H:%M:%S')
    return datetime.timedelta(hours=x.hour, minutes=x.minute, seconds=x.second)

def timestringToSeconds(timestring):
    if timestring == '':
        return 0.
    else:
        return timestringToDeltaTime(timestring).total_seconds()

def parseTotalTime(lines):
    if len(lines) > 10:
        line = "".join(lines[-10:])
        m = re.search('time: \(h:min:s\) (\d+:\d+:\d+)   \-   finished registration', line)

        #if m:
        #    import datetime
        #    datetime.datetime.strptime(m.group(1), '%H:%M:%S')
        #    return datetime.timedelta(hours=x.tm_hour, minutes=x.tm_min, seconds=x.tm_sec).total_seconds()
        #else:
        #    return ""
        return m.group(1) if m else ""
    return ""

def parseOptions(lines):
    from collections import defaultdict
    lines = lines[0:30]
    lines = "".join(lines)
    #lines = readNLinesOfLogFile(log, 20)
    options = defaultdict(str)
    m = re.search('Input (\w*),', lines)
    options["input"] = m.group(1) if m else "unknown"

    m = re.search('Registration Type: (\w+[ \t]?\w*)', lines)
    options["registration type"] = m.group(1) if m else "unknown"

    m = re.search('Sequence registration', lines)
    options["sequence"] = "Sequence" if m else ""

    m = re.search('edge length: (\d+(\.\d*)?|\.\d+)', lines)
    options["edge length"] = m.group(1) if m else "unknown"

    m = re.search('Random probability to use a vertex: (\d+(\.\d*)?|\.\d+)', lines)
    options["vertex probability"] = m.group(1) if m else "unknown"

    options["refine at"] = ""
    m = re.search('Deformation Graph Refinement', lines)
    options["refinement"] = "Refine" if m else ""
    if m:
        m = re.search('refine at vertex', lines)
        options["refine at"] = "Vertex" if m else "Edge"

    m = re.search('Adaptive Rigidity by cost function', lines)
    options["adaptive rigidity"] = "Adaptive" if m else ""
    if m:
        m = re.search(', edge', lines)
        options["refine at"] = "Edge" if m else "Vertex"

    m = re.search('rigidity cost coefficient: (\d+(\.\d*)?|\.\d+)', lines)
    options["rigidity cost coef"] = m.group(1) if m else ""

    m = re.search('Adaptive Rigidity by reducing', lines)
    options["reduce rigidity"] = "ReduceRigidity" if m else ""

    m = re.search('Reduce smooth factor: true', lines)
    options["reduce smooth"] = "ReduceSmooth" if m else ""

    return options


def parseError(lines):
    line = lines[-2]
    log_dict = dict()
    float_pattern = '[-+]?(?:(?:\d*\.\d+)|(?:\d+\.?))(?:[Ee][+-]?\d+)?'

    m = re.search('chamfer_distance (' + float_pattern + ')', line)
    log_dict["error chamfer distance"] = float(m.group(1)) if m else "unknown"

    m = re.search('mean (' + float_pattern + ')', line)
    log_dict["error mean"] = float(m.group(1)) if m else "unknown"
    log_dict["scaled error mean"] = float(m.group(1))*1000 if m else "0."

    m = re.search('variance (' + float_pattern + ')', line)
    log_dict["error variance"] = float(m.group(1)) if m else "unknown"

    m = re.search('median (' + float_pattern + ')', line)
    log_dict["error median"] = float(m.group(1)) if m else "unknown"

    m = re.search('max (' + float_pattern + ')', line)
    log_dict["error max"] = float(m.group(1)) if m else "unknown"
    return log_dict


# parse frame

def parseTimePerFrame(lines):
    line = "".join(lines[-4: -1])
    time_dict = dict()
    m = re.search('time: \(h:min:s\) (\d+:\d+:\d+).*finished', line)

    if m:
        deltatime = timestringToDeltaTime(m.group(1))
        time_dict["time in s"] = deltatime.total_seconds()
    else:
        time_dict["time in s"] = 0.0

    return time_dict

def parseFrameNumberOfNodes(frame_lines):
    number_nodes = dict()
    lines = []
    for line in frame_lines:
        if "number of deformation graph nodes" in line:
            lines.append(line)

    if len(lines) > 0:
        m = re.search('number of deformation graph nodes (\d+)', lines[-1])
        number_nodes["number nodes"] = float(m.group(1)) if m else "unknown"
    else:
        number_nodes["number nodes"] = "unknown"
    return number_nodes

def parseFrameIndex(frame_line):
    m = re.search('frame (\d+)', frame_line)
    if m:
        return (True, float(m.group(1)))
    else:
        return (False, 0)

def parseFrame(frame_lines):
    from collections import defaultdict
    valid, frame = parseFrameIndex(frame_lines[-1])
    if valid and len(frame_lines) > 1:
        error = parseError(frame_lines)
        time = parseTimePerFrame(frame_lines)
        nodes = parseFrameNumberOfNodes(frame_lines)
        dict_values = {**error, **time}
        dict_values = {**dict_values, **nodes}

        dict_values['mean per node'] = dict_values['error mean'] * dict_values["number nodes"]
        dict_values['median per node'] = dict_values['error median'] * dict_values["number nodes"]

        dict_values['frame'] = frame
        return valid, defaultdict(str, dict_values)
    return valid, defaultdict(str)


def parseErrorPerFrame(lines):
    from itertools import groupby
    # split log lines by frames
    split_at_frames = []
    for match, group in groupby(lines, lambda x : re.search("(frame \d+)", x)):
        if not match:
            split_at_frames.append(list(group))
        else:
            split_at_frames[-1].append(match.group(1))

    # parse frames
    frames = []
    for frame_lines in split_at_frames:
        valid, parsed_frame = parseFrame(frame_lines)
        if valid:
            frames.append(parsed_frame)
    return frames

# parse log file

def calculateAverageFrameError(frames):
    import math
    error = dict()

    chamfer_values = [f['error chamfer distance'] for f in frames]
    error['chamfer'] = sum(chamfer_values) / len(chamfer_values)

    mean_values = [f['error mean'] for f in frames]
    error['mean'] = sum(mean_values) / len(mean_values)

    variance_values = [f['error variance'] for f in frames]
    error['variance'] = sum(variance_values) / len(variance_values)

    median_values = [f['error median'] for f in frames]
    error['median'] = median_values[math.floor(len(median_values) / 2)]

    node_values = [f['number nodes'] for f in frames]
    error['number nodes'] = sum(node_values) / len(node_values)
    error['min nodes'] = min(node_values)
    error['max nodes'] = max(node_values)

    mean_per_node_values = [(f['error mean'] * f['number nodes']) for f in frames]
    error['mean per node'] = sum(mean_per_node_values) / len(mean_per_node_values)

    median_per_node_values = [(f['error median'] * f['number nodes']) for f in frames]
    error['median per node'] = sum(median_per_node_values) / len(median_per_node_values)

    time_values = [f['time in s'] for f in frames]
    error['time in s'] = sum(time_values) / len(time_values)

    return error

def getNameFromInfo(info):
    name = ""
    if info["registration type"] is not "":
        name += info["registration type"]

    if info["refinement"] is not "":
        name += " " + info["refinement"] + " " + info["refine at"]

    if info["adaptive rigidity"] is not "":
        name += " " + info["adaptive rigidity"] + " " + info["refine at"]

    if info["reduce rigidity"] is not "":
        name += " " + info["reduce rigidity"]

    if info["reduce smooth"] is not "":
        name += " " + info["reduce smooth"]
    return name

def getLegendName(short_name):
    if short_name == 'ARAP':
        return 'As-Rigid-As-Possible'
    elif short_name == 'ARAP ReduceRigidity':
        return 'Rigidity Reduction'
    elif short_name == 'ARAP Adaptive Edge':
        return 'Adaptive Rigidity at Edge'
    elif short_name == 'ARAP Adaptive Vertex':
        return 'Adaptive Rigidity at Vertex'
    elif short_name == 'ARAP ReduceSmooth':
        return 'Smoothness Reduction'
    elif short_name == 'ARAP Refine Edge':
        return 'Refinement at Edge'
    elif short_name == 'ARAP Refine Vertex':
        return 'Refinement at Vertex'
    elif short_name == 'Embedded Deformation':
        return 'Embedded Deformation'

    elif short_name == 'ARAP Adaptive Edge ReduceSmooth':
        return 'Adaptive Rigidity at Edge SR'
    elif short_name == 'ARAP Adaptive Vertex ReduceSmooth':
        return 'Adaptive Rigidity at Vertex SR'
    elif short_name == 'ARAP Adaptive Edge ReduceRigidity':
        return 'Adaptive Rigidity at Edge RR'
    elif short_name == 'ARAP Adaptive Vertex ReduceRigidity':
        return 'Adaptive Rigidity at Vertex RR'
    elif short_name == 'ARAP ReduceRigidity ReduceSmooth':
        return 'RR and SR'

    elif short_name == 'ARAP Refine Edge ReduceSmooth':
        return 'Refinement at Edge SR'
    elif short_name == 'ARAP Refine Vertex ReduceSmooth':
        return 'Refinement at Vertex SR'
    #elif short_name == 'ARAP Refine Edge ReduceRigidity':
    #    return 'Refinement at Edge RR'
    #elif short_name == 'ARAP Refine Vertex ReduceRigidity':
    #    return 'Refinement at Vertex RR'

    return name

def parseInfo(lines, frames):
    info = parseOptions(lines)
    info["time"] = parseTotalTime(lines)
    error = calculateAverageFrameError(frames)
    info["chamfer error"] = error['chamfer']
    info["mean error"] = error['mean']
    info["variance error"] = error['variance']
    info["median error"] = error['median']
    info["number nodes"] = error['number nodes']
    info["min nodes"] = error['min nodes']
    info["max nodes"] = error['max nodes']
    info["mean per node"] = error['mean per node']
    info["median per node"] = error['median per node']
    info["time per frame log"] = error['time in s']
    info["number frames"] = len(frames)
    info["time per frame"] = timestringToSeconds(info["time"]) / info["number frames"]
    info["name"] = getNameFromInfo(info)
    info["legend name"] = getLegendName(info["name"])
    return info


def parseLogFile(log):
    try:
        lines = readLogFile(log)
        frames = parseErrorPerFrame(lines)
        info = parseInfo(lines, frames)
        info['log file'] = log
        return info, frames
    except:
        return False, False

def getAllLogFiles(path):
    log_files = []
    for r, d, f in os.walk(path):
        for file in f:
            if 'log.txt' in file:
                log_files.append(os.path.join(r, file))
    return log_files





def getVariant(parsed_logs, name):
    logs = [log for log in parsed_logs if (log[0]['name'] == name)]
    return logs[-1] if len(logs) > 0 else []

def clusterDataset(parsed_logs):
    variants = dict()
    variants['ARAP'] = getVariant(parsed_logs, 'ARAP')
    variants['ARAP ReduceRigidity'] = getVariant(parsed_logs, 'ARAP ReduceRigidity')
    variants['ARAP Adaptive Edge'] = getVariant(parsed_logs, 'ARAP Adaptive Edge')
    variants['ARAP Adaptive Vertex'] = getVariant(parsed_logs, 'ARAP Adaptive Vertex')
    variants['ARAP ReduceSmooth'] = getVariant(parsed_logs, 'ARAP ReduceSmooth')
    variants['ARAP Refine Edge'] = getVariant(parsed_logs, 'ARAP Refine Edge')
    variants['ARAP Refine Vertex'] = getVariant(parsed_logs, 'ARAP Refine Vertex')
    variants['Embedded Deformation'] = getVariant(parsed_logs, 'Embedded Deformation')

    variants['ARAP Adaptive Edge ReduceSmooth'] = getVariant(parsed_logs, 'ARAP Adaptive Edge ReduceSmooth')
    variants['ARAP Adaptive Vertex ReduceSmooth'] = getVariant(parsed_logs, 'ARAP Adaptive Vertex ReduceSmooth')
    variants['ARAP Adaptive Edge ReduceRigidity'] = getVariant(parsed_logs, 'ARAP Adaptive Edge ReduceRigidity')
    variants['ARAP Adaptive Vertex ReduceRigidity'] = getVariant(parsed_logs, 'ARAP Adaptive Vertex ReduceRigidity')
    variants['ARAP ReduceRigidity ReduceSmooth'] = getVariant(parsed_logs, 'ARAP ReduceRigidity ReduceSmooth')

    variants['ARAP Refine Edge ReduceSmooth'] = getVariant(parsed_logs, 'ARAP Refine Edge ReduceSmooth')
    variants['ARAP Refine Vertex ReduceSmooth'] = getVariant(parsed_logs, 'ARAP Refine Vertex ReduceSmooth')
    variants['ARAP Refine Edge ReduceRigidity'] = getVariant(parsed_logs, 'ARAP Refine Edge ReduceRigidity')
    variants['ARAP Refine VertexReduceRigidity'] = getVariant(parsed_logs, 'ARAP Refine Vertex ReduceRigidity')
    return variants


def parseAndClusteredLogFiles(path):
    log_files = getAllLogFiles(path)

    from collections import defaultdict
    logs_datasets = defaultdict(list)
    for log in log_files:
        try:
            parsed_log = parseLogFile(log)
            dataset = parsed_log[0]['input']
            logs_datasets[dataset].append(parsed_log)
            print("parsed " + log)
        except:
            print("not able to parse " + log)

    clustered_datasets = defaultdict(dict)
    for dataset in logs_datasets:
        try:
            clustered_datasets[dataset] = clusterDataset(logs_datasets[dataset])
        except:
            print("not able to cluster data set " + dataset)

    return clustered_datasets




def test():
    path = "../images/run_2019_09_24"

    log_files = getAllLogFiles(path)

    #subdirectories = [x[0] for x in os.walk(root_directory)]
    for x in log_files:
        print(x)

    parsed = parseLogFile(log_files[0])