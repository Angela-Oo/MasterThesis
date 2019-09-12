import os
import re
from itertools import islice

def readLogFile(log, n):
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


def parseOptions(log):
    lines = readLogFile(log, 20)
    log_dict = dict()
    m = re.search('(\w*)_log.txt', log)
    if m:
        log_dict["name"] = m.group(1)
    m = re.search('Sequence registration', lines)
    log_dict["sequence"] = "Sequence" if m else ""

    m = re.search('edge length: (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["edge length"] = m.group(1) if m else "unknown"

    m = re.search('Random probability to use a vertex: (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["vertex probability"] = m.group(1) if m else "unknown"

    m = re.search('Deformation Graph Refinement', lines)
    log_dict["refinement"] = "Refine" if m else ""

    m = re.search('Adaptive Rigidity by cost function', lines)
    log_dict["adaptive rigidity"] = "Adaptive" if m else ""

    m = re.search('rigidity cost coefficient: (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["rigidity cost coef"] = m.group(1) if m else ""

    m = re.search('Adaptive Rigidity by reducing', lines)
    log_dict["reduce rigidity"] = "Reduce" if m else ""

    m = re.search('refine at vertex', lines)
    if m:
        log_dict["refine at vertex"] = "True"
    return log_dict

def parseError(log):
    lines = readLogFileEnd(log, 7)
    log_dict = dict()
    m = re.search('error: mean (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["error mean"] = m.group(1) if m else "unknown"

    m = re.search('variance (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["error variance"] = m.group(1) if m else "unknown"

    m = re.search('median (\d+(\.\d*)?|\.\d+)', lines)
    log_dict["error median"] = m.group(1) if m else "unknown"

    return log_dict

def parseLogFile(log):
    option_dict = parseOptions(log)
    error_dict = parseError(log)
    log_dict = {**option_dict, **error_dict}
    return log_dict


path = "images/run_2019_09_10"

log_files = []
# r=root, d=directories, f = files
for r, d, f in os.walk(path):
    for file in f:
        if 'log.txt' in file:
            log_files.append(os.path.join(r, file))

#subdirectories = [x[0] for x in os.walk(root_directory)]
for x in log_files:
    print(x)

dicts = []
for log in log_files:
    log_dict = parseLogFile(log)
    dicts.append(log_dict)


print("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}".format('Name', 'Sequence', 'Refinement', 'Adaptive', 'Reduce', 'Edge Length', 'Vertex prob', 'Error mean', 'median', 'variance'))
for log_dict in dicts:
    #for key in log_dict:
    #    print(key + ": " + log_dict[key], end = ' ')
    #print('')
    print("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}"
          .format(log_dict['name'],
                  log_dict['sequence'],
                  log_dict['refinement'],
                  log_dict["adaptive rigidity"],
                  log_dict["reduce rigidity"],
                  log_dict['edge length'],
                  log_dict['vertex probability'],
                  log_dict['error mean'],
                  log_dict['error median'],
                  log_dict['error variance']))