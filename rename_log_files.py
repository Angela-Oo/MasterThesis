import os
import re
from shutil import copyfile


path = "images/run_2019_09_24"

log_files = []
# r=root, d=directories, f = files
for r, d, f in os.walk(path):
    for file in f:
        if 'log.txt' in file:
            log_files.append(os.path.join(r, file))

if not os.path.exists(path + '\\log_files\\'):
    os.makedirs(path + '\\log_files\\')

for log in log_files:
    output_file = log
    output_file = output_file.replace(path + '\\', '')
    output_file = path + '\\log_files\\' + output_file.replace('\\', '_')
    print("src: " + log + " dst: " + output_file)
    copyfile(log, output_file)



