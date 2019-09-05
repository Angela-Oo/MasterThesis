import subprocess
import os

exe_path = "bin\master_thesis.exe"

if os.path.isfile(exe_path):
    print("exists: " + exe_path)
else:
    print("does not exists: " + exe_path)

list_args= []
list_args.append([exe_path, "-n", "10", "-r", "-refine_at_edge"])
list_args.append([exe_path, "-n", "10", "-r", "-refine_at_vertex"])
list_args.append([exe_path, '-s', "-r", "-refine_at_edge"])
list_args.append([exe_path, '-s', "-r", "-refine_at_vertex"])
#list_args.append([exe_path, '-n', '2'])


for  args in list_args:
    print("START REGISTRATION with args: " + ' '.join(args))

    proc = subprocess.Popen(args)
    proc.wait()
    print("FINISHED REGISTRATION\n")

print("finished")
