import subprocess
import os

exe_path = "bin\master_thesis.exe"

if os.path.isfile(exe_path):
    print("exists: " + exe_path)
else:
    print("does not exists: " + exe_path)

list_args= []
list_args.append([exe_path, "-i", "head", "-p", "0.1", "-n", "10", "-r", "--refinment_smooth_cost_threshold", "0.02"])
list_args.append([exe_path, "-i", "head", "-p", "0.1", "-n", "10", "-r", "--refine_at_edge", "--refinment_smooth_cost_threshold", "0.02"])
#list_args.append([exe_path, "-i", "head", "-p", "0.1", "-s", "-r"])
#list_args.append([exe_path, "-i", "head", "-p", "0.1", "-s", "-r", "--refine_at_edge"])

#list_args.append([exe_path, "-i", "hand", "-p", "0.2", "-n", "10", "-r"])
#list_args.append([exe_path, "-i", "hand", "-p", "0.2", "-n", "10", "-r", "--refine_at_edge"])
#list_args.append([exe_path, "-i", "hand", "-p", "0.2", "-s", "-r"])
#list_args.append([exe_path, "-i", "hand", "-p", "0.2", "-s", "-r", "--refine_at_edge"])
#list_args.append([exe_path, '-n', '2'])


for  args in list_args:
    print("START REGISTRATION with args: " + ' '.join(args))

    proc = subprocess.Popen(args)
    proc.wait()
    print("FINISHED REGISTRATION\n")

print("finished")
