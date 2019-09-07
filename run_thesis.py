import subprocess
import os

exe_path = "bin\master_thesis.exe"

if os.path.isfile(exe_path):
    print("exists: " + exe_path)
else:
    print("does not exists: " + exe_path)

list_args= []
base_args = [exe_path, "--image_folder_name", "images/run_07_09"]

head_args = ["-i", "head", "-p", "0.1"]
head_registration_args = base_args + head_args + ["-n", "10"]
head_sequence_args = base_args + head_args + ["-s"]

hand_args = ["-i", "hand", "-p", "0.2"]
hand_registration_args = base_args + hand_args + ["-n", "10"]
hand_sequence_args = base_args + hand_args + ["-s"]

paperbag_args = ["-i", "paperbag", "-p", "0.2"]
paperbag_registration_args = base_args + paperbag_args + ["-n", "10"]
paperbag_sequence_args = base_args + paperbag_args + ["-s"]

puppet_args = ["-i", "puppet", "-p", "0.2"]
puppet_args_registration_args = base_args + puppet_args + ["-n", "10"]
puppet_args_sequence_args = base_args + puppet_args + ["-s"]

refinement_args = ["--refine_deformation_graph", "--smooth_cost_threshold", "0.02"]
reduce_rigidity_args = ["--reduce_rigidity", "-e", "0.3", "--fit", "5.0", "--smooth", "1.0", "--minimal_rigidity", "0.01", "--smooth_cost_threshold", "0.01", "--max_iterations", "10"]
adaptive_rigidity_args = ["--adaptive_rigidity", "-e", "0.3", "--fit", "5.0", "--smooth", "1.0", "--rigidity_cost_coefficient", "0.01", "--max_iterations", "50"]


list_args.append(head_registration_args)
list_args.append(head_registration_args + refinement_args)
list_args.append(head_registration_args + refinement_args + ["--refine_at_edge"])
list_args.append(head_registration_args + reduce_rigidity_args)
list_args.append(head_registration_args + adaptive_rigidity_args)

list_args.append(hand_registration_args)
list_args.append(hand_registration_args + refinement_args)
list_args.append(hand_registration_args + refinement_args + ["--refine_at_edge"])
list_args.append(hand_registration_args + reduce_rigidity_args)
list_args.append(hand_registration_args + adaptive_rigidity_args)

list_args.append(paperbag_registration_args)
list_args.append(paperbag_registration_args + refinement_args)
list_args.append(paperbag_registration_args + refinement_args + ["--refine_at_edge"])
list_args.append(paperbag_registration_args + reduce_rigidity_args)
list_args.append(paperbag_registration_args + adaptive_rigidity_args)

list_args.append(puppet_args_registration_args)
list_args.append(puppet_args_registration_args + refinement_args)
list_args.append(puppet_args_registration_args + refinement_args + ["--refine_at_edge"])
list_args.append(puppet_args_registration_args + reduce_rigidity_args)
list_args.append(puppet_args_registration_args + adaptive_rigidity_args)


list_args.append(head_sequence_args)
list_args.append(head_sequence_args + refinement_args)
list_args.append(head_sequence_args + refinement_args + ["--refine_at_edge"])
list_args.append(head_sequence_args + reduce_rigidity_args)
list_args.append(head_sequence_args + adaptive_rigidity_args)

list_args.append(hand_sequence_args)
list_args.append(hand_sequence_args + refinement_args)
list_args.append(hand_sequence_args + refinement_args + ["--refine_at_edge"])
list_args.append(hand_sequence_args + reduce_rigidity_args)
list_args.append(hand_sequence_args + adaptive_rigidity_args)

list_args.append(paperbag_sequence_args)
list_args.append(paperbag_sequence_args + refinement_args)
list_args.append(paperbag_sequence_args + refinement_args + ["--refine_at_edge"])
list_args.append(paperbag_sequence_args + reduce_rigidity_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_args)

list_args.append(puppet_args_sequence_args)
list_args.append(puppet_args_sequence_args + refinement_args)
list_args.append(puppet_args_sequence_args + refinement_args + ["--refine_at_edge"])
list_args.append(puppet_args_sequence_args + reduce_rigidity_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_args)

#-n 10 --reduce_rigidity -e 0.35 -p 0.1 --fit 5.0 --smooth 1.0  --minimal_rigidity 0.01 --smooth_cost_threshold 0.01 --max_iterations 10 --error_evaluation="false" --term="false" --render_mode "ALL"

#list_args.append([exe_path, "-i", "hand", "-p", "0.2", "-n", "10", "-r"])
#list_args.append([exe_path, "-i", "hand", "-p", "0.2", "-n", "10", "-r", "--refine_at_edge"])
#list_args.append([exe_path, "-i", "hand", "-p", "0.2", "-s", "-r"])
#list_args.append([exe_path, "-i", "hand", "-p", "0.2", "-s", "-r", "--refine_at_edge"])
#list_args.append([exe_path, '-n', '2'])

# working adaptive rigidity hyperparameter
#-n 10 --adaptive_rigidity -e 0.3 -p 0.1 --fit 5.0 --smooth 1.0  --rigidity_cost_coefficient 0.01 --max_iterations 50

for  args in list_args:
    print("START REGISTRATION with args: " + ' '.join(args))

    proc = subprocess.Popen(args)
    proc.wait()
    print("FINISHED REGISTRATION\n")

print("finished")
