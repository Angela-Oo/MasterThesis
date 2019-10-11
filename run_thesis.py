import subprocess
import os

exe_path = "bin\master_thesis.exe"

if os.path.isfile(exe_path):
    print("exists: " + exe_path)
else:
    print("does not exists: " + exe_path)

list_args= []
base_args = [exe_path, "--image_folder_name", "images/run_2019_10_08_test", "--fit", "1.0", "--smooth", "3.0", "-p", "0.1", "-e", "0.095", "--max_iterations", "50"]

head_args = ["-i", "head"]
head_registration_args = base_args + head_args + ["-n", "-1"]
head_sequence_args = base_args + head_args + ["-s"]

hand_args = ["-i", "hand"]
hand_registration_args = base_args + hand_args + ["-n", "-1"]
hand_sequence_args = base_args + hand_args + ["-s"]

paperbag_args = ["-i", "paperbag"]
paperbag_registration_args = base_args + paperbag_args + ["-n", "-1"]
paperbag_sequence_args = base_args + paperbag_args + ["-s"]

puppet_args = ["-i", "puppet"]
puppet_args_registration_args = base_args + puppet_args + ["-n", "-1"]
puppet_args_sequence_args = base_args + puppet_args + ["-s"]


embedded_deformation_args = ["-t", "ED", "--ed_rigid", "100"]

refinement_args = ["--refine_deformation_graph", "--min_edge_length_percentage_of_area", "0.02", "--levels", "3", "--smooth_cost_threshold", "0.01", "--smooth_cost_percentage_of_max", "0.9"]
reduce_smooth_args = ["--reduce_smooth_factor"]
reduce_rigidity_args = ["--reduce_rigidity", "--minimal_rigidity", "0.001", "--rigidity_cost_threshold", "0.01"]
adaptive_rigidity_args = ["--adaptive_rigidity", "--rigidity_cost_coefficient", "0.01", "--minimal_rigidity_weight", "0.1"]
adaptive_rigidity_vertex_args = adaptive_rigidity_args + ["--rigidity_coefficient_per_vertex"]
adaptive_rigidity_edge_args = adaptive_rigidity_args


list_args.append(hand_sequence_args)
list_args.append(hand_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(hand_sequence_args + adaptive_rigidity_args)
list_args.append(hand_sequence_args + reduce_rigidity_args)
list_args.append(hand_sequence_args + refinement_args)
list_args.append(hand_sequence_args + refinement_args + ["--refine_at_edge"])
list_args.append(hand_sequence_args + reduce_smooth_args)

list_args.append(puppet_args_sequence_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_args)
list_args.append(puppet_args_sequence_args + reduce_rigidity_args)
list_args.append(puppet_args_sequence_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + refinement_args)
list_args.append(puppet_args_sequence_args + refinement_args + ["--refine_at_edge"])

list_args.append(paperbag_sequence_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_args)
list_args.append(paperbag_sequence_args + reduce_rigidity_args)
list_args.append(paperbag_sequence_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + refinement_args)
list_args.append(paperbag_sequence_args + refinement_args + ["--refine_at_edge"])

list_args.append(head_sequence_args)
list_args.append(head_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(head_sequence_args + adaptive_rigidity_args)
list_args.append(head_sequence_args + reduce_rigidity_args)
list_args.append(head_sequence_args + reduce_smooth_args)
list_args.append(head_sequence_args + refinement_args)
list_args.append(head_sequence_args + refinement_args + ["--refine_at_edge"])



list_args.append(hand_sequence_args + adaptive_rigidity_vertex_args + reduce_smooth_args)
list_args.append(hand_sequence_args + adaptive_rigidity_args + reduce_smooth_args)
list_args.append(hand_sequence_args + adaptive_rigidity_vertex_args + reduce_rigidity_args)
list_args.append(hand_sequence_args + adaptive_rigidity_args + reduce_rigidity_args)
list_args.append(hand_sequence_args + reduce_rigidity_args + reduce_smooth_args)
list_args.append(hand_sequence_args + refinement_args + reduce_smooth_args)
list_args.append(hand_sequence_args + refinement_args + ["--refine_at_edge"] + reduce_smooth_args)


list_args.append(puppet_args_sequence_args + adaptive_rigidity_vertex_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_vertex_args + reduce_rigidity_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_args + reduce_rigidity_args)
list_args.append(puppet_args_sequence_args + reduce_rigidity_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + refinement_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + refinement_args + ["--refine_at_edge"] + reduce_smooth_args)


list_args.append(paperbag_sequence_args + adaptive_rigidity_vertex_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_vertex_args + reduce_rigidity_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_args + reduce_rigidity_args)
list_args.append(paperbag_sequence_args + reduce_rigidity_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + refinement_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + refinement_args + ["--refine_at_edge"] + reduce_smooth_args)


list_args.append(head_sequence_args + adaptive_rigidity_vertex_args + reduce_rigidity_args)
list_args.append(head_sequence_args + adaptive_rigidity_args + reduce_rigidity_args)
list_args.append(head_sequence_args + adaptive_rigidity_args + reduce_smooth_args)
list_args.append(head_sequence_args + adaptive_rigidity_vertex_args + reduce_smooth_args)
list_args.append(head_sequence_args + reduce_rigidity_args + reduce_smooth_args)
list_args.append(head_sequence_args + refinement_args + reduce_smooth_args)
list_args.append(head_sequence_args + refinement_args + ["--refine_at_edge"] + reduce_smooth_args)







for  args in list_args:
    print("START REGISTRATION with args: " + ' '.join(args))

    proc = subprocess.Popen(args)
    proc.wait()
    print("FINISHED REGISTRATION\n")

print("finished")
