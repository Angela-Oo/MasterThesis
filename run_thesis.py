import subprocess
import os

exe_path = "bin\master_thesis.exe"

if os.path.isfile(exe_path):
    print("exists: " + exe_path)
else:
    print("does not exists: " + exe_path)

list_args= []
#base_args = [exe_path, "--image_folder_name", "images/run_2019_09_28", "--fit", "1.0", "--smooth", "5.0", "-p", "0.25", "-e", "0.095"]
base_args = [exe_path, "--image_folder_name", "images/run_2019_10_03_max_iterations_50", "--fit", "1.0", "--smooth", "3.0", "-p", "0.1", "-e", "0.095", "--max_iterations", "50"]

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

refinement_args = ["--refine_deformation_graph", "--min_edge_length_percentage_of_area", "0.025", "--levels", "3", "--smooth_cost_threshold", "0.1", "--smooth_cost_percentage_of_max", "0.9"]
#refinement_args_level_3 = ["--refine_deformation_graph", "--min_edge_length", "0.1", "--levels", "3", "--smooth_cost_threshold", "0.1", "--smooth_cost_percentage_of_max", "0.1"]

reduce_smooth_args = ["--reduce_smooth_factor"]

reduce_rigidity_args = ["--reduce_rigidity", "--minimal_rigidity", "0.1", "--rigidity_cost_threshold", "0.01"]

adaptive_rigidity_args = ["--adaptive_rigidity", "--rigidity_cost_coefficient", "0.02", "--minimal_rigidity_weight", "0.1"]
#adaptive_rigidity_args = ["--adaptive_rigidity", "--rigidity_cost_coefficient", "0.01", "--minimal_rigidity_weight", "0.1"]
#adaptive_rigidity_args = ["--adaptive_rigidity", "--rigidity_cost_coefficient", "0.05", "--minimal_rigidity_weight", "0.02"] # minimal rigidity weight 0.1/5 = 0.02
adaptive_rigidity_vertex_args = adaptive_rigidity_args + ["--rigidity_coefficient_per_vertex"]
adaptive_rigidity_edge_args = adaptive_rigidity_args

#adaptive_rigidity_edge_quadratic_weight_args = ["--adaptive_rigidity", "--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.1", "--use_quadratic_rigid_weight", "--max_iterations", "25"]
#adaptive_rigidity_vertex_quadratic_weight_args = ["--adaptive_rigidity", "--rigidity_coefficient_per_vertex", "--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.001", "--use_quadratic_rigid_weight", "--max_iterations", "25"]


list_args.append(puppet_args_sequence_args + reduce_rigidity_args)
list_args.append(puppet_args_sequence_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args)
list_args.append(puppet_args_sequence_args + refinement_args)
list_args.append(puppet_args_sequence_args + embedded_deformation_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_args)

list_args.append(puppet_args_sequence_args + refinement_args + ["--refine_at_edge"])

list_args.append(paperbag_sequence_args)
list_args.append(paperbag_sequence_args + embedded_deformation_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_args)
list_args.append(paperbag_sequence_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + reduce_rigidity_args)
list_args.append(paperbag_sequence_args + refinement_args)
list_args.append(paperbag_sequence_args + refinement_args + ["--refine_at_edge"])







list_args.append(hand_sequence_args)
list_args.append(hand_sequence_args + embedded_deformation_args)
list_args.append(hand_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(hand_sequence_args + adaptive_rigidity_args)
list_args.append(hand_sequence_args + reduce_smooth_args)
list_args.append(hand_sequence_args + reduce_rigidity_args)
list_args.append(hand_sequence_args + refinement_args)
list_args.append(hand_sequence_args + refinement_args + ["--refine_at_edge"])



list_args.append(head_sequence_args)
list_args.append(head_sequence_args + embedded_deformation_args)
list_args.append(head_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(head_sequence_args + adaptive_rigidity_args)
list_args.append(head_sequence_args + reduce_smooth_args)
list_args.append(head_sequence_args + reduce_rigidity_args)
list_args.append(head_sequence_args + refinement_args)
list_args.append(head_sequence_args + refinement_args + ["--refine_at_edge"])



list_args.append(paperbag_sequence_args + adaptive_rigidity_vertex_args + reduce_rigidity_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_args + reduce_rigidity_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_vertex_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + reduce_rigidity_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + refinement_args + reduce_smooth_args)
list_args.append(paperbag_sequence_args + refinement_args + ["--refine_at_edge"] + reduce_smooth_args)

list_args.append(puppet_args_sequence_args + adaptive_rigidity_vertex_args + reduce_rigidity_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_args + reduce_rigidity_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_vertex_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + reduce_rigidity_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + refinement_args + reduce_smooth_args)
list_args.append(puppet_args_sequence_args + refinement_args + ["--refine_at_edge"] + reduce_smooth_args)

list_args.append(hand_sequence_args + adaptive_rigidity_vertex_args + reduce_rigidity_args)
list_args.append(hand_sequence_args + adaptive_rigidity_args + reduce_rigidity_args)
list_args.append(hand_sequence_args + adaptive_rigidity_vertex_args + reduce_smooth_args)
list_args.append(hand_sequence_args + adaptive_rigidity_args + reduce_smooth_args)
list_args.append(hand_sequence_args + reduce_rigidity_args + reduce_smooth_args)
list_args.append(hand_sequence_args + refinement_args + reduce_smooth_args)
list_args.append(hand_sequence_args + refinement_args + ["--refine_at_edge"] + reduce_smooth_args)

list_args.append(head_sequence_args + adaptive_rigidity_vertex_args + reduce_rigidity_args)
list_args.append(head_sequence_args + adaptive_rigidity_args + reduce_rigidity_args)
list_args.append(head_sequence_args + adaptive_rigidity_vertex_args + reduce_smooth_args)
list_args.append(head_sequence_args + adaptive_rigidity_args + reduce_smooth_args)
list_args.append(head_sequence_args + reduce_rigidity_args + reduce_smooth_args)
list_args.append(head_sequence_args + refinement_args + reduce_smooth_args)
list_args.append(head_sequence_args + refinement_args + ["--refine_at_edge"] + reduce_smooth_args)


'''
list_args.append(hand_sequence_args + adaptive_rigidity_vertex_quadratic_weight_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_vertex_quadratic_weight_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_vertex_quadratic_weight_args)
list_args.append(head_sequence_args + adaptive_rigidity_vertex_quadratic_weight_args)


list_args.append(hand_sequence_args + adaptive_rigidity_edge_quadratic_weight_args)
list_args.append(paperbag_sequence_args + adaptive_rigidity_edge_quadratic_weight_args)
list_args.append(puppet_args_sequence_args + adaptive_rigidity_edge_quadratic_weight_args)
list_args.append(head_sequence_args + adaptive_rigidity_edge_quadratic_weight_args)
'''


'''
# test
head_sequence_args_test = base_args + head_args + ["-s", "-n", "20"]
adaptive_rigidity_vertex_args_base = ["--adaptive_rigidity", "--rigidity_coefficient_per_vertex", "--max_iterations", "25"]

#list_args.append(head_sequence_args_test)
#list_args.append(head_sequence_args_test + ["--reduce_smooth_factor=false"])
list_args.append(head_sequence_args_test + adaptive_rigidity_vertex_args_base + ["--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.001"])
list_args.append(head_sequence_args_test + adaptive_rigidity_vertex_args_base + ["--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.001", "--reduce_smooth_factor=false"])
#list_args.append(head_sequence_args_test + adaptive_rigidity_vertex_args_base + ["--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.01"])
#list_args.append(head_sequence_args_test + adaptive_rigidity_vertex_args_base + ["--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.01", "--reduce_smooth_factor=false"])
#list_args.append(head_sequence_args_test + adaptive_rigidity_vertex_args_base + ["--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.05"])
#list_args.append(head_sequence_args_test + adaptive_rigidity_vertex_args_base + ["--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.05", "--reduce_smooth_factor=false"])
'''

'''
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
'''




for  args in list_args:
    print("START REGISTRATION with args: " + ' '.join(args))

    proc = subprocess.Popen(args)
    proc.wait()
    print("FINISHED REGISTRATION\n")

print("finished")
