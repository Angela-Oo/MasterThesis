import subprocess
import os

exe_path = "bin\master_thesis.exe"

if os.path.isfile(exe_path):
    print("exists: " + exe_path)
else:
    print("does not exists: " + exe_path)

list_args= []
base_args = [exe_path, "--image_folder_name", "images/run_2019_09_24", "--fit", "1.0", "--smooth", "2.0"]

head_args = ["-i", "head", "-p", "0.1", "-e", "0.3", "--smooth_cost_threshold", "0.03", "--rigidity_cost_coefficient", "0.01"]
head_registration_args = base_args + head_args + ["-n", "-1"]
head_sequence_args = base_args + head_args + ["-s"]

hand_args = ["-i", "hand", "-p", "0.15", "-e", "0.15", "--smooth_cost_threshold", "0.02", "--rigidity_cost_coefficient", "0.01"]
hand_registration_args = base_args + hand_args + ["-n", "-1"]
hand_sequence_args = base_args + hand_args + ["-s"]

paperbag_args = ["-i", "paperbag", "-p", "0.3", "-e", "0.3", "--smooth_cost_threshold", "0.02", "--rigidity_cost_coefficient", "0.001"]
paperbag_registration_args = base_args + paperbag_args + ["-n", "-1"]
paperbag_sequence_args = base_args + paperbag_args + ["-s"]

puppet_args = ["-i", "puppet", "-p", "0.25", "-e", "0.3", "--smooth_cost_threshold", "0.02", "--rigidity_cost_coefficient", "0.07"]# "--rigidity_cost_coefficient", "0.1" good
puppet_args_registration_args = base_args + puppet_args + ["-n", "-1"]
puppet_args_sequence_args = base_args + puppet_args + ["-s"]

refinement_args = ["--refine_deformation_graph", "--min_edge_length", "0.1", "--levels", "4", "--smooth_cost_threshold", "0.1", "--smooth_cost_percentage_of_max", "0.1"]
refinement_args_level_3 = ["--refine_deformation_graph", "--min_edge_length", "0.1", "--levels", "3", "--smooth_cost_threshold", "0.1", "--smooth_cost_percentage_of_max", "0.1"]

reduce_smooth_args = ["--reduce_smooth_factor", "--max_iterations", "10"]

reduce_rigidity_args = ["--reduce_rigidity", "--minimal_rigidity", "0.1", "--rigidity_cost_threshold", "0.01", "--max_iterations", "10"]

adaptive_rigidity_args = ["--adaptive_rigidity", "--max_iterations", "25"]
adaptive_rigidity_vertex_args = ["--adaptive_rigidity", "--rigidity_coefficient_per_vertex", "--max_iterations", "25"]

#adaptive_rigidity_edge_quadratic_weight_args = ["--adaptive_rigidity", "--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.1", "--use_quadratic_rigid_weight", "--max_iterations", "25"]
#adaptive_rigidity_vertex_quadratic_weight_args = ["--adaptive_rigidity", "--rigidity_coefficient_per_vertex", "--fit", "1.0", "--smooth", "2.0", "--rigidity_cost_coefficient", "0.001", "--use_quadratic_rigid_weight", "--max_iterations", "25"]

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



#list_args.append(paperbag_sequence_args)
#list_args.append(paperbag_sequence_args + adaptive_rigidity_vertex_args)
#list_args.append(paperbag_sequence_args + adaptive_rigidity_args)
#list_args.append(paperbag_sequence_args + reduce_smooth_args)
#list_args.append(paperbag_sequence_args + reduce_rigidity_args)
#list_args.append(paperbag_sequence_args + refinement_args)
#list_args.append(paperbag_sequence_args + refinement_args + ["--refine_at_edge"])



list_args.append(hand_sequence_args)
list_args.append(hand_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(hand_sequence_args + adaptive_rigidity_args)
list_args.append(hand_sequence_args + reduce_smooth_args)
list_args.append(hand_sequence_args + reduce_rigidity_args)
list_args.append(hand_sequence_args + refinement_args_level_3)
list_args.append(hand_sequence_args + refinement_args_level_3 + ["--refine_at_edge"])


#list_args.append(puppet_args_sequence_args)
#list_args.append(puppet_args_sequence_args + adaptive_rigidity_vertex_args)
#list_args.append(puppet_args_sequence_args + adaptive_rigidity_args)
#list_args.append(puppet_args_sequence_args + reduce_smooth_args)
#list_args.append(puppet_args_sequence_args + reduce_rigidity_args)
list_args.append(puppet_args_sequence_args + refinement_args)
list_args.append(puppet_args_sequence_args + refinement_args + ["--refine_at_edge"])

#list_args.append(head_sequence_args)
list_args.append(head_sequence_args + adaptive_rigidity_vertex_args)
list_args.append(head_sequence_args + adaptive_rigidity_args)
list_args.append(head_sequence_args + reduce_smooth_args)
#list_args.append(head_sequence_args + reduce_rigidity_args)
list_args.append(head_sequence_args + refinement_args)
list_args.append(head_sequence_args + refinement_args + ["--refine_at_edge"])




for  args in list_args:
    print("START REGISTRATION with args: " + ' '.join(args))

    proc = subprocess.Popen(args)
    proc.wait()
    print("FINISHED REGISTRATION\n")

print("finished")
