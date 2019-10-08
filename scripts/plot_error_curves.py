
import numpy as np
import matplotlib.pyplot as plt
import math

from parse_log_files import parseAndClusteredLogFiles


def main():
    path = "../images/run_2019_10_08"
    #output_path = path + "/plots"

    logs_datasets = parseAndClusteredLogFiles(path)

    for dataset in logs_datasets:
        output_path = path + "/" + dataset
        try:
            plotDataset(logs_datasets[dataset], dataset, output_path)
        except:
            print("not able to plot dataset " + dataset)


def plotDataset(dataset, dataset_name, output_path):
    names = ['ARAP',
             'ARAP ReduceRigidity', #1
             'ARAP Adaptive Edge',  #2
             'ARAP Adaptive Vertex',#3
             'ARAP ReduceSmooth',
             'ARAP Refine Edge',
             'ARAP Refine Vertex',
             'Embedded Deformation', #7
             'ARAP Adaptive Edge ReduceSmooth',   #8
             'ARAP Adaptive Vertex ReduceSmooth', #9
             'ARAP Adaptive Edge ReduceRigidity', #10
             'ARAP Adaptive Vertex ReduceRigidity', #11
             'ARAP ReduceRigidity ReduceSmooth',    #12
             'ARAP Refine Edge ReduceSmooth',       #13
             'ARAP Refine Vertex ReduceSmooth',
    ]

    dataset_no_ed = dict()
    for k in dataset:
        if k in names[0:7]:
            dataset_no_ed[k] = dataset[k]

    dataset_rigidity = dict()
    for k in dataset:
        if k in names[0:4]:
            dataset_rigidity[k] = dataset[k]

    dataset_refinment = dict()
    for k in dataset:
        if k in names[4:7] or k in names[0]:
            dataset_refinment[k] = dataset[k]


    dataset_combinations_rigidity_reduction = dict()
    for k in dataset:
        if k in names[10:12] or k in names[0] or k in names[1]:
            dataset_combinations_rigidity_reduction[k] = dataset[k]

    dataset_combinations_smoothness_reduction_adaptive = dict()
    for k in dataset:
        if k in names[8:10] or k in names[0] or k in names[4]:
            dataset_combinations_smoothness_reduction_adaptive[k] = dataset[k]

    dataset_combinations_smoothness_reduction_refinement = dict()
    for k in dataset:
        if k in names[13:] or k in names[0] or k in names[4]:
            dataset_combinations_smoothness_reduction_refinement[k] = dataset[k]

    dataset_combinations_smoothness_rigidity_reduction = dict()
    for k in dataset:
        if k in names[12:13] or k in names[0:2] or k in names[4]:
            dataset_combinations_smoothness_rigidity_reduction[k] = dataset[k]

    for k in dataset:
        if dataset[k]:
            print("log file " + dataset[k][0]['log file'])

    #plotDatasetMeanAndVariance(dataset, dataset_name + '_with_ed', output_path)
    plotDatasetMean(dataset_no_ed, dataset_name, output_path)
    plotDatasetMedian(dataset_no_ed, dataset_name, output_path)
    plotDatasetMean(dataset_rigidity, dataset_name + ' adaptive rigidity', output_path)
    plotDatasetMean(dataset_refinment, dataset_name + ' refinement', output_path)
    plotDatasetMeanPerNode(dataset_refinment, dataset_name + ' refinement', output_path)

    plotDatasetMeanCombinations(dataset_combinations_smoothness_reduction_adaptive, dataset_name, output_path + "/smoothness_adaptive_")
    plotDatasetMeanCombinations(dataset_combinations_smoothness_reduction_refinement, dataset_name,
                                output_path + "/smoothness_refinement_")
    plotDatasetMeanCombinations(dataset_combinations_rigidity_reduction, dataset_name, output_path + "/rigidity_")
    plotDatasetMeanCombinations(dataset_combinations_smoothness_rigidity_reduction, dataset_name, output_path + "/rigidity_smoothness_")
    #plotDatasetMeanAndVariance(dataset, dataset_name + '_all', output_path)
    #plotDatasetMeanAndVariance([arap_logs, adaptive_rigidity_edge, adaptive_rigidity_vertex, refinement_edge, reduce_smooth, reduce_rigidity], dataset, output_path)

def plotDatasetMean(logs, title, output_path):
    plotAndSaveImage(logs, 'error mean', title, 'Chamfer Distance Mean', output_path + "/mean_")
    plotAndSaveImage(logs, 'error mean', title, 'Chamfer Distance Mean', output_path + "/legend_mean_", False, 'lower right')

def plotDatasetMeanCombinations(logs, title, output_path):
    plotAndSaveImageCombinations(logs, 'error mean', title, 'Chamfer Distance Mean', output_path + "mean_")
    plotAndSaveImageCombinations(logs, 'error mean', title, 'Chamfer Distance Mean', output_path + "legend_mean_", False, 'lower right')

def plotDatasetMeanPerNode(logs, title, output_path):
    plotAndSaveImage(logs, 'mean per node', title, 'Chamfer Distance Mean per Node', output_path + "/mean_per_node_")

def plotDatasetMedian(logs, title, output_path):
    plotAndSaveImage(logs, 'error median', title, 'chamfer distance median', output_path + "/median_", True)
    plotAndSaveImage(logs, 'median per node', title, 'chamfer distance median per node', output_path + "/median_per_node_", True)



def plotAndSaveImage(logs, key, title, ylabel, output_path, log_scale = False, legend_location = 'upper left'):
    #print('output path' + " " + output_path + title)
    plt.rcParams.update({'font.size': 12})
    fig = plt.figure(figsize=plt.figaspect(0.60))

    plt.title(title, {'fontsize':18})
    #plt.xlabel('Frames', {'fontsize':16})
    #plt.ylabel(ylabel, fontsize=16)
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.set_xlabel('Frames', fontsize=14)  # xlabel
    ax1.set_ylabel(ylabel, fontsize=14)  # ylabel

    if log_scale:
        ax1.set_yscale('log')

    plot_colors = ['r', 'b', 'm', 'c', 'g',  'y', 'k',
                   (0.0, 1.0, 0.5, 1.), (1.0, 0.5, 0.0, 1.), (0.5, 1.0, 0.0, 1.), (0.0, 0.5, 1.0, 1.),
                    (0.25, 0.5, 0.25, 1.), (0.25, 0.5, 0.5, 1.), 'b', 'b', 'b', 'b', 'b']

    n = 0
    for k in logs:
        data = logs[k]
        if not data:
            n = n + 1
            continue
        log_dict = data[1]
        error_means = [(d[key]) for d in log_dict]
        frames = [int(d['frame']) for d in log_dict]
        std_deviations = [d['error variance'] for d in log_dict]

        ax1.plot(frames, error_means, color = plot_colors[n], label = data[0]['legend name'])

        #lower_std = [max(0, error_means[i]-(std_deviations[i])) for i in range(len(frames))]
        #ax1.plot(frames, lower_std, color=std_dev_line_colors[n], linestyle='dashed')

        #upper_std = [error_means[i]+(std_deviations[i]) for i in range(len(frames))]
        #ax1.plot(frames, upper_std, color=std_dev_line_colors[n], linestyle='dashed')

        #plt.fill_between(steps, lower_std, upper_std, color=std_dev_fill_colors[n])
        n = n+1

    ax1.set_xlim([0, max(frames)])
    #ax1.set_ylim([0, 0.00001])
    x_step_size = 10. if max(frames) < 75 else 20.
    plt.xticks(np.arange(min(frames)-1, max(frames) + 1, x_step_size))

    plt.legend(loc=legend_location, prop={'size': 12})
    plt.grid(b=None, which='major', axis='both')
    fig.tight_layout()


    output = output_path + title.replace(" ", "_") + '.pdf'
    try:
        plt.savefig(output)
        print('saved' + " " + output)
    except:
        print('Could not save to ' + output)
    plt.clf()
    plt.close()




def plotAndSaveImageCombinations(logs, key, title, ylabel, output_path, log_scale = False, legend_location = 'upper left'):
    #print('output path' + " " + output_path + title)
    plt.rcParams.update({'font.size': 14})
    fig = plt.figure(figsize=plt.figaspect(0.75))

    plt.title(title, {'fontsize':18})
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.set_xlabel('Frames', fontsize=14)  # xlabel
    ax1.set_ylabel(ylabel, fontsize=14)  # ylabel

    if log_scale:
        ax1.set_yscale('log')

    plot_colors = ['r', 'b', 'm', 'c', 'g',  'y', 'k',
                   (0.0, 1.0, 0.5, 1.), (1.0, 0.5, 0.0, 1.), (0.5, 1.0, 0.0, 1.), (0.0, 0.5, 1.0, 1.),
                    (0.25, 0.5, 0.25, 1.), (0.25, 0.5, 0.5, 1.), 'b', 'b', 'b', 'b', 'b']

    n = 0
    for k in logs:
        data = logs[k]
        if not data:
            n = n + 1
            continue
        log_dict = data[1]
        error_means = [(d[key]) for d in log_dict]
        frames = [int(d['frame']) for d in log_dict]
        std_deviations = [d['error variance'] for d in log_dict]

        ax1.plot(frames, error_means, color = plot_colors[n], label = data[0]['legend name'])

        n = n+1

    ax1.set_xlim([0, max(frames)])
    if title == 'paperbag':
        ax1.set_ylim([0, 0.00085])
    elif title == 'puppet':
        ax1.set_ylim([0, 0.0006])
    elif title == 'hand':
        ax1.set_ylim([0, 0.0004])
    elif title == 'head':
        ax1.set_ylim([0, 0.0001])

    x_step_size = 10. if max(frames) < 75 else 20.
    plt.xticks(np.arange(min(frames)-1, max(frames) + 1, x_step_size))

    plt.legend(loc=legend_location, prop={'size': 14})
    plt.grid(b=None, which='major', axis='both')
    fig.tight_layout()


    output = output_path + title.replace(" ", "_") + '.pdf'
    try:
        plt.savefig(output)
        print('saved' + " " + output)
    except:
        print('Could not save to ' + output)
    plt.clf()
    plt.close()




if __name__=='__main__':
    main()


