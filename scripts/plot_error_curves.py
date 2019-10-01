
import numpy as np
import matplotlib.pyplot as plt
import math

from parse_log_files import parseAndClusteredLogFiles


def main():
    path = "../images/run_2019_10_01_smooth_3"
    #output_path = path + "/plots"

    logs_datasets = parseAndClusteredLogFiles(path)

    for dataset in logs_datasets:
        output_path = path + "/" + dataset
        try:
            plotDataset(logs_datasets[dataset], dataset, output_path)
        except:
            print("not able to plot dataset " + dataset)


def plotDataset(dataset, dataset_name, output_path):
    dataset_no_ed = dict()
    for k in dataset:
        if k != 'Embedded Deformation':
            dataset_no_ed[k] = dataset[k]

    dataset_no_refinement = dict()
    for k in dataset_no_ed:
        if 'Refinement' not in k:
            dataset_no_refinement[k] = dataset_no_ed[k]
    #arap = parsed_logs['arap']
    #adaptive_rigidity_edge = parsed_logs['adaptive rigidity edge']
    #adaptive_rigidity_vertex = parsed_logs['adaptive rigidity vertex']
    #refinement_edge = parsed_logs['refinement edge']
    #refinement_vertex = parsed_logs['refinement vertex']
    #reduce_smooth = parsed_logs['reduce smooth']
    #reduce_rigidity = parsed_logs['reduce rigidity']
    for k in dataset:
        if dataset[k]:
            print("log file " + dataset[k][0]['log file'])

    plotDatasetMeanAndVariance(dataset, dataset_name + '_with_ed', output_path)
    plotDatasetMeanAndVariance(dataset_no_ed, dataset_name, output_path)
    plotDatasetMeanAndVariance(dataset_no_refinement, dataset_name + '_no_refinement', output_path)
    #plotDatasetMeanAndVariance([arap_logs, adaptive_rigidity_edge, adaptive_rigidity_vertex, refinement_edge, reduce_smooth, reduce_rigidity], dataset, output_path)

def plotDatasetMeanAndVariance(logs, title, output_path):
    plotAndSaveImage(logs, 'error mean', title, 'chamfer distance mean', output_path + "/mean_")
    plotAndSaveImage(logs, 'error median', title, 'chamfer distance median', output_path + "/median_", True)
    plotAndSaveImage(logs, 'mean per node', title, 'chamfer distance mean per node', output_path + "/mean_per_node_")
    plotAndSaveImage(logs, 'median per node', title, 'chamfer distance median per node', output_path + "/median_per_node_", True)



def plotAndSaveImage(logs, key, title, ylabel, output_path, log_scale = False):
    #print('output path' + " " + output_path + title)
    fig = plt.figure(figsize=plt.figaspect(0.65))

    plt.title(title, {'fontsize':16})
    #plt.xlabel('Frames', {'fontsize':16})
    #plt.ylabel(ylabel, fontsize=16)
    ax1 = fig.add_subplot(1, 1, 1)
    ax1.set_xlabel('Frames', fontsize=12)  # xlabel
    ax1.set_ylabel(ylabel, fontsize=12)  # ylabel

    if log_scale:
        ax1.set_yscale('log')

    plot_colors = ['r', 'b', 'm', 'c', 'g',  (0.0, 1.0, 0.5, 1.), (1.0, 0.5, 0.0, 1.), (0.5, 1.0, 0.0, 1.), (0.0, 0.5, 1.0, 1.), 'y']

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

        ax1.plot(frames, error_means, color = plot_colors[n], label = k)#data[0]['name'])

        #lower_std = [max(0, error_means[i]-(std_deviations[i])) for i in range(len(frames))]
        #ax1.plot(frames, lower_std, color=std_dev_line_colors[n], linestyle='dashed')

        #upper_std = [error_means[i]+(std_deviations[i]) for i in range(len(frames))]
        #ax1.plot(frames, upper_std, color=std_dev_line_colors[n], linestyle='dashed')

        #plt.fill_between(steps, lower_std, upper_std, color=std_dev_fill_colors[n])
        n = n+1

    ax1.set_xlim([0, max(frames)])
    #ax1.set_ylim([0, 0.00001])
    plt.xticks(np.arange(min(frames)-1, max(frames) + 1, 10.0))

    plt.legend(loc='upper left', prop={'size': 12})
    plt.grid(b=None, which='major', axis='both')
    fig.tight_layout()

    output = output_path + title + '.pdf'
    plt.savefig(output)
    print('saved' + " " + output)
    plt.clf()
    plt.close()







if __name__=='__main__':
    main()


