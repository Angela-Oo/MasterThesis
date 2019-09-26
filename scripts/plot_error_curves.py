
import numpy as np
import matplotlib.pyplot as plt
import math

from parse_log_files import getAllLogFiles, parseLogFile


def main():
    path = "../images/run_2019_09_24"
    #output_path = path + "/plots"


    log_files = getAllLogFiles(path)

    #log_files = getLogFilesWithName(log_files, 'paperbag')
    for x in log_files:
        print(x)

    from collections import defaultdict
    logs_datasets = defaultdict(list)
    for log in log_files:
        parsed_log = parseLogFile(log)
        dataset = parsed_log[0]['input']
        logs_datasets[dataset].append(parsed_log)

    for dataset in logs_datasets:
        output_path = path + "/" + dataset
        plotDataset(logs_datasets[dataset], dataset, output_path)

def getDefault(parsed_logs):
    logs = [log for log in parsed_logs if (log[0]['name'] == 'ARAP' or log[0]['name'] == 'ED')]
    return logs

def getVariant(parsed_logs, key):
    logs = [log for log in parsed_logs if (log[0][key] != '')]
    return logs

def plotDataset(parsed_logs, dataset, output_path):
    baseline_logs = getDefault(parsed_logs)
    adaptive_rigidity_logs = getVariant(parsed_logs, 'adaptive rigidity')
    refinement_logs = getVariant(parsed_logs, 'refinement')
    reduce_smooth_logs = getVariant(parsed_logs, 'reduce smooth')
    reduce_rigidity_logs = getVariant(parsed_logs, 'reduce rigidity')

    plotDatasetMeanAndVariance(baseline_logs + adaptive_rigidity_logs, dataset + " " + 'Adaptive Rigidity', output_path)
    plotDatasetMeanAndVariance(baseline_logs + refinement_logs, dataset + " " + 'Refinement', output_path)
    plotDatasetMeanAndVariance(baseline_logs + reduce_smooth_logs, dataset + " " + 'Reduce Smooth', output_path)
    plotDatasetMeanAndVariance(baseline_logs + reduce_rigidity_logs, dataset + " " + 'Reduce Rigidity', output_path)

    plotDatasetMeanAndVariance(baseline_logs + adaptive_rigidity_logs + refinement_logs + reduce_smooth_logs + reduce_rigidity_logs, dataset, output_path)

def plotDatasetMeanAndVariance(logs, title, output_path):
    plotAndSaveImage(logs, 'error mean', title, 'mean distance error', output_path + "/mean_")
    plotAndSaveImage(logs, 'error median', title, 'median distance error', output_path + "/median_", True)
    plotAndSaveImage(logs, 'mean per node', title, 'mean distance error per node', output_path + "/mean_per_node_")
    plotAndSaveImage(logs, 'median per node', title, 'median distance error per node', output_path + "/median_per_node_", True)



def plotAndSaveImage(logs, key, title, ylabel, output_path, log_scale = False):
    fig = plt.figure(figsize=plt.figaspect(0.5))

    plt.title(title, {'fontsize':16})
    plt.xlabel('Frames')
    plt.ylabel(ylabel)
    ax1 = fig.add_subplot(1, 1, 1)

    if log_scale:
        ax1.set_yscale('log')

    plot_colors = ['r', 'b', 'g', 'm', 'c', 'y', (1.0, 0.5, 0.0, 1.), (0.5, 1.0, 0.0, 1.), (0.0, 1.0, 0.5, 1.), (0.0, 0.5, 1.0, 1.)]

    n = 0
    for log in logs:
        log_dict = log[1]
        error_means = [(d[key]) for d in log_dict]
        frames = [int(d['frame']) for d in log_dict]
        std_deviations = [d['error variance'] for d in log_dict]

        ax1.plot(frames, error_means, color = plot_colors[n], label=log[0]['name'])

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
    plt.savefig(output_path + title + '.png')
    plt.clf()
    plt.close()






def make_10seeds_scatter_plot(logs, title, output_path):
    fig = plt.figure(figsize=plt.figaspect(0.5))

    plt.title(title, {'fontsize':16})
    plt.xlabel('Frames')
    plt.ylabel('Distance Error')
    ax1 = fig.add_subplot(1, 1, 1)

    plot_colors = ['r', 'b', 'g', 'm', 'c', 'y', (1.0, 0.5, 0.0, 1.), (0.5, 1.0, 0.0, 1.), (0.0, 1.0, 0.5, 1.), (0.0, 0.5, 1.0, 1.)]
    #plot_colors = [(1.0, 0.0, 0.0, 0.8), (0.0, 0.0, 1.0, 0.8), (0.0, 1.0, 0.0, 0.8), (0.5,0.5,0.5,0.8)]
    #std_dev_fill_colors = [(0.0, 1.0, 0.0, 0.05), (1.0, 0.0, 0.0, 0.05), (0.0, 0.0, 1.0, 0.05)]

    n = 0
    for log in logs:
        log_dict = log[1]
        error_means = [d['error mean'] for d in log_dict]
        frames = [int(d['frame']) for d in log_dict]
        std_deviations = [d['error variance'] for d in log_dict]

        ax1.plot(frames, error_means, color = plot_colors[n], label=log[0]['name'])

        #lower_std = [max(0, error_means[i]-(std_deviations[i])) for i in range(len(frames))]
        #ax1.plot(frames, lower_std, color=std_dev_line_colors[n], linestyle='dashed')

        #upper_std = [error_means[i]+(std_deviations[i]) for i in range(len(frames))]
        #ax1.plot(frames, upper_std, color=std_dev_line_colors[n], linestyle='dashed')

        #plt.fill_between(steps, lower_std, upper_std, color=std_dev_fill_colors[n])
        n = n+1

    ax1.set_xlim([0, max(frames)])
    plt.xticks(np.arange(min(frames)-1, max(frames) + 1, 10.0))

    plt.legend(loc='upper left', prop={'size': 12})
    plt.grid(b=None, which='major', axis='both')

    fig.tight_layout()
    #plt.show()

    plt.savefig(output_path +"/" + title + '.png')

    #fig.clear()
    #del fig
    plt.clf()
    plt.close('all')



if __name__=='__main__':
    main()


