from parse_log_files import parseAndClusteredLogFiles


def improvementRelativeToBaselineDataset(dataset):
    baseline = dataset['ARAP']
    if not baseline:
        return False
    baseline_mean = baseline[0]['mean error']
    baseline_median = baseline[0]['median error']
    baseline_mean_per_node = baseline[0]['mean per node']
    dataset['ARAP'][0]['improvement'] = 0.
    dataset['ARAP'][0]['improvement per node'] = 0.
    dataset['ARAP'][0]['relative error'] = 1.
    dataset['ARAP'][0]['relative error median'] = 1.
    dataset['ARAP'][0]['relative error per node'] = 1.

    for key in dataset:
        if key != 'arap' and dataset[key]:
            data = dataset[key][0]
            mean = data['mean error']
            median = data['median error']
            mean_per_node = data['mean per node']
            improvement = 1. - (mean / baseline_mean)
            improvement_in_percent = improvement * 100.
            dataset[key][0]['improvement'] = improvement_in_percent
            dataset[key][0]['improvement per node'] = (1. - (mean_per_node / baseline_mean_per_node)) * 100.
            dataset[key][0]['relative error'] = (mean / baseline_mean)
            dataset[key][0]['relative error median'] = (median / baseline_median)
            dataset[key][0]['relative error per node'] = (mean_per_node / baseline_mean_per_node)
    #return dataset
    return True

def improvementRelativeToBaseline(datasets):
    for key in datasets:
        dataset = datasets[key]
        improvementRelativeToBaselineDataset(dataset)


def totalImprovementOfVariants(datasets):
    from collections import defaultdict
    variants = defaultdict(list)
    for key in datasets:
        dataset = datasets[key]
        for k in dataset:
            if dataset[k]:
                variants[k].append(dataset[k])
    improvements = defaultdict(list)
    for key in variants:
        data = variants[key]
        improvement = 0.
        improvement_per_node = 0.
        relative_error = 0.
        relative_error_median = 0.
        relative_error_per_node = 0.
        variance = 0.
        for d in data:
            improvement += d[0]['improvement']
            improvement_per_node += d[0]['improvement per node']
            relative_error += d[0]['relative error']
            relative_error_median += d[0]['relative error median']
            relative_error_per_node += d[0]['relative error per node']
            variance += d[0]['variance error']
        improvement /= len(data)
        improvement_per_node /= len(data)
        relative_error /= len(data)
        relative_error_median /= len(data)
        relative_error_per_node /= len(data)
        variance /= len(data)
        improvements[key] = { "improvement" : improvement,
                              "improvement per node" : improvement_per_node,
                              "relative error": relative_error,
                              "relative error median": relative_error_median,
                              "relative error per node": relative_error_per_node,
                              "mean variance": variance,
                              "name" : data[0][0]['name']}
    return improvements


def createTable(parsed_logs):
    header = ("{:<10} {:<40} {:<10} {:<8} {:<10} {:<20} {:<14} {:<14} {:<14} {:<18} {:<14} {:<8} {:<11}"
              .format('dataset', 'name', 'sequence', 'nodes', 'time', 'time per frame in s', 'error mean', 'variance', 'num frames',
                      'mean per node', 'improvement \%','improvement per node\%', 'vertex prob'))

    table = []
    table.append(header)

    for key in parsed_logs:
        dataset = parsed_logs[key]
        for k in dataset:
            log_dict = dataset[k]
            if log_dict:
                log_dict = log_dict[0]
                column = ("{:<10} {:<40} {:<10} {:<8} {:<10} {:<20} {:<14} {:<14} {:<14} {:<18} {:<14} {:<8} {:<11}"
                          .format(log_dict['input'],
                                  log_dict['name'],
                                  log_dict['sequence'],
                                  "{:.1f}".format(log_dict['number nodes']),
                                  log_dict["time"],
                                  "{:.2f} s".format(log_dict["time per frame"]),
                                  "{:.2f} e-04".format(log_dict['mean error'] * 1000),
                                  "{:.2f} e-04".format(log_dict['variance error'] * 1000),
                                  log_dict["number frames"],
                                  "{:.4f}".format(log_dict["mean per node"]),
                                  "{:.2f}".format(log_dict["improvement"]),
                                  "{:.2f}".format(log_dict["improvement per node"]),
                                  log_dict['vertex probability'])
                          )
                table.append(column)
    return table



def createTableRelativeError(parsed_logs):
    header = ("{:<10} {:<40} {:<10} {:<10} {:<10} {:<14} {:<14} {:<16} {:<16} {:<16} {:<16}"
              .format('dataset', 'name', 'nodes', 'nodes min', 'nodes max', 'num frames',
                      'rel error', 'rel error node', 'rel error median', 'improvement \%','variance'))

    table = []
    table.append(header)

    for key in parsed_logs:
        dataset = parsed_logs[key]
        for k in dataset:
            log_dict = dataset[k]
            if log_dict:
                log_dict = log_dict[0]
                column = ("{:<10} {:<40} {:<10} {:<10} {:<10} {:<14} {:<14} {:<16} {:<16} {:<16} {:<16}"
                          .format(log_dict['input'],
                                  log_dict['name'],
                                  "{:.1f}".format(log_dict['number nodes']),
                                  "{:.1f}".format(log_dict['min nodes']),
                                  "{:.1f}".format(log_dict['max nodes']),
                                  log_dict["number frames"],
                                  "{:.4f}".format(log_dict["relative error"]),
                                  "{:.4f}".format(log_dict["relative error per node"]),
                                  "{:.4f}".format(log_dict["relative error median"]),
                                  "{:.2f}".format(log_dict["improvement"]),
                                  "{:.4f} e-05".format(log_dict['variance error'] * 10000))
                          )
                table.append(column)
    return table


def createImprovementTable(variants):
    header = ("{:<40} {:<15} {:<15} {:<15} {:<15} {:<15}"
              .format('name','rel error', 'rel error median', 'rel error node', 'improvements %', "mean variance"))

    table = []
    table.append(header)

    for key in variants:
        dataset = variants[key]
        column = ("{:<40} {:<15} {:<15} {:<15} {:<15} {:<15}"
                  .format(dataset['name'],
                          "{:.4f}".format(dataset["relative error"]),
                          "{:.4f}".format(dataset["relative error median"]),
                          "{:.4f}".format(dataset["relative error per node"]),
                          "{:.4f}".format(dataset["improvement"]),
                          "{:.4f} e-04".format(dataset["mean variance"] * 1000.)
                          ))
        table.append(column)
    return table


path = "../images/run_2019_10_08"

parsed_logs = parseAndClusteredLogFiles(path)

improvementRelativeToBaseline(parsed_logs)
table = createTable(parsed_logs)

relative_table = createTableRelativeError(parsed_logs)

improvements = totalImprovementOfVariants(parsed_logs)
improve_table = createImprovementTable(improvements)
#log_files = getAllLogFiles(path)


print('')
for c in table:
    print(c)

print('')
for c in relative_table:
    print(c)

print('')
for c in improve_table:
    print(c)

file = open(path + "\\result.txt", "w")
for c in table:
    file.write(c + '\n')

file.write('\n')
for c in relative_table:
    file.write(c + '\n')

file.write('\n')
for c in improve_table:
    file.write(c + '\n')

file.close()