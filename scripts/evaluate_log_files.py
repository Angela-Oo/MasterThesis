from parse_log_files import parseAndClusteredLogFiles


def improvementRelativeToBaselineDataset(dataset):
    baseline = dataset['arap']
    if not baseline:
        return False
    baseline_mean = baseline[0]['mean error']
    baseline_mean_per_node = baseline[0]['mean per node']
    dataset['arap'][0]['improvement'] = 0.
    dataset['arap'][0]['improvement per node'] = 0.

    for key in dataset:
        if key != 'arap' and dataset[key]:
            data = dataset[key][0]
            mean = data['mean error']
            mean_per_node = data['mean per node']
            improvement = 1 - (mean / baseline_mean)
            improvement_in_percent = improvement * 100.
            dataset[key][0]['improvement'] = improvement_in_percent
            dataset[key][0]['improvement per node'] = (1. - (mean_per_node / baseline_mean_per_node)) * 100.
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
        for d in data:
            improvement += d[0]['improvement']
            improvement_per_node += d[0]['improvement per node']
        improvement /= len(data)
        improvements[key] = { "improvement" : improvement, "improvement per node" : improvement_per_node, "name" : data[0][0]['name']}
    return improvements


def createTable(parsed_logs):
    header = ("{:<10} {:<22} {:<10} {:<8} {:<10} {:<14} {:<14} {:<14} {:<18} {:<14} {:<8} {:<11}"
              .format('dataset', 'name', 'sequence', 'nodes', 'time', 'error mean', 'median', 'variance',
                      'mean per node', 'improvement \%','edge len', 'vertex prob'))

    table = []
    table.append(header)

    for key in parsed_logs:
        dataset = parsed_logs[key]
        for k in dataset:
            log_dict = dataset[k]
            if log_dict:
                log_dict = log_dict[0]
                column = ("{:<10} {:<22} {:<10} {:<8} {:<10} {:<14} {:<14} {:<14} {:<18} {:<14} {:<8} {:<11}"
                          .format(log_dict['input'],
                                  log_dict['name'],
                                  log_dict['sequence'],
                                  "{:.1f}".format(log_dict['number nodes']),
                                  log_dict["time"],
                                  "{:.2f} e-04".format(log_dict['mean error'] * 1000),
                                  "{:.2f} e-06".format(log_dict['median error'] * 100000),
                                  "{:.2f} e-04".format(log_dict['variance error'] * 1000),
                                  "{:.4f}".format(log_dict["mean per node"]),
                                  "{:.2f}".format(log_dict["improvement"]),
                                  log_dict['edge length'],
                                  log_dict['vertex probability'])
                          )
                table.append(column)
    return table



def createImprovementTable(variants):
    header = ("{:<22} {:<10} {:<10}"
              .format('name', 'improvements %', 'improvements per node %'))

    table = []
    table.append(header)

    for key in variants:
        dataset = variants[key]
        column = ("{:<22} {:<10} {:<10}"
                  .format(dataset['name'],
                          "{:.2f}".format(dataset["improvement"]),
                          "{:.2f}".format(dataset["improvement per node"])
                          ))
        table.append(column)
    return table


path = "../images/run_2019_09_28"

parsed_logs = parseAndClusteredLogFiles(path)

improvementRelativeToBaseline(parsed_logs)
table = createTable(parsed_logs)

improvements = totalImprovementOfVariants(parsed_logs)
improve_table = createImprovementTable(improvements)
#log_files = getAllLogFiles(path)


print('')
for c in table:
    print(c)

print('')
for c in improve_table:
    print(c)

#file = open(path + "\\result.txt", "w")
#for c in table:
#    file.write(c + '\n')
#file.close()