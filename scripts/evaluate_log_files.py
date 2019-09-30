from parse_log_files import parseAndClusteredLogFiles


def createTable():
    header = ("{:<10} {:<22} {:<10} {:<8} {:<10} {:<14} {:<14} {:<14} {:<18} {:<8} {:<11}"
              .format('dataset', 'name', 'sequence', 'nodes', 'time', 'error mean', 'median', 'variance',
                      'median per node', 'edge len', 'vertex prob'))

    table = []
    table.append(header)

    for log_dict in dicts:
        column = ("{:<10} {:<22} {:<10} {:<8} {:<10} {:<14} {:<14} {:<14} {:<18} {:<8} {:<11}"
                  .format(log_dict['input'],
                          log_dict['name'],
                          log_dict['sequence'],
                          "{:.1f}".format(log_dict['number nodes']),
                          log_dict["time"],
                          "{:.2f} e-04".format(log_dict['mean error'] * 1000),
                          "{:.2f} e-06".format(log_dict['median error'] * 100000),
                          "{:.2f} e-04".format(log_dict['variance error'] * 1000),
                          "{:.2f}".format(log_dict["mean per node"]),
                          log_dict['edge length'],
                          log_dict['vertex probability'])
                  )
        table.append(column)
    return table

path = "../images/run_2019_09_28"

parsed_logs = parseAndClusteredLogFiles(path)

#log_files = getAllLogFiles(path)

#for x in log_files:
#    print(x)

#dicts = []
#for log in log_files:
#    log_dict = parseLogFile(log)[0]
#    dicts.append(log_dict)


#table = createTable()

#for c in table:
#    print(c)

#file = open(path + "\\result.txt", "w")
#for c in table:
#    file.write(c + '\n')
#file.close()