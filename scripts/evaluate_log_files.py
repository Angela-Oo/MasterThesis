from parse_log_files import getAllLogFiles, parseLogFile



path = "../images/run_2019_09_24"

log_files = getAllLogFiles(path)

for x in log_files:
    print(x)

dicts = []
for log in log_files:
    log_dict = parseLogFile(log)[0]
    dicts.append(log_dict)


header = ("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}"
          .format('Name', 'Sequence', 'Refinement', 'Adaptive', 'Reduce', 'Vertex/Edge', 'Nodes', 'Time', 'Edge Length', 'Vertex prob', 'Error mean', 'median', 'variance'))

table =[]
table.append(header)

for log_dict in dicts:
    column = ("{:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}"
          .format(log_dict['name'],
                  log_dict['sequence'],
                  log_dict['refinement'],
                  log_dict["adaptive rigidity"],
                  log_dict["reduce rigidity"],
                  log_dict["refine at"],
                  log_dict["nodes"],
                  log_dict["time"],
                  log_dict['edge length'],
                  log_dict['vertex probability'],
                  log_dict['mean error'],
                  log_dict['median error'],
                  log_dict['variance error']))
    table.append(column)


for c in table:
    print(c)

#file = open(path + "\\result.txt", "w")
#for c in table:
#    file.write(c + '\n')
#file.close()