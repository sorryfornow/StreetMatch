import csv

# 输入和输出文件名
input_filename = 'DataFile_2020_10_v2.csv'
output_filename = 'DataFile_2020_10_v2_with_index.txt'

with open(input_filename, 'r', newline='') as infile, open(output_filename, 'w', newline='') as outfile:
    reader = csv.reader(infile)
    #输出为txt
    writer = outfile

    # 删除原本的表头
    next(reader)
    # TODO：在输出文件中包含一个标题行,以;分割
    writer.write('pointID;tripID;xGPS;yGPS;UnixTime;heading;speed;accuracy\n')

    # 读取每一行，并在前面加上行数
    for index, row in enumerate(reader, start=1):  # start=1 从1开始计数
        # 修建每一行的空格
        row = [item.strip() for item in row]
        # 在每一行的前面加上行数
        writer.write(f'{index};{";".join(row)}\n')

# 对比输入输出文件行数是否一致
with open(input_filename, 'r', newline='') as infile:
        reader = csv.reader(infile)
        input_rows = list(reader)
        print(f'Input file has {len(input_rows)} rows')

with open(output_filename, 'r', newline='') as outfile:
        reader = csv.reader(outfile)
        output_rows = list(reader)
        print(f'Output file has {len(output_rows)} rows')

