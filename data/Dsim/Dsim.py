import os, sys
import shutil
import RflyDtrain


def get_cmd(path):
    with open(path, 'r') as file:
        content = file.read()
        cmd = content.split(';')
        return cmd


if __name__ == "__main__":
    
    fmode = '1-hover'

    ulg_dir = sys.path[0] + '/' + fmode + '/ulg' 
    data_dir = sys.path[0] + '/' + fmode + '/data'
    cmd_dir = sys.path[0] + '/' + fmode + '/cmd.txt'
    os.makedirs(data_dir, exist_ok=True)  

    ulg_files = [f for f in os.listdir(ulg_dir) if f.endswith('.ulg')]

    for index, ulg_file in enumerate(ulg_files, start=1):
        index_folder = os.path.join(data_dir, str(index), 'mav1')
        log_folder = os.path.join(index_folder, 'log')
        ntrain_folder = os.path.join(index_folder, 'ntrain data')
        
        os.makedirs(log_folder, exist_ok=True)
        os.makedirs(ntrain_folder, exist_ok=True)

        ulg_src = os.path.join(ulg_dir, ulg_file)
        ulg_dest = os.path.join(log_folder, ulg_file)
        shutil.copy2(ulg_src, ulg_dest)
        os.chdir(log_folder)
        cmd = f"ulog2csv {ulg_file}"
        os.system(cmd)
        print(f'Processed {ulg_file}, saved to {ulg_dest}')

        Dtrain = RflyDtrain.RflyDtrain()
        ctrlSeq = get_cmd(cmd_dir)

        my_file_name = ['sensor_combined_0.csv', 'vehicle_magnetometer_0.csv', 'vehicle_local_position_0.csv']
        my_colums = [
            ['gyro_rad[0]', 'gyro_rad[1]', 'gyro_rad[2]', 'accelerometer_m_s2[0]', 'accelerometer_m_s2[1]', 'accelerometer_m_s2[2]', 'timestamp'],
            ['magnetometer_ga[0]', 'magnetometer_ga[1]', 'magnetometer_ga[2]', 'timestamp'],
            ['x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']
        ]

        for mfile, mcolumn in zip(my_file_name, my_colums):
            Dtrain.get_normal_train_data_with_file_and_column(log_folder, ntrain_folder, ctrlSeq, my_file_name=mfile, my_columns=mcolumn)

        Dtrain.combine_data(ntrain_folder)

    print("All ULG files have been processed.")