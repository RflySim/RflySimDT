import pandas as pd
import os
import numpy as np

class RflyDtrain:
    def __init__(self) -> None:
        pass

    def deShuffle(self, ctrlSeq):
        '''"2,1;1,1,1;2,3,0,0,-1;1,1,5;2,10,2,3;2,6;1,1,5;2,10,2,6;2,5,0.2,0,0;1,1,15;2,10,2,5;2,7;1,1,5;2,10,2,7"'''
        new = [ca for ca in ctrlSeq if ca.startswith('2,10')]
        result = []
        for ca in new:
            temp = ca.split(',')
            r = f"{int(temp[2])}_{int(temp[3])}"
            result.append(r)
        return result

    def get_normal_train_data_with_file_and_column(self, mav_log_dir, save_path, ctrlSeq, my_file_name="sensor_combined_0.csv", my_columns=['gyro_rad[0]', 'gyro_rad[1]', 'gyro_rad[2]']):
        # step1: find timestamp of drone before fault injecting
        all_files = os.listdir(mav_log_dir)
        save_name = my_file_name.split('_0')[0]
        for file_name in all_files:
            if file_name.endswith('vehicle_command_0.csv'):
                vehicle_command_file_path = os.path.join(mav_log_dir, file_name)
            if file_name.endswith(my_file_name):
                my_file_path = os.path.join(mav_log_dir, file_name)

        if not os.path.exists(save_path):
            os.makedirs(save_path)
        timestamp = self.find_timestamp(vehicle_command_file_path, ctrlSeq)

        my_file_df = pd.read_csv(my_file_path)
        columns_to_read = my_columns
        my_file_result_df = pd.DataFrame(columns=columns_to_read + ['DTMode'])
        for key, value in timestamp.items():
            split_my_file_result_df = pd.DataFrame(columns=columns_to_read + ['DTMode'])
            if value is not None:
                mask = (my_file_df['timestamp'] >= value[0]) & (my_file_df['timestamp'] <= value[1])
                temp_df = my_file_df.loc[mask, columns_to_read]
                temp_df['DTMode'] = key
                # 2.2 save split_sensor_combined raw data 
                split_temp_df = temp_df
                split_path = os.path.join(save_path, f'raw_split_{save_name}_mode{key}.csv')
                split_my_file_result_df = pd.concat([split_my_file_result_df, split_temp_df], ignore_index=True)
                split_my_file_result_df.to_csv(split_path, index=False)
                my_file_result_df = pd.concat([my_file_result_df, temp_df], ignore_index=True)
            else:
                continue

        my_file_result_df.to_csv(os.path.join(save_path, f'raw_{save_name}.csv'), index=False)

    def combine_data(self, output_folder):
        suffix = ['mode2_3.csv','mode2_4.csv','mode2_5.csv','mode2_6.csv','mode2_7.csv','mode2_8.csv','mode2_9.csv']
        for sf in suffix:
            matching_files = [f for f in os.listdir(output_folder) if f.endswith(sf)]
            if len(matching_files) > 1:
                dfs = [pd.read_csv(os.path.join(output_folder, f), 
                                   usecols=[col for col in pd.read_csv(os.path.join(output_folder, f)).columns 
                                            if col not in ['timestamp', 'DTMode']]) for f in matching_files]
                resampled_dfs = self.resample_upping(dfs)
                combined_df = pd.concat(resampled_dfs, axis=1)
                output_file = os.path.join(output_folder, f'sync_merge_data_{sf}')
                combined_df.to_csv(output_file, index=False)
                # print(f"Data saved to '{output_file}'")
    
    def resample_upping(self, dfs):
        max_length = max(len(df) for df in dfs)
        resampled_dfs = []
        
        for df in dfs:
            numeric_df = df.select_dtypes(include=[np.number])
            non_numeric_df = df.select_dtypes(exclude=[np.number])
            
            if len(df) < max_length:
                new_index = np.linspace(0, len(df) - 1, max_length)
                
                new_numeric_df = pd.DataFrame(index=new_index)
                for col in numeric_df.columns:
                    new_numeric_df[col] = np.interp(new_index, np.arange(len(numeric_df)), numeric_df[col])
                
                new_non_numeric_df = pd.DataFrame(index=new_index)
                for col in non_numeric_df.columns:
                    new_non_numeric_df[col] = non_numeric_df.iloc[np.floor(new_index).astype(int)][col].values
                
                new_df = pd.concat([new_numeric_df, new_non_numeric_df], axis=1)
                new_df.reset_index(drop=True, inplace=True)
                resampled_dfs.append(new_df)
            else:
                resampled_dfs.append(df.reset_index(drop=True))
        
        return resampled_dfs


    def find_timestamp(self, file_path, ctrlSeq):
        df = pd.read_csv(file_path)
        df_666 = df[df['param7'] == 666]
        df_777 = df[df['param7'] == 777]
        
        deReshuffle = self.deShuffle(ctrlSeq)
        matching_dict = {}
        for mode in deReshuffle:
            matching_dict[mode] = None

        # matching_dict = {
        #     '2_1':None,
        #     '2_2':None,
        #     '2_3':None,
        #     '2_4':None,
        #     '2_5':None,
        #     '2_6':None,
        #     '2_7':None,
        #     '2_8':None,
        #     '2_9':None,
        # }
        
        for _, row_666 in df_666.iterrows():
            param1_666 = row_666['param1']
            param2_666 = row_666['param2']

            matching_row = df_777[
                (df_777['param1'] == param1_666) &
                (df_777['param2'] == param2_666)
            ]
            
            if not matching_row.empty:
                row_777 = matching_row.iloc[0]
                '''
                df_777['param*']: 为飞行工况识别标志位
                df_777['param1'] df_777['param2']:2,1:解锁
                df_777['param1'] df_777['param2']:2,3:起飞
                df_777['param1'] df_777['param2']:2,4:位置
                df_777['param1'] df_777['param2']:2,5:速度
                df_777['param1'] df_777['param2']:2,6:悬停
                df_777['param1'] df_777['param2']:2,7:降落
                df_777['param1'] df_777['param2']:2,8:任务
                df_777['param1'] df_777['param2']:2,9:故障
                '''

                key = f"{int(row_777['param1'])}_{int(row_777['param2'])}"
                value = (row_666['timestamp'], row_777['timestamp'])
                matching_dict[key] = value
        
        return matching_dict

    def find_csv_files(self, base_path):
        mav_dirs = [d for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))]
        sync_files = {}

        for mav_dir in mav_dirs:
            ntrain_path = os.path.join(base_path, mav_dir, 'ntrain data')
            if os.path.exists(ntrain_path):
                files = [f for f in os.listdir(ntrain_path) if f.startswith('sync_merge_data_mode2_') and f.endswith('.csv')]
                for file in files:
                    if file not in sync_files:
                        sync_files[file] = []
                    sync_files[file].append(os.path.join(ntrain_path, file))
        
        return sync_files

    def get_dt_err_data(self, sync_files):
        error_data = {}
        
        for file, paths in sync_files.items():
            if len(paths) == 2:  # Ensure there are exactly two files to compare
                data1 = pd.read_csv(paths[0])
                data2 = pd.read_csv(paths[1])

                # Convert DataFrames to numpy arrays
                data1_np = data1.to_numpy()
                data2_np = data2.to_numpy()
                
                # Determine which data is smaller and needs interpolation
                if len(data1_np) < len(data2_np):
                    data1_np = self.interpolate_data(data1_np, data2_np)
                else:
                    data2_np = self.interpolate_data(data2_np, data1_np)

                # Calculate the difference for all columns
                error = data2_np - data1_np
                error_df = pd.DataFrame(error, columns=data1.columns)
                error_data[file] = error_df
        
        return error_data

    def interpolate_data(self, smaller_data_np, larger_data_np):
        smaller_length = len(smaller_data_np)
        larger_length = len(larger_data_np)
        
        # Interpolation indices
        x_smaller = np.arange(smaller_length)
        x_larger = np.arange(larger_length)
        
        # Prepare an array to hold interpolated data
        interpolated_data_np = np.full_like(larger_data_np, np.nan, dtype=float)
        
        # Perform interpolation for each column
        for col in range(smaller_data_np.shape[1]):
            y_smaller = smaller_data_np[:, col]
            
            # Use numpy's interpolation
            interpolated_column = np.interp(x_larger, x_smaller, y_smaller)
            interpolated_data_np[:, col] = interpolated_column
        
        return interpolated_data_np
    
    def save_dt_error_data(self, error_data, output_path):
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        
        for file, data in error_data.items():
            output_file = os.path.join(output_path, f'error_{file}')
            data.to_csv(output_file, index=False)
    
    def get_and_save_DT(self, basepath, savepath):
        sync_files = self.find_csv_files(basepath)
        error_data = self.get_dt_err_data(sync_files)
        self.save_dt_error_data(error_data, savepath)

    def apply_sliding_window(self, data, window_size, columns):
        window_data = []
        for i in range(0, len(data) - window_size + 1):
            window_avg = data[i:i+window_size][columns].mean()
            window_avg['timestamp'] = data.iloc[i+window_size-1]['timestamp']
            window_data.append(window_avg)
        return pd.DataFrame(window_data)








