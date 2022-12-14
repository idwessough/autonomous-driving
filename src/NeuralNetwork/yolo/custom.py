import os
import pandas as pd

actual_path = os.getcwd()
file_list = os.listdir()
listing = [actual_path+file for file in file_list]
final_list=[]
for file in listing: 
    if file.endswith(".jpg"):
        final_list.append(file)
final_df = pd.DataFrame(final_list) 
final_df.to_csv("files_which_ends_with_jpeg.csv", index=False)