import os
import pandas as pd 
FILENAME_LENGHT = 8
actual_path = os.getcwd() 
file_list = os.listdir()
for file in file_list:
    if file.endswith(".jpg"):
        filename = file[0:FILENAME_LENGHT] + ".jpg"
    elif file.endswith(".txt"):
        filename = file[0:FILENAME_LENGHT] + ".txt"
    os.rename(file, filename)



file_list = os.listdir()
listing = [os.path.join(actual_path, str(file)) for file in file_list] 
print(listing) 
final_df = pd.DataFrame(listing) 
final_df.to_csv("files_which_ends_with_jpeg.csv", index=False) 