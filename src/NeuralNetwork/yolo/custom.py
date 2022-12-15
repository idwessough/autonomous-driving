import os
import pandas as pd

actual_path = os.getcwd() 
file_list = os.listdir()
listing = [os.path.join(actual_path, str(file)[0:8]+".jpg") for file in file_list] 
print(listing) 
final_df = pd.DataFrame(listing) 
print(final_df)#final_df.to_csv("files_which_ends_with_jpeg.csv", index=False)