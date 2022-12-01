
import torch
import os

from PIL import Image
from IPython.display import Image, display
# Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5n - yolov5x6, custom
# Images
actual_path = os.getcwd() 
image_folder = os.path.join(actual_path, "images") # 'https://ultralytics.com/images/zidane.jpg'  # or file, Path, PIL, OpenCV, numpy, list  
images = os.listdir(image_folder)
print(images)
for image in images:
    image_path = os.path.join(image_folder, image)
    #image_file = Image.open(image_path)
    display(Image(filename=image_path))
    # Inference
    results = model(image_path)
    # Results
    results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
    #time.sleep(4)