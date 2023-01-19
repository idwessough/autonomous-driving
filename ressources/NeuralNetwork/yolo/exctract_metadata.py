from PIL import Image
from PIL.ExifTags import TAGS

# path to the image or video
imagename = "IMG_2764.jpg"

# read the image data using PIL
image = Image.open(imagename)

# extract EXIF data
exifdata = image.getexif()

# iterating over all EXIF data fields
for tag_id in exifdata:
    # get the tag name, instead of human unreadable tag id
    tag = TAGS.get(tag_id, tag_id)
    data = exifdata.get(tag_id).decode("utf-16")
    print(f"{tag:25}: {data}") 