import pydicom
from PIL import Image
import numpy as np

def dicom_to_png(dicom, png):

    dicom_img = pydicom.dcmread(dicom)
    pixel_array = dicom_img.pixel_array
    
    norm_array = (pixel_array - np.min(pixel_array)) / (np.max(pixel_array) - np.min(pixel_array)) * 255
    norm_array = norm_array.astype(np.uint8)

    img = Image.fromarray(norm_array)
    img.save(png)

    print("Saved Image")

dicom = 'IM00005.dcm'
png = 'IM05-as-png.png'

dicom_to_png(dicom, png)

