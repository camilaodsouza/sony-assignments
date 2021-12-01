# Binary to Image Converter
# Read executable binary file and convert it to RGB image
#
# Author: Camila Souza

import math
from PIL import Image

index = 0
binary_values = []
rgb_values = []
width = 320
height = 240

with open("VIDEO001.RGB", 'rb') as fileobject:
    data = fileobject.read(1)
    
    while data != b'':
        binary_values.append(ord(data))
        data = fileobject.read(1)

print("Read binary file")

while (index + 2) < len(binary_values):
    loByte = bin(binary_values[index])[2:].rjust(8, '0')
    hiByte = bin(binary_values[index+1])[2:].rjust(8, '0')
    # Conversion to RGB565
    R = (int(hiByte[0:5],2) * 527 + 23) >> 6
    G = (int(hiByte[5:] + loByte[0:3], 2) * 259 + 33) >> 6
    B = (int(loByte[3:],2) * 527 + 23) >> 6
    index += 2
    rgb_values.append((R, G, B))

size = (width, height)
print("Changed to RGB format")

try: 
    image = Image.new('RGB', size)
    image.putdata(rgb_values)
    image.save("proper_picture.png")
    print("Saved file")
except Exception as err:
    print(err)