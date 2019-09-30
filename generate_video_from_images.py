import os
import re
import imageio
from itertools import islice
from PIL import Image
from skimage import transform,io
import cv2


def generateVideo(path, output_name):
    sorted_files = []
    for file in os.listdir(path):
        if file.startswith('frame') and "rigid" not in file:
            complete_path = path + '/' + file
            try:
                img = Image.open(complete_path)  # open the image file
                img.verify()  # verify that it is, in fact an image
                m = re.search('frame_(\d*?)_', file)
                if not m:
                    print("key not found")
                else:
                    sorted_files.append((int(m.group(1)), complete_path))
            except (IOError, SyntaxError) as e:
                print('Bad file:', complete_path)

    print('output video: ' + output_name)
    writer = imageio.get_writer(output_name, fps=4)

    sorted_files = sorted(sorted_files, key=lambda x: x[0])
    for (k, im) in sorted_files:
        #print(str(k) + " " + im)
        img = imageio.imread(im)
        img = cv2.resize(img, (1600, 912), interpolation = cv2.INTER_AREA)
        writer.append_data(img)
    writer.close()

path = "images\\run_2019_09_28"

image_dirs = []
# r=root, d=directories, f = files
for r, d, f in os.walk(path):
    for dir in d:
        if 'color' in dir:
            image_dirs.append(os.path.join(r, dir))



for image_dir in image_dirs:
    output_file = image_dir.replace(path + '\\', '')
    output_file = path + '\\' + output_file.replace('\\', '_') + '.avi'
    print(output_file)
    generateVideo(image_dir, output_name = output_file)


