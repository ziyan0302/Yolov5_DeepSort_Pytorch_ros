import sys
import pdb
import json
import time

det_file = "/home/ziyan/det2track/Yolov5_DeepSort_Pytorch/sample.json"
with open(det_file, "r") as f:
    # Reading from json file
    total_labels = json.load(f)

pdb.set_trace()