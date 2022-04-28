#!/usr/bin/env python3
# limit the number of cpus used by high performance libraries
import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import sys
sys.path.insert(0, './yolov5')

import argparse
import os
import platform
import shutil
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn

from yolov5.models.experimental import attempt_load
from yolov5.utils.downloads import attempt_download
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.datasets import LoadImages, LoadStreams
from yolov5.utils.general import (LOGGER, check_img_size, non_max_suppression, scale_coords, 
                                  check_imshow, xyxy2xywh, increment_path)
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.plots import Annotator, colors
from deep_sort.utils.parser import get_config
import pdb
from deep_sort.deep_sort import DeepSort
import rospy
from detection_only.msg import Bbox_6, Bbox6Array, Track_6, Track6Array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import numpy as np
from csv import writer

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 deepsort root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

##### Trajectory code #####
def draw_grtr(img, positions, width):
    for t in range(len(positions) - 1):
        cv2.line(img, (round(positions[t][0]), round(positions[t][1])), \
                 (round(positions[t + 1][0]), round(positions[t + 1][1])), \
                 color=(255,255,255), thickness=width, lineType=cv2.LINE_AA)

# Each time as a single loop/single frame
# Add frame signal inside
def cleaning_traj_data(tracking_output, frame): 
    # 0-3: x1,y1,x2,y2   4: id   5:cls 
    # <-> Original: 
    # frame(0), class(1), ID(2), x1(3), y1(4), w(5), h(6)
    tracking_data_ped = []
    #pdb.set_trace()
    for e in tracking_output:
        if e[5] == 0:
            tracking_data_ped.append(e)
    # 0: ped, 2: vehicle
    # Delete cls with vehicle
    traj_data = []

    if len(tracking_data_ped) == 0:
        return np.array(traj_data)

    tracking_data_ped = np.delete(tracking_data_ped, 5, axis=1) # frame(timestamp), class, ID, x1, y1, x2, y2
    #tracking_data = tracking_data_ped[:, :4].copy() # frame(timestamp), ID, x1, y1
    
    # x,y,w,h
    tracking_data_ped[:, 2] = tracking_data_ped[:, 2] - tracking_data_ped[:, 0]
    tracking_data_ped[:, 3] = tracking_data_ped[:, 3] - tracking_data_ped[:, 1]
    tracking_data_ped[:, 0] = tracking_data_ped[:, 0] + (tracking_data_ped[:, 2]) / 2 
    tracking_data_ped[:, 1] = tracking_data_ped[:, 1] + tracking_data_ped[:, 3] #  # frame, ID, xc, y2
    # id move to the front (insert to each list)
    tracking_data_ped_list = tracking_data_ped.tolist()

    if len(tracking_data_ped) > 0:
        for l in tracking_data_ped_list:
            l.insert(0, l[4])
            l.pop(5)
            l.insert(0, frame)
            traj_data.append(l)


    # frame, ID, xc, y2, w, h
    return np.array(traj_data)





def create_traj_data(frame_counter, predict_len, forget_frames, frame, extra_length=2.0):  
    global tmp_tracking_agent, tmp_disappear_id

    ######## slim version ##########
    agent_now = []
    for a in frame_counter[-1]:
        agent_now.append(a)
    for agent in agent_now:
        frame = agent[0]
        id = agent[1]
        if id not in tmp_tracking_agent.keys():
            tmp_tracking_agent[id] = np.expand_dims(agent, axis=0)
        else:
            # pdb.set_trace()
            tmp_tracking_agent[id] = np.concatenate((tmp_tracking_agent[id], np.expand_dims(agent, axis=0)))

    pop = False
    pop_keys = []
    for key, track_his in tmp_tracking_agent.items():
        if len(track_his) == 0:
            pop = True
            pop_keys.append(key)
            continue
        else:
            tmp_track_his = []
            # pdb.set_trace()
            for his in track_his:
                if his[0] > frame - 5:
                    tmp_track_his.append(his)
            # pdb.set_trace()
            tmp_tracking_agent[key] = np.array(tmp_track_his)
    if pop:
        for key in pop_keys:
            tmp_tracking_agent.pop(key,None)
    # print("tmp: \n",tmp_tracking_agent)
    ######### slim version ##########


    ########slim################
    tmp_traj_data = {}
    for agent in agent_now:
        frame = agent[0]
        id = agent[1]
        if (tmp_tracking_agent[id][-1][0]-tmp_tracking_agent[id][0][0]+2) >= predict_len:
            start_traj_frame = frame
            frames = np.arange(start_traj_frame, start_traj_frame+predict_len)
            ids = np.repeat(id, predict_len)
            y = tmp_tracking_agent[id][:,3] # past ys
            x = tmp_tracking_agent[id][:,2] # past xs
            parameter = np.polyfit(x, y, 1) # create extrapolate function
            p = np.poly1d(parameter)
            xs = np.linspace(x[-1],x[-1]+extra_length*(x[-1]-x[0]), predict_len) # extrapolate 1.5x length of past trajectory
            ys = p(xs)
            if start_traj_frame not in tmp_traj_data.keys():
                tmp_traj_data[start_traj_frame] = np.expand_dims(np.transpose(np.stack((frames, ids, xs, ys)), (1,0)), axis=0)
            else:
                tmp_new_id_traj = np.expand_dims(np.transpose(np.stack((frames, ids, xs, ys)), (1,0)), axis=0)
                tmp_traj_data[start_traj_frame] = np.concatenate((tmp_traj_data[start_traj_frame], tmp_new_id_traj)) # new id  
    #######slim#################

    # print("tmp_traj_data: \n", tmp_traj_data)
    return tmp_traj_data

def create_traj_data1(frame_counter, predict_len, forget_frames, extra_length=2.0):  
    # count frame_id? agent_id?  
    #frame_ids, counts = np.unique(frame_counter[0, :, 0], return_counts=True)

    tracking_agent = {}
    traj_data = {}

    #index = 0
    disappear_id = {}



    '''
    for i, t in enumerate(frame_counter): # frame_ids
        agent_in_the_frame = t
        # Stop tracking an agent when it does not appear in near frames (forget_frames)
        pop_list = []
        for key, value in tracking_agent.items():
            if key not in agent_in_the_frame[:, 1]: # ID_list -> !!!
                if key not in disappear_id.keys():
                    disappear_id[key] = 1
                else:
                    disappear_id[key] += 1
                if disappear_id[key] >= forget_frames:
                    pop_list.append(key)
            else:
                if key not in disappear_id.keys():
                    pass
                else:
                    disappear_id.pop(key)
        for key in pop_list:
            tracking_agent.pop(key)
        # Start or continuous tracking agents in this frame
        for agent in agent_in_the_frame:
            frame = agent[0]
            id = agent[1]
            if id not in tracking_agent.keys():
                tracking_agent[id] = np.expand_dims(agent, axis=0)
            else:
                tracking_agent[id] = np.concatenate((tracking_agent[id], np.expand_dims(agent, axis=0)))

                # Output trajectory data when continuous tracking exceeds 20 frames (predict_len=20)
                if (tracking_agent[id][-1][0]-tracking_agent[id][0][0]+1) >= predict_len:
                    start_frame = frame
                    frames = np.arange(start_frame, start_frame+predict_len)
                    ids = np.repeat(id, predict_len)
                    x = tracking_agent[id][:][:,2] # past xs
                    y = tracking_agent[id][:][:,3] # past ys
                    parameter = np.polyfit(x, y, 1) # create extrapolate function
                    p = np.poly1d(parameter)
                    xs = np.linspace(x[-1],x[-1]+extra_length*(x[-1]-x[0]), predict_len) # extrapolate 1.5x length of past trajectory
                    ys = p(xs)

                    if start_frame not in traj_data.keys():
                        traj_data[start_frame] = np.expand_dims(np.transpose(np.stack((frames, ids, xs, ys)), (1,0)), axis=0)
                    else:
                        new_id_traj = np.expand_dims(np.transpose(np.stack((frames, ids, xs, ys)), (1,0)), axis=0)

                        traj_data[start_frame] = np.concatenate((traj_data[start_frame], new_id_traj)) # new id         
                    tracking_agent[id] = tracking_agent[id][1:]          
    '''
    #for i, t in enumerate(frame_counter): # frame_ids
    agent_in_the_frame = []
    for f in frame_counter:
        for a in f:
            agent_in_the_frame.append(a)
            # pdb.set_trace()
        
    # Stop tracking an agent when it does not appear in near frames (forget_frames)
    pop_list = []
    for key, _ in tracking_agent.items():
        if key not in agent_in_the_frame[:, 1]: # ID_list -> !!!
            if key not in disappear_id.keys():
                disappear_id[key] = 1
            else:
                disappear_id[key] += 1
            if disappear_id[key] >= forget_frames:
                pop_list.append(key)
        else:
            if key not in disappear_id.keys():
                pass
            else:
                disappear_id.pop(key)

    print(disappear_id)
    for key in pop_list:
        tracking_agent.pop(key)
    # Start or continuous tracking agents in this frame
    for agent in agent_in_the_frame:
        frame = agent[0]
        id = agent[1]
        if id not in tracking_agent.keys():
            tracking_agent[id] = np.expand_dims(agent, axis=0)
        else:
            tracking_agent[id] = np.concatenate((tracking_agent[id], np.expand_dims(agent, axis=0)))
            # Output trajectory data when continuous tracking exceeds 20 frames (predict_len=20)
            if (tracking_agent[id][-1][0]-tracking_agent[id][0][0]+1) >= predict_len:
                start_frame = frame
                frames = np.arange(start_frame, start_frame+predict_len)
                ids = np.repeat(id, predict_len)
                # y = tracking_agent[id][:][:,3] # past ys
                # x = tracking_agent[id][:][:,2] # past xs
                y = tracking_agent[id][:,3] # past ys
                x = tracking_agent[id][:,2] # past xs
                # pdb.set_trace()
                parameter = np.polyfit(x, y, 1) # create extrapolate function
                p = np.poly1d(parameter)
                xs = np.linspace(x[-1],x[-1]+extra_length*(x[-1]-x[0]), predict_len) # extrapolate 1.5x length of past trajectory
                ys = p(xs)
                # pdb.set_trace()
                if start_frame not in traj_data.keys():
                    traj_data[start_frame] = np.expand_dims(np.transpose(np.stack((frames, ids, xs, ys)), (1,0)), axis=0)
                else:
                    new_id_traj = np.expand_dims(np.transpose(np.stack((frames, ids, xs, ys)), (1,0)), axis=0)
                    traj_data[start_frame] = np.concatenate((traj_data[start_frame], new_id_traj)) # new id         
                tracking_agent[id] = tracking_agent[id][1:]  
        #index += counts[i]
    print("tracking_agent: \n",tracking_agent)
    tmp_tracking_agent = tracking_agent
    for key, track_his in tmp_tracking_agent.items():
        tmp_track_his = []
        if track_his == []:
            tracking_agent.pop(key,None)
        else:
            for his in track_his:
                if his[0] > seen - 5:
                    tmp_track_his.append(his)
            track_his = tmp_track_his

    print("tmp: \n",tmp_tracking_agent)

    return traj_data


thetas = np.linspace(-0.2, 0.2, num=20)
def traj_diversify(traj_data, n_samples): # Expand the single path into a sector
    diverse_traj_data = {}
    for key, value in traj_data.items():
        diverse_traj_data[key] = [[] for i in range(n_samples)]

        for agent_data in value:
            agent_velocity_data = agent_data[1:, 2:4] - agent_data[:-1, 2:4] # x_t+1 - x_t, y_t+1 - y_t

            for i in range(n_samples):
                theta = thetas[i]

                rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
                rotated_velocity = np.matmul(agent_velocity_data, rotation_matrix)
                rotated_traj = agent_data[0, 2:4] + np.cumsum(rotated_velocity, axis=0) # a = array([[1 2 3], [4 5 6]]) np.cumsum(a) = array([1 3 6 10 15 21]) np.cumsum(a, axis=0) = array([1 2 3], [5 7 9])
                rotated_traj = np.concatenate((np.expand_dims(agent_data[0, 2:4], 0), rotated_traj), axis=0) # t1~t -> t0~t

                rotated_agent_data = agent_data.copy()
                rotated_agent_data[:, 2:4] = rotated_traj
                diverse_traj_data[key][i].append(rotated_agent_data)

    return diverse_traj_data


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # copy from utils/augmentations.py
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2
    

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)

def track(msg):
    global out, source, yolo_model, deep_sort_model, show_vid, save_vid, save_txt, imgsz, evaluate, half, project, name, exist_ok, device_g
    global config_deepsort, save_img
    global webcam, deepsort, save_dir, model, dataset, names, txt_path, dt, seen
    global img_idx, frame_counter
    img_idx +=1
    traj_intime = None
    traj_outtime = None
    
    track_start = time.time()
    
    # subcribe msg
    det_results = msg.bboxes
    raw_image = msg.image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(raw_image, desired_encoding='passthrough')
    pred = []
    for i in range(len(det_results)):
        pred_sim = []
        pred_sim.append(det_results[i].x1)
        pred_sim.append(det_results[i].y1)
        pred_sim.append(det_results[i].x2)
        pred_sim.append(det_results[i].y2)
        pred_sim.append(det_results[i].conf)
        pred_sim.append(det_results[i].cls)
        pred.append(pred_sim)
    
    pred = [torch.FloatTensor(pred).to(device)]

    
    path = str(img_idx) + ".jpg"
    # Read image
    im0s = cv_image[:,:,[2,1,0]]

    # Padded resize
    img_size = 640
    stride=32
    auto = True
    
    img = letterbox(im0s, img_size, stride=stride, auto=auto)[0]
    # Convert
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)


    

    # Process detections
    for i, det in enumerate(pred):  # detections per image # since len(pred) = 1, det = pred
        seen += 1

        p, im0= str(path), im0s.copy()

        save_path = str(save_dir / p)

        annotator = Annotator(im0, line_width=2, pil=not ascii)

        #frame = 1 # timestamp
        if det is not None and len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            xywhs = xyxy2xywh(det[:, 0:4])
            # (12,4)
            confs = det[:, 4]
            # (12,1)
            clss = det[:, 5]
            # (12,1)

            # pass detections to deepsort
            track_intime = time.time()
            outputs = deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), im0)
            # x1, y1, x2, y2, ID, cls
            track_outtime = time.time()
            # print(outputs)


            ##### Trajectory code #####
            predict_len = 6
            forget = 3 
            traj_data = []
            diverse_traj_data = []
            blur_size = 3
            ###########

            # draw boxes for visualization
            vis_start = 0
            vis_finish = 0
            if len(outputs) > 0:
                vis_start = time.time()

                ###### Trajectory code #####
                ### Simple way first, for each id that appears in 6 continous frames
                ### Only the start 6 frames have problem, skip those instead (counter)
                ### queue and pop 6 frames, while 6 frames, do enumerate, then pop the oldest
                frame_traj_data = cleaning_traj_data(outputs, seen) # seen is frame here
                frame_counter.append(frame_traj_data)
                if len(frame_counter) == 6:
                    # pdb.set_trace()
                    traj_data = create_traj_data(frame_counter, predict_len, seen, forget)
                    diverse_traj_data = traj_diversify(traj_data, n_samples=20)
                    annotator.draw_trajectory(diverse_traj_data, blur_size, outputs, seen,(0,0,255))
                ###########

                for j, (output, conf) in enumerate(zip(outputs, confs)):
                    bboxes = output[0:4]
                    id = output[4]
                    cls = output[5]

                    c = int(cls)  # integer class
                    label = f'{id} {names[c]} {conf:.2f}'
                    annotator.box_label(bboxes, label, color=colors(c, True))
                
                vis_finish = time.time()
                ###### Trajectory code #####
                if len(frame_counter) == 6:
                    frame_counter.pop(0)
                ###########
                

        else:
            deepsort.increment_ages()
            LOGGER.info('No detections')


        # Stream results
        im0 = annotator.result()
        if show_vid:
            cv2.imshow(str(p), im0)
            print("show image")
            if cv2.waitKey(1) == ord('q'):  # q to quit
                raise StopIteration

        track_image = bridge.cv2_to_imgmsg(im0)
        track_pub.publish(track_image)
        if save_img: 
            cv2.imwrite(os.path.join(save_dir , '{}.jpg'.format(str(img_idx))), im0)
    

    track_finish = time.time()
    global track_cost_array
    track_cost_array.append(track_finish - track_start)
    print("tracking cost: ", track_finish - track_start)
    print("average tracking cost: ", sum(track_cost_array) / len(track_cost_array))

    if save_txt or save_vid:
        print('Results saved to %s' % save_path)
        if platform == 'darwin':  # MacOS
            os.system('open ' + save_path)
    
    
    


def det_cb(msg, opt):
    print("det_cb")
    det_data = msg
    opt = opt
    with torch.no_grad():
        track(det_data)

def img_cb(msg):
    global raw_image, image_change
    raw_image = msg

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--yolo_model', nargs='+', type=str, default='yolov5m.pt', help='model.pt path(s)')
    parser.add_argument('--deep_sort_model', type=str, default='osnet_x0_25')
    parser.add_argument('--source', type=str, default='0', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.3, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--show-vid', action='store_true', help='display tracking video results')
    parser.add_argument('--save-vid', action='store_true', help='save video tracking results')
    parser.add_argument('--save-img', action='store_true', help='save img tracking results(for img input)')
    parser.add_argument('--save-txt', action='store_true', help='save MOT compliant results to *.txt')
    # class 0 is person, 1 is bycicle, 2 is car... 79 is oven
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 16 17')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--evaluate', action='store_true', help='augmented inference')
    parser.add_argument("--config_deepsort", type=str, default="/home/ziyan/Yolov5_DeepSort_Pytorch_ros/Yolov5_DeepSort_Pytorch/deep_sort/configs/deep_sort.yaml")
    parser.add_argument("--half", action="store_true", help="use FP16 half-precision inference")
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detection per image')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--project', default=ROOT / 'runs/track', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand

    global out, source, yolo_model, deep_sort_model, show_vid, save_vid, save_txt, imgsz, evaluate, half, project, name, exist_ok, device_g
    global config_deepsort, save_img
    global deepsort
    global raw_image, img_idx, image_change
    global track_cost_array
    track_cost_array = []
    out, source, yolo_model, deep_sort_model, show_vid, save_vid, save_txt, imgsz, evaluate, half, project, name, exist_ok, device_g,\
        config_deepsort= \
        opt.output, opt.source, opt.yolo_model, opt.deep_sort_model, opt.show_vid, opt.save_vid, \
        opt.save_txt, opt.imgsz, opt.evaluate, opt.half, opt.project, opt.name, opt.exist_ok, opt.device, \
        opt.config_deepsort
    save_img = opt.save_img

    webcam = source == '0' or source.startswith(
        'rtsp') or source.startswith('http') or source.endswith('.txt')
    
    # initialize deepsort
    device = select_device(device_g)
    cfg = get_config()
    cfg.merge_from_file(config_deepsort)

    deepsort = DeepSort(deep_sort_model,
                        device,
                        max_dist=cfg.DEEPSORT.MAX_DIST,
                        max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                        max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                        )
    
    # Initialize   
    half &= device.type != 'cpu'  # half precision only supported on CUDA

    # The MOT16 evaluation runs multiple inference streams in parallel, each one writing to
    # its own .txt file. Hence, in that case, the output folder is not restored
    if not evaluate:
        if os.path.exists(out):
            pass
            shutil.rmtree(out)  # delete output folder
        os.makedirs(out)  # make new output folder
    
    # Directories
    global save_dir
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    save_dir.mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(yolo_model, device=device, dnn=opt.dnn)
    global names
    stride, names, pt, jit, _ = model.stride, model.names, model.pt, model.jit, model.onnx
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Half
    half &= pt and device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
    if pt:
        model.model.half() if half else model.model.float()
    
    # Set Dataloader
    global vid_path, vid_writer
    vid_path, vid_writer = None, None
    # Check if environment supports image displays
    if show_vid:
        show_vid = check_imshow()

    # Dataloader
    global dataset
    # if webcam:
    #     show_vid = check_imshow()
    #     cudnn.benchmark = True  # set True to speed up constant image size inference
    #     dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt and not jit)
    #     bs = len(dataset)  # batch_size
    # else:
    #     dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt and not jit)
    #     bs = 1  # batch_size
    # vid_path, vid_writer = [None] * bs, [None] * bs

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names

    # extract what is in between the last '/' and last '.'
    txt_file_name = source.split('/')[-1].split('.')[0]
    global txt_path
    txt_path = str(Path(save_dir)) + '/' + txt_file_name + '.txt'
    # pdb.set_trace()

    if pt and device.type != 'cpu':
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.model.parameters())))  # warmup
    global dt, seen
    dt, seen = [0.0, 0.0, 0.0, 0.0], 0

    img_idx = 0

    # pdb.set_trace()

    # trajectory 
    global frame_counter
    frame_counter = []
    global tmp_tracking_agent, tmp_disappear_id
    tmp_tracking_agent = {}
    tmp_disappear_id = {}

    try:
        while not rospy.is_shutdown():
            rospy.init_node('tracking_node_slim', anonymous=False)
            rate = rospy.Rate(10)
            det_sub = rospy.Subscriber('det_result', Bbox6Array, det_cb, opt, queue_size=1)
            track_pub = rospy.Publisher('track_result_slim', Image, queue_size=1)
            

            rate.sleep()
            rospy.spin()

    except RuntimeError:
        rospy.logfatal("get runtime error")
    except rospy.ROSInterruptException:
        pass