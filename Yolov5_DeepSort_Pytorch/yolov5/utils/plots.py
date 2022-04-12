# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Plotting utils
"""

import math
import os
from copy import copy
from pathlib import Path

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sn
import torch
from PIL import Image, ImageDraw, ImageFont
import pdb

from utils.general import (LOGGER, Timeout, check_requirements, clip_coords, increment_path, is_ascii, is_chinese,
                           try_except, user_config_dir, xywh2xyxy, xyxy2xywh)
from utils.metrics import fitness

# Settings
CONFIG_DIR = user_config_dir()  # Ultralytics settings dir
RANK = int(os.getenv('RANK', -1))
matplotlib.rc('font', **{'size': 11})
matplotlib.use('Agg')  # for writing to files only


class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))


colors = Colors()  # create instance for 'from utils.plots import colors'


def check_font(font='Arial.ttf', size=10):
    # Return a PIL TrueType Font, downloading to CONFIG_DIR if necessary
    font = Path(font)
    font = font if font.exists() else (CONFIG_DIR / font.name)
    try:
        return ImageFont.truetype(str(font) if font.exists() else font.name, size)
    except Exception as e:  # download if missing
        url = "https://ultralytics.com/assets/" + font.name
        print(f'Downloading {url} to {font}...')
        torch.hub.download_url_to_file(url, str(font), progress=False)
        try:
            return ImageFont.truetype(str(font), size)
        except TypeError:
            check_requirements('Pillow>=8.4.0')  # known issue https://github.com/ultralytics/yolov5/issues/5374


class Annotator:
    if RANK in (-1, 0):
        check_font()  # download TTF if necessary

    # YOLOv5 Annotator for train/val mosaics and jpgs and detect/hub inference annotations
    def __init__(self, im, line_width=None, font_size=None, font='Arial.ttf', pil=False, example='abc'):
        assert im.data.contiguous, 'Image not contiguous. Apply np.ascontiguousarray(im) to Annotator() input images.'
        self.pil = pil or not is_ascii(example) or is_chinese(example)
        if self.pil:  # use PIL
            self.im = im if isinstance(im, Image.Image) else Image.fromarray(im)
            self.draw = ImageDraw.Draw(self.im)
            self.font = check_font(font='Arial.Unicode.ttf' if is_chinese(example) else font,
                                   size=font_size or max(round(sum(self.im.size) / 2 * 0.035), 12))
        else:  # use cv2
            self.im = im
        self.lw = line_width or max(round(sum(im.shape) / 2 * 0.003), 2)  # line width

    def box_label(self, box, label='', color=(128, 128, 128), txt_color=(255, 255, 255)):
        # Add one xyxy box to image with label
        if self.pil or not is_ascii(label):
            self.draw.rectangle(box, width=self.lw, outline=color)  # box
            if label:
                w, h = self.font.getsize(label)  # text width, height
                outside = box[1] - h >= 0  # label fits outside box
                self.draw.rectangle([box[0],
                                     box[1] - h if outside else box[1],
                                     box[0] + w + 1,
                                     box[1] + 1 if outside else box[1] + h + 1], fill=color)
                # self.draw.text((box[0], box[1]), label, fill=txt_color, font=self.font, anchor='ls')  # for PIL>8.0
                self.draw.text((box[0], box[1] - h if outside else box[1]), label, fill=txt_color, font=self.font)
        else:  # cv2
            p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
            cv2.rectangle(self.im, p1, p2, color, thickness=self.lw, lineType=cv2.LINE_AA)
            if label:
                tf = max(self.lw - 1, 1)  # font thickness
                w, h = cv2.getTextSize(label, 0, fontScale=self.lw / 3, thickness=tf)[0]  # text width, height
                outside = p1[1] - h - 3 >= 0  # label fits outside box
                p2 = p1[0] + w, p1[1] - h - 3 if outside else p1[1] + h + 3
                cv2.rectangle(self.im, p1, p2, color, -1, cv2.LINE_AA)  # filled
                cv2.putText(self.im, label, (p1[0], p1[1] - 2 if outside else p1[1] + h + 2), 0, self.lw / 3, txt_color,
                            thickness=tf, lineType=cv2.LINE_AA)

    def rectangle(self, xy, fill=None, outline=None, width=1):
        # Add rectangle to image (PIL-only)
        self.draw.rectangle(xy, fill, outline, width)

    def text(self, xy, text, txt_color=(255, 255, 255)):
        # Add text to image (PIL-only)
        w, h = self.font.getsize(text)  # text width, height
        self.draw.text((xy[0], xy[1] - h + 1), text, fill=txt_color, font=self.font)

    def result(self):
        # Return annotated image as array
        return np.asarray(self.im)

    ##### Trajectory code #####
    def draw_grtr(self, img, positions, width):
        for t in range(len(positions) - 1):
            cv2.line(img, (round(positions[t][0]), round(positions[t][1])), \
                     (round(positions[t + 1][0]), round(positions[t + 1][1])), \
                     color=(255,255,255), thickness=width, lineType=cv2.LINE_AA)

    def draw_trajectory(self, diverse_traj_data, blur_size, dets, frame_id):
        ### Ask ###
        w = dets[:, 2] - dets[:, 0] 
        h = dets[:, 3] - dets[:, 1]
        cxs = dets[:, 0] + (w//2)
        y2s = dets[:, 1] + h # 
        ########
        traj_mask = np.zeros((self.im.shape[0], self.im.shape[1], 3))

        # line_mask = np.zeros((npy_line.shape[0], npy_line.shape[1], 3))
        line_mask = np.zeros((self.im.shape[0], self.im.shape[1], 3))
        
        final = None

        bot_max = -1
        for l in range(len(cxs)):
            cx = cxs[l]
            y2 = y2s[l]

            # if cx >= npy_line.shape[1]:
            #     cx = npy_line.shape[1] - 1
            # if y2 > npy_line.shape[0]:
            #     y2 = npy_line.shape[0] - 1

            if cx >= self.im.shape[1]:
                cx = self.im.shape[1] - 1
            if y2 > self.im.shape[0]:
                y2 = self.im.shape[0] - 1
        if frame_id in diverse_traj_data.keys():
            frame_data = diverse_traj_data[frame_id] # n_samples, num_id, timestep, coordinate (20, 2, 25, 4)
            traj_layers = np.zeros((20, self.im.shape[0], self.im.shape[1]), np.uint8)
            for i, sample in enumerate(frame_data): # i: sample_id, sample: different_traj in a frame
                routes = None
                for agent_data in sample:
                    route = agent_data[:, 2:4]
                    self.draw_grtr(traj_layers[i], route, 15) 
                    
            heatmap_layer = np.sum(traj_layers, axis=0).astype(np.uint8)
            heatmap_layer = cv2.blur(heatmap_layer, (blur_size, blur_size))
            heat = heatmap_layer.copy()

            heat[heat>0] = 255
            traj_mask[heat==255] = (0,0,255)
            index_y, index_x = np.where(heat == 255)[0], np.where(heat == 255)[1]
            tra_max = -1
            for iy, ix in zip(index_y, index_x):
                # if line_mask[iy, ix, 1] == 255 and line_mask[iy, ix, 2] == 0 and iy > tra_max:
                if iy > tra_max: # Add new line
                    tra_max = iy
            if tra_max >= 0:
                #line_mask[npy_line==1] = (0,255,0)
                line_mask[:tra_max, :, [1, 2]] = line_mask[:tra_max, :, [2, 1]]
            else:
                line_mask = line_mask

            heat_gray = cv2.cvtColor(traj_mask.astype(np.uint8), cv2.COLOR_BGR2GRAY)
            cnts, hierarchy = cv2.findContours(heat_gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            heatmap_layer = cv2.applyColorMap(heatmap_layer, cv2.COLORMAP_HOT)
            final_1 = cv2.addWeighted(line_mask.astype(np.uint8), 1, self.im, 1, 0)
            final_2 = cv2.addWeighted(traj_mask.astype(np.uint8), 1, self.im, 1, 0)
            final = cv2.addWeighted(final_1, 0.5, final_2, 0.5, 0)
            cv2.drawContours(final, cnts, -1, (0, 0, 0), 1, lineType=cv2.LINE_AA)

        else:
            final = cv2.addWeighted(line_mask.astype(np.uint8), 1, self.im, 1, 0)
            #final_2 = image
            heat = np.zeros((self.im.shape[0], self.im.shape[1]))
        
        #final = cv2.resize(final, (0,0), fx = 0.5, fy = 0.5) 
        self.im = final
    ###########

    ##### Sound Signal code #####
    def addTogether(self, car,x,y, type='icon'):   # img為背景圖 ,car為車子圖 , x,y為圖像在背景的位置
    
        
        rows,cols,channels = car.shape #拆分圖片信息
        #轉換格式 
        img_hsv = cv2.cvtColor(car,cv2.COLOR_RGB2HSV) #把圖片轉換成HSV格式，用於摳圖 
        #摳圖 
        lower_blue=np.array([0,0,0]) #獲取最小閾值 
        #upper_blue=np.array([0,255,255]) #獲取最大閾值 
        upper_color = np.array([0,0,0])
        if type == 'car':
            upper_color = np.array([0,255,255])
        else:
            upper_color = np.array([255,0,0])
        mask = cv2.inRange(img_hsv, lower_blue, upper_color) #創建遮罩 

        erode=cv2.erode(mask,None,iterations=3) #圖像腐蝕 

        dilate=cv2.dilate(erode,None,iterations=1) #圖像膨脹 

        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8))) #開運算 

        #========圖像合併==========================================================================================
        center = [y,x] #設置car在背景圖的起始位置
        for i in range(rows): 
            for j in range(cols): 
                if opening[i,j]==0: #代表黑色
                    self.im[center[0]+i,center[1]+j] =car[i,j] #賦值顏色
    
    def sound_signal(self, data): # Add signal
        #導入車子圖
        #global tmp # Take the global var tmp
        car = cv2.imread('/home/ziyan/Yolov5_DeepSort_Pytorch_ros/Yolov5_DeepSort_Pytorch/icon_imgs/car.png') #car圖片導入
        car = cv2.resize(car,(0,0),fx=0.4,fy=0.4) # 0.15, 0.15

        #導入ambulance
        amb = cv2.imread('/home/ziyan/Yolov5_DeepSort_Pytorch_ros/Yolov5_DeepSort_Pytorch/icon_imgs/type_icon/amb.png') #ambulance圖片導入
        #pdb.set_trace()
        amb = cv2.resize(amb,(0,0),fx=1.2,fy=1.2)
        #pdb.set_trace()

        fire = cv2.imread('/home/ziyan/Yolov5_DeepSort_Pytorch_ros/Yolov5_DeepSort_Pytorch/icon_imgs/type_icon/fire.png') #ambulance圖片導入
        #pdb.set_trace()
        fire = cv2.resize(amb,(0,0),fx=1.2,fy=1.2)

        police = cv2.imread('/home/ziyan/Yolov5_DeepSort_Pytorch_ros/Yolov5_DeepSort_Pytorch/icon_imgs/type_icon/police.png') #ambulance圖片導入
        #pdb.set_trace()
        police = cv2.resize(amb,(0,0),fx=1.2,fy=1.2)

        #================初始化座標值=====================
        x = int(0.1*self.im.shape[1]) #圓的x座標 圖片的寬的倍數 shape=(length,width,channel)
        y = int(0.8*self.im.shape[0]) #圓的y座標
        r =int(0.15*self.im.shape[0]) #圓的半徑

        #================畫出圓形=====================
        cv2.circle(self.im,(x,y),r,(84,46,8),-1)
        m = 3 #畫m個同心圓
        r_circle = r/m
        for i in range(1,m+1):
            cv2.circle(self.im,(x,y),int(r_circle*i),(255,255,255),1)

        #================畫出直線=====================
        #每45度畫一條線
        #直線點座標(x_line,y_line)
        n = 12 #畫出n等份的圓
        degree_line = 360/n #每 degree_line度 就畫線
        #print("degree_line:"+str(degree_line))

        for i in range(n):
            theta_line =  ((i*degree_line)/180)*math.pi #角度轉成弧度
            x_line = int(x + r*math.cos(theta_line))
            y_line = int(y - r*math.sin(theta_line))
            cv2.line(self.im,(x,y),(x_line,y_line),(250,255,255),1)

        #===========================================
        #方位角 角度為n等份的範圍值 (畫扇形:橢圓的長軸與短軸=圓形半徑)
        angle, label_type = data.split(',')
        label_list = [2,3,4]
        if label_type in label_list:
            ### Select a specific signal
            p = (12 - angle // 30) % 12 # 330->11->1, 30->1->11
            #p = 7 # 7*30
            #p = np.random.randint(0,n)  #隨機挑選第n等份的某值 -> Also, the pic being select is not in the correct order (Double random)


            #print("p:"+str(p))
            degree_elli_start = p * degree_line # degree_line=360/n #起始角度
            degree_elli_end = (p+1) * degree_line #終止角度

            # #橢圓(圖片名稱,中心點,(長軸,短軸),旋轉角度（順時針方向）,繪製的起始角度（順時針方向）,繪製的終止角度(順時針方向),線條顏色,線條粗細)
            cv2.ellipse(self.im,(x,y),(r,r),0,-degree_elli_start,-degree_elli_end,(0,215,255),-1)

        # x,y
        self.addTogether(car, x - car.shape[1]//2, y - car.shape[0]//2, type='car')
        
        if label_type == 2: # police
            self.addTogether(police, 2*x,y - police.shape[0]//2)
        elif label_type == 3: # fire
            self.addTogether(fire, 2*x,y - fire.shape[0]//2)
        elif label_type == 4: # amb
            self.addTogether(amb, 2*x,y - amb.shape[0]//2)


    #return draw_img


    ###########

def feature_visualization(x, module_type, stage, n=32, save_dir=Path('runs/detect/exp')):
    """
    x:              Features to be visualized
    module_type:    Module type
    stage:          Module stage within model
    n:              Maximum number of feature maps to plot
    save_dir:       Directory to save results
    """
    if 'Detect' not in module_type:
        batch, channels, height, width = x.shape  # batch, channels, height, width
        if height > 1 and width > 1:
            f = save_dir / f"stage{stage}_{module_type.split('.')[-1]}_features.png"  # filename

            blocks = torch.chunk(x[0].cpu(), channels, dim=0)  # select batch index 0, block by channels
            n = min(n, channels)  # number of plots
            fig, ax = plt.subplots(math.ceil(n / 8), 8, tight_layout=True)  # 8 rows x n/8 cols
            ax = ax.ravel()
            plt.subplots_adjust(wspace=0.05, hspace=0.05)
            for i in range(n):
                ax[i].imshow(blocks[i].squeeze())  # cmap='gray'
                ax[i].axis('off')

            print(f'Saving {f}... ({n}/{channels})')
            plt.savefig(f, dpi=300, bbox_inches='tight')
            plt.close()
            np.save(str(f.with_suffix('.npy')), x[0].cpu().numpy())  # npy save


def hist2d(x, y, n=100):
    # 2d histogram used in labels.png and evolve.png
    xedges, yedges = np.linspace(x.min(), x.max(), n), np.linspace(y.min(), y.max(), n)
    hist, xedges, yedges = np.histogram2d(x, y, (xedges, yedges))
    xidx = np.clip(np.digitize(x, xedges) - 1, 0, hist.shape[0] - 1)
    yidx = np.clip(np.digitize(y, yedges) - 1, 0, hist.shape[1] - 1)
    return np.log(hist[xidx, yidx])


def butter_lowpass_filtfilt(data, cutoff=1500, fs=50000, order=5):
    from scipy.signal import butter, filtfilt

    # https://stackoverflow.com/questions/28536191/how-to-filter-smooth-with-scipy-numpy
    def butter_lowpass(cutoff, fs, order):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        return butter(order, normal_cutoff, btype='low', analog=False)

    b, a = butter_lowpass(cutoff, fs, order=order)
    return filtfilt(b, a, data)  # forward-backward filter


def output_to_target(output):
    # Convert model output to target format [batch_id, class_id, x, y, w, h, conf]
    targets = []
    for i, o in enumerate(output):
        for *box, conf, cls in o.cpu().numpy():
            targets.append([i, cls, *list(*xyxy2xywh(np.array(box)[None])), conf])
    return np.array(targets)


def plot_images(images, targets, paths=None, fname='images.jpg', names=None, max_size=1920, max_subplots=16):
    # Plot image grid with labels
    if isinstance(images, torch.Tensor):
        images = images.cpu().float().numpy()
    if isinstance(targets, torch.Tensor):
        targets = targets.cpu().numpy()
    if np.max(images[0]) <= 1:
        images *= 255  # de-normalise (optional)
    bs, _, h, w = images.shape  # batch size, _, height, width
    bs = min(bs, max_subplots)  # limit plot images
    ns = np.ceil(bs ** 0.5)  # number of subplots (square)

    # Build Image
    mosaic = np.full((int(ns * h), int(ns * w), 3), 255, dtype=np.uint8)  # init
    for i, im in enumerate(images):
        if i == max_subplots:  # if last batch has fewer images than we expect
            break
        x, y = int(w * (i // ns)), int(h * (i % ns))  # block origin
        im = im.transpose(1, 2, 0)
        mosaic[y:y + h, x:x + w, :] = im

    # Resize (optional)
    scale = max_size / ns / max(h, w)
    if scale < 1:
        h = math.ceil(scale * h)
        w = math.ceil(scale * w)
        mosaic = cv2.resize(mosaic, tuple(int(x * ns) for x in (w, h)))

    # Annotate
    fs = int((h + w) * ns * 0.01)  # font size
    annotator = Annotator(mosaic, line_width=round(fs / 10), font_size=fs, pil=True)
    for i in range(i + 1):
        x, y = int(w * (i // ns)), int(h * (i % ns))  # block origin
        annotator.rectangle([x, y, x + w, y + h], None, (255, 255, 255), width=2)  # borders
        if paths:
            annotator.text((x + 5, y + 5 + h), text=Path(paths[i]).name[:40], txt_color=(220, 220, 220))  # filenames
        if len(targets) > 0:
            ti = targets[targets[:, 0] == i]  # image targets
            boxes = xywh2xyxy(ti[:, 2:6]).T
            classes = ti[:, 1].astype('int')
            labels = ti.shape[1] == 6  # labels if no conf column
            conf = None if labels else ti[:, 6]  # check for confidence presence (label vs pred)

            if boxes.shape[1]:
                if boxes.max() <= 1.01:  # if normalized with tolerance 0.01
                    boxes[[0, 2]] *= w  # scale to pixels
                    boxes[[1, 3]] *= h
                elif scale < 1:  # absolute coords need scale if image scales
                    boxes *= scale
            boxes[[0, 2]] += x
            boxes[[1, 3]] += y
            for j, box in enumerate(boxes.T.tolist()):
                cls = classes[j]
                color = colors(cls)
                cls = names[cls] if names else cls
                if labels or conf[j] > 0.25:  # 0.25 conf thresh
                    label = f'{cls}' if labels else f'{cls} {conf[j]:.1f}'
                    annotator.box_label(box, label, color=color)
    annotator.im.save(fname)  # save


def plot_lr_scheduler(optimizer, scheduler, epochs=300, save_dir=''):
    # Plot LR simulating training for full epochs
    optimizer, scheduler = copy(optimizer), copy(scheduler)  # do not modify originals
    y = []
    for _ in range(epochs):
        scheduler.step()
        y.append(optimizer.param_groups[0]['lr'])
    plt.plot(y, '.-', label='LR')
    plt.xlabel('epoch')
    plt.ylabel('LR')
    plt.grid()
    plt.xlim(0, epochs)
    plt.ylim(0)
    plt.savefig(Path(save_dir) / 'LR.png', dpi=200)
    plt.close()


def plot_val_txt():  # from utils.plots import *; plot_val()
    # Plot val.txt histograms
    x = np.loadtxt('val.txt', dtype=np.float32)
    box = xyxy2xywh(x[:, :4])
    cx, cy = box[:, 0], box[:, 1]

    fig, ax = plt.subplots(1, 1, figsize=(6, 6), tight_layout=True)
    ax.hist2d(cx, cy, bins=600, cmax=10, cmin=0)
    ax.set_aspect('equal')
    plt.savefig('hist2d.png', dpi=300)

    fig, ax = plt.subplots(1, 2, figsize=(12, 6), tight_layout=True)
    ax[0].hist(cx, bins=600)
    ax[1].hist(cy, bins=600)
    plt.savefig('hist1d.png', dpi=200)


def plot_targets_txt():  # from utils.plots import *; plot_targets_txt()
    # Plot targets.txt histograms
    x = np.loadtxt('targets.txt', dtype=np.float32).T
    s = ['x targets', 'y targets', 'width targets', 'height targets']
    fig, ax = plt.subplots(2, 2, figsize=(8, 8), tight_layout=True)
    ax = ax.ravel()
    for i in range(4):
        ax[i].hist(x[i], bins=100, label=f'{x[i].mean():.3g} +/- {x[i].std():.3g}')
        ax[i].legend()
        ax[i].set_title(s[i])
    plt.savefig('targets.jpg', dpi=200)


def plot_val_study(file='', dir='', x=None):  # from utils.plots import *; plot_val_study()
    # Plot file=study.txt generated by val.py (or plot all study*.txt in dir)
    save_dir = Path(file).parent if file else Path(dir)
    plot2 = False  # plot additional results
    if plot2:
        ax = plt.subplots(2, 4, figsize=(10, 6), tight_layout=True)[1].ravel()

    fig2, ax2 = plt.subplots(1, 1, figsize=(8, 4), tight_layout=True)
    # for f in [save_dir / f'study_coco_{x}.txt' for x in ['yolov5n6', 'yolov5s6', 'yolov5m6', 'yolov5l6', 'yolov5x6']]:
    for f in sorted(save_dir.glob('study*.txt')):
        y = np.loadtxt(f, dtype=np.float32, usecols=[0, 1, 2, 3, 7, 8, 9], ndmin=2).T
        x = np.arange(y.shape[1]) if x is None else np.array(x)
        if plot2:
            s = ['P', 'R', 'mAP@.5', 'mAP@.5:.95', 't_preprocess (ms/img)', 't_inference (ms/img)', 't_NMS (ms/img)']
            for i in range(7):
                ax[i].plot(x, y[i], '.-', linewidth=2, markersize=8)
                ax[i].set_title(s[i])

        j = y[3].argmax() + 1
        ax2.plot(y[5, 1:j], y[3, 1:j] * 1E2, '.-', linewidth=2, markersize=8,
                 label=f.stem.replace('study_coco_', '').replace('yolo', 'YOLO'))

    ax2.plot(1E3 / np.array([209, 140, 97, 58, 35, 18]), [34.6, 40.5, 43.0, 47.5, 49.7, 51.5],
             'k.-', linewidth=2, markersize=8, alpha=.25, label='EfficientDet')

    ax2.grid(alpha=0.2)
    ax2.set_yticks(np.arange(20, 60, 5))
    ax2.set_xlim(0, 57)
    ax2.set_ylim(25, 55)
    ax2.set_xlabel('GPU Speed (ms/img)')
    ax2.set_ylabel('COCO AP val')
    ax2.legend(loc='lower right')
    f = save_dir / 'study.png'
    print(f'Saving {f}...')
    plt.savefig(f, dpi=300)


@try_except  # known issue https://github.com/ultralytics/yolov5/issues/5395
@Timeout(30)  # known issue https://github.com/ultralytics/yolov5/issues/5611
def plot_labels(labels, names=(), save_dir=Path('')):
    # plot dataset labels
    LOGGER.info(f"Plotting labels to {save_dir / 'labels.jpg'}... ")
    c, b = labels[:, 0], labels[:, 1:].transpose()  # classes, boxes
    nc = int(c.max() + 1)  # number of classes
    x = pd.DataFrame(b.transpose(), columns=['x', 'y', 'width', 'height'])

    # seaborn correlogram
    sn.pairplot(x, corner=True, diag_kind='auto', kind='hist', diag_kws=dict(bins=50), plot_kws=dict(pmax=0.9))
    plt.savefig(save_dir / 'labels_correlogram.jpg', dpi=200)
    plt.close()

    # matplotlib labels
    matplotlib.use('svg')  # faster
    ax = plt.subplots(2, 2, figsize=(8, 8), tight_layout=True)[1].ravel()
    y = ax[0].hist(c, bins=np.linspace(0, nc, nc + 1) - 0.5, rwidth=0.8)
    # [y[2].patches[i].set_color([x / 255 for x in colors(i)]) for i in range(nc)]  # update colors bug #3195
    ax[0].set_ylabel('instances')
    if 0 < len(names) < 30:
        ax[0].set_xticks(range(len(names)))
        ax[0].set_xticklabels(names, rotation=90, fontsize=10)
    else:
        ax[0].set_xlabel('classes')
    sn.histplot(x, x='x', y='y', ax=ax[2], bins=50, pmax=0.9)
    sn.histplot(x, x='width', y='height', ax=ax[3], bins=50, pmax=0.9)

    # rectangles
    labels[:, 1:3] = 0.5  # center
    labels[:, 1:] = xywh2xyxy(labels[:, 1:]) * 2000
    img = Image.fromarray(np.ones((2000, 2000, 3), dtype=np.uint8) * 255)
    for cls, *box in labels[:1000]:
        ImageDraw.Draw(img).rectangle(box, width=1, outline=colors(cls))  # plot
    ax[1].imshow(img)
    ax[1].axis('off')

    for a in [0, 1, 2, 3]:
        for s in ['top', 'right', 'left', 'bottom']:
            ax[a].spines[s].set_visible(False)

    plt.savefig(save_dir / 'labels.jpg', dpi=200)
    matplotlib.use('Agg')
    plt.close()


def plot_evolve(evolve_csv='path/to/evolve.csv'):  # from utils.plots import *; plot_evolve()
    # Plot evolve.csv hyp evolution results
    evolve_csv = Path(evolve_csv)
    data = pd.read_csv(evolve_csv)
    keys = [x.strip() for x in data.columns]
    x = data.values
    f = fitness(x)
    j = np.argmax(f)  # max fitness index
    plt.figure(figsize=(10, 12), tight_layout=True)
    matplotlib.rc('font', **{'size': 8})
    for i, k in enumerate(keys[7:]):
        v = x[:, 7 + i]
        mu = v[j]  # best single result
        plt.subplot(6, 5, i + 1)
        plt.scatter(v, f, c=hist2d(v, f, 20), cmap='viridis', alpha=.8, edgecolors='none')
        plt.plot(mu, f.max(), 'k+', markersize=15)
        plt.title(f'{k} = {mu:.3g}', fontdict={'size': 9})  # limit to 40 characters
        if i % 5 != 0:
            plt.yticks([])
        print(f'{k:>15}: {mu:.3g}')
    f = evolve_csv.with_suffix('.png')  # filename
    plt.savefig(f, dpi=200)
    plt.close()
    print(f'Saved {f}')


def plot_results(file='path/to/results.csv', dir=''):
    # Plot training results.csv. Usage: from utils.plots import *; plot_results('path/to/results.csv')
    save_dir = Path(file).parent if file else Path(dir)
    fig, ax = plt.subplots(2, 5, figsize=(12, 6), tight_layout=True)
    ax = ax.ravel()
    files = list(save_dir.glob('results*.csv'))
    assert len(files), f'No results.csv files found in {save_dir.resolve()}, nothing to plot.'
    for fi, f in enumerate(files):
        try:
            data = pd.read_csv(f)
            s = [x.strip() for x in data.columns]
            x = data.values[:, 0]
            for i, j in enumerate([1, 2, 3, 4, 5, 8, 9, 10, 6, 7]):
                y = data.values[:, j]
                # y[y == 0] = np.nan  # don't show zero values
                ax[i].plot(x, y, marker='.', label=f.stem, linewidth=2, markersize=8)
                ax[i].set_title(s[j], fontsize=12)
                # if j in [8, 9, 10]:  # share train and val loss y axes
                #     ax[i].get_shared_y_axes().join(ax[i], ax[i - 5])
        except Exception as e:
            print(f'Warning: Plotting error for {f}: {e}')
    ax[1].legend()
    fig.savefig(save_dir / 'results.png', dpi=200)
    plt.close()


def profile_idetection(start=0, stop=0, labels=(), save_dir=''):
    # Plot iDetection '*.txt' per-image logs. from utils.plots import *; profile_idetection()
    ax = plt.subplots(2, 4, figsize=(12, 6), tight_layout=True)[1].ravel()
    s = ['Images', 'Free Storage (GB)', 'RAM Usage (GB)', 'Battery', 'dt_raw (ms)', 'dt_smooth (ms)', 'real-world FPS']
    files = list(Path(save_dir).glob('frames*.txt'))
    for fi, f in enumerate(files):
        try:
            results = np.loadtxt(f, ndmin=2).T[:, 90:-30]  # clip first and last rows
            n = results.shape[1]  # number of rows
            x = np.arange(start, min(stop, n) if stop else n)
            results = results[:, x]
            t = (results[0] - results[0].min())  # set t0=0s
            results[0] = x
            for i, a in enumerate(ax):
                if i < len(results):
                    label = labels[fi] if len(labels) else f.stem.replace('frames_', '')
                    a.plot(t, results[i], marker='.', label=label, linewidth=1, markersize=5)
                    a.set_title(s[i])
                    a.set_xlabel('time (s)')
                    # if fi == len(files) - 1:
                    #     a.set_ylim(bottom=0)
                    for side in ['top', 'right']:
                        a.spines[side].set_visible(False)
                else:
                    a.remove()
        except Exception as e:
            print(f'Warning: Plotting error for {f}; {e}')
    ax[1].legend()
    plt.savefig(Path(save_dir) / 'idetection_profile.png', dpi=200)


def save_one_box(xyxy, im, file='image.jpg', gain=1.02, pad=10, square=False, BGR=False, save=True):
    # Save image crop as {file} with crop size multiple {gain} and {pad} pixels. Save and/or return crop
    xyxy = torch.tensor(xyxy).view(-1, 4)
    b = xyxy2xywh(xyxy)  # boxes
    if square:
        b[:, 2:] = b[:, 2:].max(1)[0].unsqueeze(1)  # attempt rectangle to square
    b[:, 2:] = b[:, 2:] * gain + pad  # box wh * gain + pad
    xyxy = xywh2xyxy(b).long()
    clip_coords(xyxy, im.shape)
    crop = im[int(xyxy[0, 1]):int(xyxy[0, 3]), int(xyxy[0, 0]):int(xyxy[0, 2]), ::(1 if BGR else -1)]
    if save:
        file.parent.mkdir(parents=True, exist_ok=True)  # make directory
        cv2.imwrite(str(increment_path(file).with_suffix('.jpg')), crop)
    return crop
