import os
import cv2
import numpy as np
from tqdm import tqdm
import argparse
import math

parser = argparse.ArgumentParser(description='Convert Models')
parser.add_argument('input', metavar='I', help='input name')
args = parser.parse_args()

def addTogether(img,car,x,y):   # img為背景圖 ,car為車子圖 , x,y為圖像在背景的位置
  
    
    rows,cols,channels = car.shape #拆分圖片信息
    #轉換格式 
    img_hsv = cv2.cvtColor(car,cv2.COLOR_RGB2HSV) #把圖片轉換成HSV格式，用於摳圖 
    #摳圖 
    lower_blue=np.array([0,0,0]) #獲取最小閾值 
    upper_blue=np.array([0,255,255]) #獲取最大閾值 
    mask = cv2.inRange(img_hsv, lower_blue, upper_blue) #創建遮罩 

    erode=cv2.erode(mask,None,iterations=3) #圖像腐蝕 

    dilate=cv2.dilate(erode,None,iterations=1) #圖像膨脹 

    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8))) #開運算 

    #========圖像合併==========================================================================================
    center = [y,x] #設置car在背景圖的起始位置
    for i in range(rows): 
        for j in range(cols): 
            if opening[i,j]==0: #代表黑色
                img[center[0]+i,center[1]+j] =car[i,j] #賦值顏色

#tmp = 0 # Should be accurate signal later on
def sound_signal(draw_img):
    #導入車子圖
    #global tmp # Take the global var tmp
    car = cv2.imread('./car.png') #car圖片導入
    car = cv2.resize(car,(0,0),fx=0.15,fy=0.15)

    #導入ambulance
    amb = cv2.imread('./amb_copy.png') #ambulance圖片導入
    amb = cv2.resize(amb,(0,0),fx=0.5,fy=0.5)

    #================初始化座標值=====================
    x = int(0.1*draw_img.shape[1]) #圓的x座標 圖片的寬的倍數 shape=(length,width,channel)
    y = int(0.8*draw_img.shape[0]) #圓的y座標
    r =int(0.15*draw_img.shape[0]) #圓的半徑

    #================畫出圓形=====================
    cv2.circle(draw_img,(x,y),r,(84,46,8),-1)
    m = 3 #畫m個同心圓
    r_circle = r/m
    for i in range(1,m+1):
        cv2.circle(draw_img,(x,y),int(r_circle*i),(255,255,255),1)

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
        cv2.line(draw_img,(x,y),(x_line,y_line),(250,255,255),1)

    #===========================================
    #方位角 角度為n等份的範圍值 (畫扇形:橢圓的長軸與短軸=圓形半徑)
    ### Select a specific signal
    p = 7 # 7*30
    #p = np.random.randint(0,n)  #隨機挑選第n等份的某值 -> Also, the pic being select is not in the correct order (Double random)


    #print("p:"+str(p))
    degree_elli_start = p * degree_line # degree_line=360/n #起始角度
    degree_elli_end = (p+1) * degree_line #終止角度

    # #橢圓(圖片名稱,中心點,(長軸,短軸),旋轉角度（順時針方向）,繪製的起始角度（順時針方向）,繪製的終止角度(順時針方向),線條顏色,線條粗細)
    cv2.ellipse(draw_img,(x,y),(r,r),0,-degree_elli_start,-degree_elli_end,(0,215,255),-1)

    addTogether(draw_img,car,70,410)
    addTogether(draw_img,amb,200,400)

    return draw_img

def draw_grtr(img, positions, width):
    for t in range(len(positions) - 1):
        cv2.line(img, (round(positions[t][0]), round(positions[t][1])), \
                 (round(positions[t + 1][0]), round(positions[t + 1][1])), \
                 color=(255,255,255), thickness=width, lineType=cv2.LINE_AA)

def create_traj_data(tracking_txt_path, predict_len, block_id_list, forget_frames, extra_length=2.0): # 1.0 # 1.5, too long?
    tracking_data_full = np.loadtxt(tracking_txt_path)
    #print(tracking_data_full[0])
    # tracking_data = tracking_data_full[:, :4].copy() # frame, ID, x1, y1 (Only pedestrians)

    # Keep pedestrian only
    tracking_data_ped = [e for e in tracking_data_full if e[1] == 0] # 0: ped, 2: vehicle
    # For new class types
    ### Don't need frame in real-time
    tracking_data_ped = np.delete(tracking_data_ped, 1, axis=1) # frame(timestamp), class, ID, x1, y1, x2, y2
    #print(tracking_data_full[0])
    tracking_data = tracking_data_ped[:, :4].copy() 
    frame_ids, counts = np.unique(tracking_data[:, 0], return_counts=True)

    tracking_data[:, 2] = tracking_data[:, 2] + (tracking_data_ped[:, 4] / 2) 
    tracking_data[:, 3] = tracking_data[:, 3] + tracking_data_ped[:, 5] # frame, ID, xc, y2

    tracking_agent = {}
    traj_data = {}

    index = 0
    disappear_id = {}
    for i, t in enumerate(frame_ids):
        agent_in_the_frame = tracking_data[index:index+counts[i]]
        # Stop tracking an agent when it does not appear in near frames (forget_frames)
        pop_list = []
        for key, value in tracking_agent.items():
            if key not in agent_in_the_frame[:, 1]: # ID_list
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
            '''
            # if id in block_id_list:
            #     continue
            # if id == 16 and agent[2]>=500:
            #     continue
            # if id == 16 and agent[0]>390:
            #     continue
            '''
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

        index += counts[i]
    return traj_data # {start_frame(679): array([[[679(frame), 7(id), x, y], [680, 7, x, y],,, [703, 7, x, y]], [[679(frame), 55(id), x, y], [680, 55, x, y],,, [703, 55, x, y]]])}

thetas = np.linspace(-0.2, 0.2, num=20)

def traj_diversify(traj_data, n_samples): # Expand the single path into a sector
    diverse_traj_data = {}
    for key, value in tqdm(traj_data.items()):
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

def draw_traj_heatmap(image_dir, diverse_traj_data, output_dir, blur_size, bbox_fromtxt):
    for file_name in tqdm(os.listdir(image_dir)):
        #print(os.path.join(image_dir, file_name))
        image = cv2.imread(os.path.join(image_dir, file_name))
        frame_id = int(file_name[:-4])
        # npy_line = np.load('npy/'+file_name[:-4].zfill(4)+'.npy') # load binary mask of drivable area from lane detection
        dets = bbox_fromtxt[bbox_fromtxt[:, 0] == frame_id][:, 2:6].astype(int) # x1, y1, w, h
        dets[:, 0] = dets[:, 0] + dets[:, 2]//2
        dets[:, 1] = dets[:, 1] + dets[:, 3]
        
        # traj_mask = np.zeros((npy_line.shape[0], npy_line.shape[1], 3))
        traj_mask = np.zeros((image.shape[0], image.shape[1], 3))

        # line_mask = np.zeros((npy_line.shape[0], npy_line.shape[1], 3))
        line_mask = np.zeros((image.shape[0], image.shape[1], 3))
        # line_mask[npy_line==1] = (0,255,0)

        final = None

        bot_max = -1
        for det in dets:
            cx = det[0]
            y2 = det[1]

            # if cx >= npy_line.shape[1]:
            #     cx = npy_line.shape[1] - 1
            # if y2 > npy_line.shape[0]:
            #     y2 = npy_line.shape[0] - 1

            if cx >= image.shape[1]:
                cx = image.shape[1] - 1
            if y2 > image.shape[0]:
                y2 = image.shape[0] - 1

            # if line_mask[y2, cx, 1] != 0 and y2 > bot_max:
            #     bot_max = y2
        
        # if bot_max >= 0:
        #     line_mask[:bot_max, :, [1, 2]] = line_mask[:bot_max, :, [2, 1]]

        if frame_id in diverse_traj_data.keys():
            frame_data = diverse_traj_data[frame_id] # n_samples, num_id, timestep, coordinate (20, 2, 25, 4)
            traj_layers = np.zeros((20, image.shape[0], image.shape[1]), np.uint8)
            for i, sample in enumerate(frame_data): # i: sample_id, sample: different_traj in a frame
                routes = None
                for agent_data in sample:
                    route = agent_data[:, 2:4]
                    draw_grtr(traj_layers[i], route, 15) 
                    
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
            final_1 = cv2.addWeighted(line_mask.astype(np.uint8), 1, image, 1, 0)
            final_2 = cv2.addWeighted(traj_mask.astype(np.uint8), 1, image, 1, 0)
            final = cv2.addWeighted(final_1, 0.5, final_2, 0.5, 0)
            cv2.drawContours(final, cnts, -1, (0, 0, 0), 1, lineType=cv2.LINE_AA)

        else:
            final = cv2.addWeighted(line_mask.astype(np.uint8), 1, image, 1, 0)
            #final_2 = image
            heat = np.zeros((image.shape[0], image.shape[1]))
            
        out_path = os.path.join(output_dir, file_name)
        final = cv2.resize(final, (0,0), fx = 0.5, fy = 0.5) 

        ### ADD additional function here
        final_img = sound_signal(final)

        cv2.imwrite(out_path, final_img)
        # cv2.imwrite(out_path[:-4]+'_line.png', line_mask)
        # cv2.imwrite(out_path[:-4]+'_gray.png', traj_mask)

        # Delete this
        # npy_path = os.path.join(npy_dir, file_name[:-4])+'.npy'
        # np.save(npy_path, heat)
    
#args.input = 'test1'
# tracking_txt_path = 'result/'+args.input+'.txt' # object tracking result txt [frame id, ID, x1, y1, w, h]
tracking_txt_path = args.input+'.txt' #'video_0106/'+args.input+'.txt' # object tracking result txt [frame id, ID, x1, y1, w, h]
bboxes = np.loadtxt(tracking_txt_path)
image_dir = 'demo_in/'+args.input
output_dir = 'demo_out/'+args.input
# npy_dir = 'demo_out/'+args.input+'_npy'

os.makedirs(output_dir, exist_ok=True)
#os.makedirs(npy_dir, exist_ok=True)

predict_len = 6 #10 #6 #20 # tracking consistency
n_samples = 20 # multi-modality num
# blur_size = 10
blur_size = 3

# # ========== round2 all stop ==========
# block_id_list = \
# [1,4,5,8,13,14,15,20,21,37,41,56,69,67,84,85,102,110,111,121,129,130,137,148,152,153,154,155,157,164,179,185,186,194,196,200,201,208,214,224,225,230,246,251,256,259,269,273,275,277,278,
#  281,282,287,289,290,300,306,312,315,317,318,324,330,337,338,342,343,344,348,353,355,356,358,364,366,373,383,392,401,402,405,406,409,415,420,424,427,430,431,433,434,439,443,453,459,464,478,491,
#  482,484,486,494,499,505,506,508,513,518,521,524,532,539,543,544,549,553,565,575,586,589,590,594,596,   # No. 1500
#  639,640,641,642,643,644,645,646,647,648,649,650,651,652,653,654,655,656,657,658,659,660,661,662,663,664,665,666,667,668,669,670,671,672,673,674,675,676,677,678,679,680,
#  681,682,683,684,685,686,687,688,689,690,691,692,693,694,695,696,697,698,699,700,708,715,716,
#  736,737,743,745,746,748,752,773,785,791,792,793,810,811,821,823,828,830,832,833,840,845,853,858,864,869,884,850,893,895,896,911,921,929,935,936,938,945,960,970,974,976,983,1004,1015,1019,
#  1041,1050,1068,1079,1081,1082,1083,1094,1098,1104,1120,1178,1183,1188,1192,1195,1197,1199,1201,1209,1211,1212,1213,1214,1217,1219,1220,1121,1233,1246,1245,1251,1252,1254,1260,1261,1264,1266,
#  1270,1277,1284,1286,1295,1297,1343,1345,   # No. 3100
#  1379,1383,1385,1391,1403,1404,1407,1412,1413,1429,1435,1449,1458,1480,1483,1499,1500,1504,1505,1511,1516,1522,1535,1539,1551,1554,1558,1561,1565,1570,1572,1594,1600,1605,
#  1609,1616,1618,1620,1625,1626,1634,1638,1642,1646,1652,1656,1676,1680,1685,   # No. 3830
#  1702,1704,1720,1731,1737,1741,1749,1760,1762,1770,1772,1783,1798,1808,1810,1814,1819,1827,1838,1847,1865,1866,1871,1872,1877,1882,1888,1902,1906,1908,1909,1911,1912,1918,1923,1924,
#  1926,1928,1953,1960,1965,1966,1984,1990,1993,1998,2020,2025,2030,2032,2036,2048,2053,2056,2058,2079,2085,2090,2092,2098,2100,2104,2105,2111,2112,2146,   # No. 4850
#  2154,2161,2162,2163,2164,2165,2166,2167,2168,2169,2170,2171,2172,2173,2201,2202,2203,2204,2205,2206,2207,2208,2209,2210,2211,2212,2213,2214,2215,2216,2217,2218,2219,
#  2220,2221,2222,2223,2224,2225,2226,2227,2228,2230,2231,2237,2238,2243,2246,2249,2251,2263,2266,2279,2280,2283,2284,2289,2323,2326,2338,2348,2373,2398,2400,2405,2415,2416,2419,
#  2422,2426,2429,2432,2435,2436,2445,2448,2461,2464,2466,2485,2499,2505,2517,2524,2528,2531,2550,2555,2564,2569,2572,2575,2596,2604,2633,2646,2662,2664,2673,2674,2585,2630,2665,2668,2673,2681,
#  2684,2691,2694,2701,2702,2705,2713,2722,2725,2727,2731,2735,2738,2739,2748,2765,2766,2769,2771,2780,2791,2792,2794,2816,2823,2824,2826,2843,2845,2849,2850,2852,2853,2856,2859,2861,2864,
#  2867,2869,2871,2876,2884,2887,2889,2894,2896,2897,2900,2928,2934,2941,2942,2980,
#  # not in 3d detect
#  2,89,153,190,191,206,625,630,633,800,816,1280,1286,1287,1441,1491,1530,1585,
# ] 
# # =====================================


# =========== round2 normal ===========
# block_id_list = \
# [
#  # NG case
#  41,139,234,251,255,260,313,319,448,560,564,619,738,793,808,873,894,922,925,963,1042,1097,1119,1129,1150,1164,1175,1246,1247,1298,1457,1466,1469,1495,1565,
#  1576,1681,1905,1908,1932,2037,2053,2057,2078,2087,2132,2167,2299,2326,2403,2424,2426,2427,2434,2511,2518,2563,2678,2653,2687,2691,2701,2713,2753,2775,2928,2932,2942,
#  2975,2988,2990,2991,3036,3037,
#  # not in 3d detect
#  2,3,4,5,8,9,12,13,15,16,19,22,75,87,89,91,153,187,190,191,200,201,206,209,211,213,214,470,555,557,562,564,565,567,585,586,587,588,589,590,591,592,593,594,595,596,
#  597,598,599,600,601,602,603,604,605,606,607,608,609,610,611,612,613,614,615,616,617,618,625,630,633,646,775,800,816,922,925,935,941,944,945,950,951,952,953,954,955,956,
#  957,958,970,972,980,985,997,1079,1084,1103,1118,1264,1272,1280,1286,1287,1321,1441,1490,1491,1503,1526,1530,1585,1594,1601,1607,1608,1614,1616,1619,1627,1640,
#  1641,1642,1643,1645,1646,1648,1649,1653,1661,1663,1669,1880,1886,1888,1891,1946,2015,2028,2147,2234,2263,2266,2272,2273,2274,2275,2276,2277,2278,2279,2280,
#  2377,2853,2888,2926,2970,2982,3003,3008
# ]
# =====================================
block_id_list = []

forget = 3 #5 #10

traj_data = create_traj_data(tracking_txt_path, predict_len, block_id_list, forget)
diverse_traj_data = traj_diversify(traj_data, n_samples=n_samples)
draw_traj_heatmap(image_dir, diverse_traj_data, output_dir, blur_size, bboxes)