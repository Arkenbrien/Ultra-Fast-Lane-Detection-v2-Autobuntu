from cmath import isnan
import torch, os, cv2
from utils.dist_utils import dist_print
import torch, os
from utils.common import merge_config, get_model
import tqdm
import torchvision.transforms as transforms
from data.dataset import LaneTestDatasetRos
import rospy
import numpy as np
import time

from sensor_msgs.msg import Image
from lidar2cam_projection.msg import pixel_ranges
from lidar2cam_projection.msg import pixel_range

class pixel_range_sub(object):
    def __init__(self):
        self.pixel_sub  = rospy.Subscriber("/pixel_range_cloud", pixel_ranges, self.get_pixel_range)

    def get_pixel_range(self, data):
        self.pixel_vector =  data

def distance(point1,point2):
    return ((point1[0] - point2[0])**2 + (point1[1]-point2[1])**2)**(1/2)


def extract_lane_ranges(line_list, pixel_points, pixel_threshold):
    lidar_point_master = []
    for point in pixel_points.points:
        pixel_u = point.u
        pixel_v = point.v

        pixel_r = point.range
        pixel_x = point.x
        pixel_y = point.y
        pixel_z = point.z
        color = (255,0,0)

        for line in range(len(line_list)):
            for point_px in range(len(line_list[line])):
                line_pix = line_list[line][point_px]
                distance_px = distance([pixel_u, pixel_v], line_pix)
                if distance_px < pixel_threshold:
                    lidar_point_master.append([pixel_x, pixel_y, pixel_z, pixel_u, pixel_v, line])
            
    return_list = []
    for line_number in range(len(line_list)):
        this_line_x  = []
        this_line_y  = []
        this_line_z  = []
        this_line_u  = []
        this_line_v  = []

        for matched_point in range(len(lidar_point_master)):
            extracted_point = lidar_point_master[matched_point]
            pixel_x =  extracted_point[0]
            pixel_y =  extracted_point[1]
            pixel_z =  extracted_point[2]
            pixel_u =  extracted_point[3]
            pixel_v =  extracted_point[4]
            line    =  extracted_point[5]

            if line == line_number:
                color = (255,0,255)
                this_line_x.append(pixel_x)
                this_line_y.append(pixel_y)
                this_line_z.append(pixel_z)
                this_line_u.append(pixel_u)
                this_line_v.append(pixel_v)

        line_x_avg = np.average(this_line_x)
        line_y_avg = np.average(this_line_y)
        line_z_avg = np.average(this_line_z)
        line_u_avg = np.average(this_line_u)
        line_v_avg = np.average(this_line_v)

        return_list.append([line_x_avg, line_y_avg, line_z_avg, line_u_avg, line_v_avg, line])

    return return_list

def pred2coords(pred, row_anchor, col_anchor, local_width = 1, original_image_width = 1640, original_image_height = 590):
    batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape
    batch_size, num_grid_col, num_cls_col, num_lane_col = pred['loc_col'].shape

    max_indices_row = pred['loc_row'].argmax(1).cpu()
    # n , num_cls, num_lanes
    valid_row = pred['exist_row'].argmax(1).cpu()
    # n, num_cls, num_lanes

    max_indices_col = pred['loc_col'].argmax(1).cpu()
    # n , num_cls, num_lanes
    valid_col = pred['exist_col'].argmax(1).cpu()
    # n, num_cls, num_lanes

    pred['loc_row'] = pred['loc_row'].cpu()
    pred['loc_col'] = pred['loc_col'].cpu()

    coords = []

    row_lane_idx = [1,2]
    col_lane_idx = [0,3]

    for i in row_lane_idx:
        tmp = []
        if valid_row[0,:,i].sum() > num_cls_row / 2:
            for k in range(valid_row.shape[1]):
                if valid_row[0,k,i]:
                    all_ind = torch.tensor(list(range(max(0,max_indices_row[0,k,i] - local_width), min(num_grid_row-1, max_indices_row[0,k,i] + local_width) + 1)))
                    out_tmp = (pred['loc_row'][0,all_ind,k,i].softmax(0) * all_ind.float()).sum() + 0.5
                    out_tmp = out_tmp / (num_grid_row-1) * original_image_width
                    tmp.append((int(out_tmp), int(row_anchor[k] * original_image_height)))
            coords.append(tmp)

    for i in col_lane_idx:
        tmp = []
        if valid_col[0,:,i].sum() > num_cls_col / 4:
            for k in range(valid_col.shape[1]):
                if valid_col[0,k,i]:
                    all_ind = torch.tensor(list(range(max(0,max_indices_col[0,k,i] - local_width), min(num_grid_col-1, max_indices_col[0,k,i] + local_width) + 1)))
                    out_tmp = (pred['loc_col'][0,all_ind,k,i].softmax(0) * all_ind.float()).sum() + 0.5
                    out_tmp = out_tmp / (num_grid_col-1) * original_image_height
                    tmp.append((int(col_anchor[k] * original_image_width), int(out_tmp)))
            coords.append(tmp)

    return coords
    
if __name__ == "__main__":
    torch.backends.cudnn.benchmark = True
    

    rospy.init_node('ultra_fast_v2', anonymous=True)

    args, cfg = merge_config()
    cfg.batch_size = 1
    print('setting batch_size to 1 for demo generation')

    range_lane = False

    rospy.wait_for_message(cfg.ros_topic, Image, timeout=60)
    pixel_range_dat = pixel_range_sub()
    pixel_threshold = 10
    time.sleep(1)

    dist_print('start testing...')
    assert cfg.backbone in ['18','34','50','101','152','50next','101next','50wide','101wide']

    net = get_model(cfg)

    state_dict = torch.load(cfg.test_model, map_location='cpu')['model']
    compatible_state_dict = {}
    for k, v in state_dict.items():
        if 'module.' in k:
            compatible_state_dict[k[7:]] = v
        else:
            compatible_state_dict[k] = v

    net.load_state_dict(compatible_state_dict, strict=False)
    net.eval()

    img_transforms = transforms.Compose([
        transforms.Resize((int(cfg.train_height / cfg.crop_ratio), cfg.train_width)),
        transforms.ToTensor(),
        transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
    ])

    splits = ['test.txt']
    datasets = [LaneTestDatasetRos(cfg.ros_topic,img_transform = img_transforms, crop_size = cfg.train_height) for split in splits]
    r = rospy.Rate(1)
    r.sleep()

    rate = 30
    while not rospy.is_shutdown():
        loaded_im = datasets.__getitem__(0)[0][0]
        img_w = datasets[0].cv_image.shape[1]
        img_h = datasets[0].cv_image.shape[0]
        extracted_image = datasets[0].cv_image

        pixel_info = pixel_range_dat.pixel_vector
        extracted_image = datasets[0].cv_image

        for split, dataset in zip(splits, datasets):
            loader = torch.utils.data.DataLoader(dataset, batch_size=1, shuffle = False, num_workers=1)
            for i, data in enumerate(loader):
                imgs, names = data
                imgs = imgs.cuda()
                with torch.no_grad():
                    pred = net(imgs)

                coords = pred2coords(pred, cfg.row_anchor, cfg.col_anchor, original_image_width = img_w, original_image_height = img_h)
                count = 0

                line_list = []
                for lane in coords:
                    line = []
                    if count < 2:
                        for coord in lane:
                            line.append(coord)
                            cv2.circle(extracted_image, coord, 5, (0,255,0), -1)
                    line_list.append(line)
                    count += 1
            
                lane_range_extract = extract_lane_ranges(line_list, pixel_info, pixel_threshold)
                
                dev_list = []
                for results in lane_range_extract:
                    line_x_avg = results[0]
                    line_y_avg = results[1]
                    line_z_avg = results[2]
                    line_u_avg = results[3]
                    line_v_avg = results[4]
                    count = results[5]

                    if not np.isnan(line_x_avg):
                        dev_list.append(line_x_avg)

                    try:
                        cv2.putText(extracted_image, str(count)+" DEV: "+str(round(line_x_avg,2)), (int(line_u_avg+30), int(line_v_avg)), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 3, cv2.LINE_AA)
                    except:
                        continue

                lane_min = np.min(dev_list)
                lane_max = np.max(dev_list)
                lane_width = np.abs(lane_min) + np.abs(lane_max)
                try:
                    cv2.putText(extracted_image, "Lane Width:  "+str(round(lane_width,2))+"m", (int(img_w/2), int(img_h-img_h*0.02)), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), 5, cv2.LINE_AA)
                except:
                    continue
                

            
        vis = cv2.resize(extracted_image, (640,480))
        cv2.imshow('vis',vis)
        cv2.waitKey(1)
        r = rospy.Rate(rate)
        r.sleep()

