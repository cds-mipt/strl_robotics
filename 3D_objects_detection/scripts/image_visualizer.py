import argparse
import os
import cv2
import torch
from tqdm import tqdm
import numpy as np

from mmdet.apis import init_detector, inference_detector, show_result_ins


def build_parser():
    parser = argparse.ArgumentParser(description='MMDet test detector')
    parser.add_argument('--input_folder', help='path to folder with raw images')
    parser.add_argument('--output_folder', help='path to folder with with visualization')
    parser.add_argument('--config', help='test config file path', default=
    r'/home/docker_solo/catkin_ws/src/solov2/lift_project_2/solov2_light2.py')
    parser.add_argument('--checkpoint', help='checkpoint file', default=
    r'/home/docker_solo/catkin_ws/src/solov2/lift_project_2/epoch_125.pth')
    parser.add_argument('--treshold', help='score treshold', type=float, default=0.5)
    return parser

def remove_noize(img_gray):
    kernel = np.ones((5, 5), np.uint8)
    eroded = cv2.erode(img_gray, kernel, iterations=2)
    dilated = cv2.dilate(eroded, kernel, iterations=2)
    return dilated

def set_value(mask_shape, bbox, coord, radius):
    mask = np.zeros(mask_shape, dtype=np.uint8)
    x, y = coord
    for i in range(bbox[0], bbox[1]):
        for j in range(bbox[2], bbox[3]):
            if (i - x)**2 + (j - y)**2 <= radius**2:
                mask[j][i] = 1
    return mask

def main(args):

    model = init_detector(args.config, args.checkpoint, device='cuda:0')

    for image_path in tqdm(os.listdir(args.input_folder)):
        img_path = os.path.join(args.input_folder, image_path)
        #print(img_path)
        img = cv2.imread(img_path)
        #print(type(img), img.shape)

        output = inference_detector(model, img)
        if output[0] is None:
            output = [(torch.Tensor(0, 1), torch.Tensor(0, 1), torch.Tensor(0, 1))]

        masks = output[0][0].cpu().numpy()
        labels = output[0][1].cpu().numpy()
        scores = output[0][2].cpu().numpy()

        m = scores > args.treshold
        masks = masks[m]
        labels = labels[m]
        scores = scores[m]

        try:
            semantic_mask = sum(masks)
            semantic_mask = np.clip(semantic_mask, 0, 1)
            semantic_mask = semantic_mask.astype(np.uint8)
            semantic_image = remove_noize(semantic_mask)
            #print(semantic_image.shape)
            cnts, _ = cv2.findContours(semantic_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_parameters = [cv2.minEnclosingCircle(c) for c in cnts]
            cnts_parameters = [param for param in cnts_parameters if param[1] > 10] #param == ((x, y), radius)

            if cnts_parameters:
                for cnt_parameter in cnts_parameters:
                    (x, y), radius = cnt_parameter
                    bbox = (max(0, int(x - radius)), 
                            min(int(x + radius), semantic_mask.shape[1]), 
                            max(0, int(y - radius)), 
                            min(int(y + radius), semantic_mask.shape[0]))
                    mask = set_value(semantic_mask.shape, bbox, (int(x), int(y)), int(radius)).astype(np.bool)
                    #print(mask.shape)
                    color = np.random.randint(0, 256, 3)
                    img[mask] = img[mask] * 0.5 + color * 0.5

            output_path = os.path.join(args.output_folder, image_path)
            cv2.imwrite(output_path, img)
        except Exception:
            print(f"Image skipped - {img_path}")

if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    main(args)
