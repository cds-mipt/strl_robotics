"""
Extract image features and save them in faiss database format
"""

import argparse
import os

import yaml
import pandas as pd
import numpy as np
import torch
import torchvision.transforms as transforms
from PIL import Image
from tqdm import tqdm
import faiss

from src.models.model_factory import get_model
from src.utils import convert_dict_to_tuple


def input_transform(resize=(480, 640)):
    if resize[0] > 0 and resize[1] > 0:
        return transforms.Compose([
            transforms.Resize(resize),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])
    else:
        return transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, required=True,
                        help="Model yaml config file")
    args = parser.parse_args()

    with open(args.config) as f:
        data = yaml.safe_load(f)

    config = convert_dict_to_tuple(data)
    print(f"Model: {config.model.name}")

    encoder = get_model(config.model)
    print(f"Loading model weights: {config.model.weights}")
    weights = torch.load(config.model.weights, map_location=torch.device('cpu'))
    encoder.load_state_dict(weights['state_dict'])

    if config.enable_cuda and torch.cuda.is_available():
        print("Moving to cuda...")
        encoder = encoder.to('cuda')
    else:
        print("Running on cpu...")

    print("Loading database csv file...")
    db_csv_path = config.database.csv
    assert os.path.isfile(db_csv_path), f"No such file: {db_csv_path}"

    db_df = pd.read_csv(db_csv_path, index_col=0)
    images_list = db_df['image_ts'].tolist()
    images_dir = os.path.abspath(config.database.images.dir)
    assert os.path.isdir(images_dir), f"No such directory: {images_dir}"
    images_list = [os.path.join(images_dir, str(ts)+'.png') for ts in images_list]
    
    transform = input_transform(resize=config.database.images.resize)

    descriptors_np = np.zeros((len(images_list), config.model.feature_dim), dtype=np.float32)
    for i, image_path in tqdm(enumerate(images_list), desc="Extracting features", total=len(images_list)):
        img = transform(Image.open(image_path)).unsqueeze(0)
        if config.enable_cuda and torch.cuda.is_available():
            img = img.to('cuda')
        descriptor = encoder(img)
        descriptors_np[i, :] = descriptor.detach().cpu().numpy()

    db_index = faiss.IndexFlatL2(descriptors_np.shape[1])
    db_index.add(descriptors_np)
    print(f"Added {db_index.ntotal} descriptors to database")

    index_filename = config.database.faiss_index
    faiss.write_index(db_index, index_filename)
    print(f"Saved index: {index_filename}")
