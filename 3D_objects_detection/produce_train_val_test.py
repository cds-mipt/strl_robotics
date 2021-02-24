import os
import argparse
import json
import random
from tqdm import tqdm
from copy import deepcopy
import shutil

class Dataset:
    def __init__(self, data):
        self.data = data
        self.json = self.make_json(result)
        self.path = None
    
    @staticmethod
    def make_json(base):
        return deepcopy(base)

def task_from_image(image_name):
    return image_name.split('_')[-1][:-4]


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Produce dataset with train val test in COCO format from different pieces(cvat tasks)")
    parser.add_argument('path_to_data', type=str, help='path to the folder with several COCO datasets')
    parser.add_argument('-categories', nargs='+', help='use only these categories')
    args = parser.parse_args()
    
    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'single_dataset')
    if os.path.exists(output_path):
        shutil.rmtree(output_path)
    os.mkdir('single_dataset')
    

    result = {}
    result['licenses'] = [{'name': '', 'id': 0, 'url': ''}]
    result['info'] = {'contributor': '', 'date_created': '', 'description': '', 'url': '', 'version': '', 'year': ''}
    result['categories'] = []
    result['annotations'] = []
    result['images'] = []
    

    image_id = 0
    ann_id = 0
    res_ann = []
    res_images = []
    set_cat = set()

    folders = tqdm(os.listdir(args.path_to_data))
    for folder in folders:
        folders.set_description("Adding %s" % folder)

    for task in folders:
        task_path = os.path.join(args.path_to_data, task)
        ann_path = os.path.join(task_path, 'annotations')

        with open(os.path.join(ann_path, 'instances_default.json'), 'rt') as file:
            annotations = json.load(file)    
            for category in annotations['categories']:
                if category['name'] in args.categories and category['name'] not in set_cat:
                    result['categories'].append(category)
                    set_cat.add(category['name'])

        id = 1
        used_cat =[i['id'] for i in result['categories']]
        for ann in annotations['annotations']:
            if ann['category_id'] in used_cat:
                ann['id'] = id + ann_id
                ann['image_id'] += image_id
                res_ann.append(ann)
                id += 1

        image_path = os.path.join(task_path, 'images')
        used_images = [i['image_id'] for i in annotations['annotations']]
        for _id, image in enumerate(annotations['images'], 1):
            image['id'] = _id + image_id
            if image['id'] in used_images:
                file_name = image['file_name']
                image['file_name'] = image['file_name'][:-4] + f'_{task}.PNG'
                shutil.copy(os.path.join(image_path, file_name), os.path.join(output_path, image['file_name']))
                res_images.append(image)

        ann_id += id - 1
        image_id = _id + len(annotations['images'])


    assert len(res_ann) > 4, 'small dataset'
    
    range_ = tqdm(range(1, 6))
    for r in range_:
        range_.set_description("Shuffling %s" % r)

    for i in range_:
        random.shuffle(res_images)


    train_size = 0.6
    val_size = 0.2
    test_size = 0.2
    
    train = res_images[:round(len(res_images)*train_size)]
    val = res_images[round(len(res_images)*train_size):round(len(res_images)*(train_size+val_size))]
    test = res_images[round(len(res_images)*(train_size+val_size)):]

 
    print("Packing")
    for data, name in zip([train, val, test], ['train', 'val', 'test']):
        d = Dataset(data)
        d.path = os.path.join(output_path, name)
        os.mkdir(d.path)
        os.mkdir(os.path.join(d.path, 'images'))
        os.mkdir(os.path.join(d.path, 'annotations'))

        for image in data:
            shutil.copy(os.path.join(output_path, image['file_name']), os.path.join(d.path, 'images'))
            os.remove(os.path.join(output_path, image['file_name']))
        
        images_id = [i["id"] for i in d.data]
        d_ann = [i for i in res_ann if i["image_id"] in images_id]
        d.json['annotations'] = d_ann
        d.json['images'] = d.data

        with open(os.path.join(d.path, 'annotations', f'{name}.json'), 'wt') as file:
            json.dump(d.json, file)