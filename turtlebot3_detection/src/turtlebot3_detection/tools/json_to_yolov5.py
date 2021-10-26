import json
import os
import shutil
import argparse


# convert class name into index
CLASS = {
    'robot': 0
}


def parse_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--json_annot_path',
        type=str,
        default='dataset/robot_camera_json',
        help='Path to json annotation files.'
    )

    parser.add_argument(
        '--save_dir',
        type=str,
        default='dataset/yolov5_annotations',
        help='Save directory to store .txt yolov5 annotations'
    )

    return parser.parse_args()


def json_to_txt(args):
    load_dir = args.json_annot_path
    save_dir = args.save_dir

    if not os.path.isdir(save_dir):
        os.mkdir(save_dir)
    else:
        shutil.rmtree(save_dir)
        os.mkdir(save_dir)

    for json_path in os.listdir(load_dir):
        filename = json_path.split('.')[0]

        with open(os.path.join(load_dir, json_path), 'r') as json_file:
            load = json.load(json_file)

        # get the necessary information from annotation
        json_dict = load['annotation']

        # store bounding box as (x_min, y_min, x_max, y_max)
        img_object = json_dict['object']
        if type(img_object) is dict:
            bbox = img_object['bndbox']
            bbox_info = [(img_object['name'], int(bbox['xmin']), int(bbox['ymin']), int(bbox['xmax']), int(bbox['ymax']))]
        elif type(img_object) is list:
            bbox_info = [(o['name'], int(o['bndbox']['xmin']), int(o['bndbox']['ymin']),
                          int(o['bndbox']['xmax']), int(o['bndbox']['ymax'])) for o in img_object]

        new_bboxes = normalize_bbox(bbox_info,)
        for cls_idx, x_c, y_c, w, h in new_bboxes:
            one_annotation = f'{cls_idx} {x_c} {y_c} {w} {h}\n'
            with open(os.path.join(save_dir, filename+'.txt'), 'a') as txt_file:
                txt_file.write(one_annotation)


# convert ('label', x_min, y_min, x_max, y_max) into (idx, x, y, w, h) format
def normalize_bbox(bboxes, img_size=(512, 640)):
    global CLASS
    img_height, img_width = img_size

    new_bboxes = []
    for label, x_min, y_min, x_max, y_max in bboxes:
        label_idx = CLASS[label]

        x_cnorm = 1/img_width * (x_min + x_max) / 2
        y_cnorm = 1/img_height * (y_min + y_max) / 2
        w = 1/img_width * (x_max - x_min)
        h = 1/img_height * (y_max - y_min)

        new_bboxes.append((label_idx, x_cnorm, y_cnorm, w, h))

    return  new_bboxes


def main():
    args = parse_arguments()
    json_to_txt(args)


if __name__ == '__main__':
    main()