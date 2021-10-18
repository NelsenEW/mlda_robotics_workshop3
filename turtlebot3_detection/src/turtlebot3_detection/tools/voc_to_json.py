import xmltodict
from xml.etree import ElementTree as ET
import json

import argparse
import os
import shutil


def parse_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--annot_dir',
        type=str,
        default='dataset/robot_camera_annotation',
        help='Path to Pascal VOC annotations'
    )

    parser.add_argument(
        '--save_dir',
        type=str,
        default='dataset/robot_camera_json',
        help='Directory to store json annotations'
    )

    return parser.parse_args()


# Convert xml to json file
def xml_to_json(args):
    load_dir = args.annot_dir
    save_dir = args.save_dir

    # Create the folder if it is not exist
    if not os.path.isdir(save_dir):
        os.mkdir(save_dir)
    else:
        shutil.rmtree(save_dir)
        os.mkdir(save_dir)

    for xml_path in os.listdir(load_dir):
        filename = xml_path.split('.')[0]

        tree = ET.parse(os.path.join(load_dir, xml_path))
        root = tree.getroot()
        xml_str = ET.tostring(root, encoding='unicode', method='xml')
        xml_dict = xmltodict.parse(xml_str, process_namespaces=True)

        # save into json files
        with open(os.path.join(save_dir, filename+'.json'), 'w') as json_file:
            json.dump(xml_dict, json_file, indent=2)


def main():
    args = parse_arguments()
    xml_to_json(args)


if __name__ == '__main__':
    main()