import os
import shutil

if __name__ == '__main__':
    dataset_path = 'dataset/robot_camera'
    annot_path = 'dataset/labels'
    save_dir = 'dataset/robot_camera_clean'

    if not os.path.isdir(save_dir):
        os.mkdir(save_dir)
    else:
        shutil.rmtree(save_dir)
        os.mkdir(save_dir)

    avail_images = [filename.split('.')[0] for filename in os.listdir(annot_path)]
    for file in os.listdir(dataset_path):
        filename = file.split('.')[0]

        if filename in avail_images:
            shutil.copyfile(os.path.join(dataset_path, file), os.path.join(save_dir, file))