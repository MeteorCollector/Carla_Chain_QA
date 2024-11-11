import os
import shutil

def remove_empty_directories(root_path):
    ret = 0
    for root, dirs, files in os.walk(root_path, topdown=False):
        for dir_name in dirs:
            dir_path = os.path.join(root, dir_name)
            if dir_name in filter_words:
                # 检查同级文件夹是否非空
                siblings_non_empty = any([os.listdir(os.path.join(root, d)) for d in dirs if d != dir_name])
                if not siblings_non_empty:
                    print("Removing directory named '{}' because it's empty and no non-empty siblings found: {}".format(dir_name, dir_path))
                    os.rmdir(dir_path)
                    ret += 1
            elif not os.listdir(dir_path):
                print("Removing empty directory:", dir_path)
                os.rmdir(dir_path)
                ret += 1
    return ret

def delete_folders_with_all_json_files(path):
    ret = 0
    for root, dirs, files in os.walk(path, topdown=False):
        for name in dirs:
            try:
                folder_path = os.path.join(root, name)
                only_json_files = all(file.endswith('.json') for file in os.listdir(folder_path))
                no_subfolders = not any(os.path.isdir(os.path.join(folder_path, f)) for f in os.listdir(folder_path))
                same_level_empty_folders = all(not os.listdir(os.path.join(root, d)) for d in dirs if d != name)
                if only_json_files and no_subfolders and same_level_empty_folders:
                        ret += 1
                        shutil.rmtree(folder_path)
                        print(f"Deleted folder: {folder_path}")
            except:
                print("Error when removing")
    return ret


filter_words = ["map", "maps"]
clear_path = os.environ.get("CLEAR_PATH")


if clear_path:
    remove_empty_directories(clear_path)
    delete_folders_with_all_json_files(clear_path)
    number = 1
    while (number > 0):
        number = remove_empty_directories(clear_path)
        number += delete_folders_with_all_json_files(clear_path)
else:
    print("CLEAR_PATH environment variable not set.")
