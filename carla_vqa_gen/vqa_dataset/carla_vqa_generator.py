import os
import glob
import gzip
import json
import tqdm
import random
import numpy as np
from pathlib import Path
from collections import Counter
import cv2
from PIL import Image, ImageDraw
from collections import Counter
import carla
from offline_map_calculations import *

from graph_utils import *

"""
Main class for processing and converting the pdm_lite dataset to the Graph-QAs for DriveLM-Carla.
"""

class QAsGenerator():
    all_qa_pairs = []

    def __init__(self, args):
        # Image and camera parameters
        self.TARGET_IMAGE_SIZE = args.target_image_size
        self.ORIGINAL_IMAGE_SIZE = args.original_image_size
        self.ORIGINAL_FOV = args.original_fov

        # Region of interest (ROI) for image projection
        self.MIN_X = args.min_x
        self.MAX_X = args.max_x
        self.MIN_Y = args.min_y
        self.MAX_Y = args.max_y

        # Sampling parameters
        self.random_subset_count = args.random_subset_count
        self.sample_frame_mode = args.sample_frame_mode
        self.sample_uniform_interval = args.sample_uniform_interval

        # Visualization and saving options
        self.save_examples = args.save_examples         
        self.visualize_projection = args.visualize_projection 
        self.filter_routes_by_result = args.filter_routes_by_result
        self.remove_pedestrian_scenarios = args.remove_pedestrian_scenarios

        self.data_directory = args.data_directory 
        self.path_keyframes = args.path_keyframes 

        self.output_graph_directory = args.output_graph_directory 
        self.output_graph_examples_directory = args.output_graph_examples_directory

        # Build camera projection matrix
        self.CAMERA_MATRIX = None
        self.CAM_INTRINSIC_FRONT = None
        self.CAM_INTRINSIC_FRONT_LEFT = None
        self.CAM_INTRINSIC_FRONT_RIGHT = None
        self.CAM_INTRINSIC_BACK = None
        self.CAM_INTRINSIC_BACK_LEFT = None
        self.CAM_INTRINSIC_BACK_RIGHT = None
        self.WORLD2CAM_FRONT = None
        self.WORLD2CAM_FRONT_LEFT = None
        self.WORLD2CAM_FRONT_RIGHT = None
        self.WORLD2CAM_BACK = None
        self.WORLD2CAM_BACK_LEFT = None
        self.WORLD2CAM_BACK_RIGHT = None
        self.CAMERA_MATRIX = build_projection_matrix(self.ORIGINAL_IMAGE_SIZE[0],
                                                     self.ORIGINAL_IMAGE_SIZE[1],
                                                     self.ORIGINAL_FOV)

        # creaete the directories where we save the graph and some graph examples
        Path(self.output_graph_directory).mkdir(parents=True, exist_ok=True)
        if self.save_examples:
            Path(self.output_graph_examples_directory).mkdir(parents=True, exist_ok=True)

        # all the paths to the boxes in the data
        self.data_boxes_paths = glob.glob(os.path.join(self.data_directory, '**/anno/*.json.gz'), recursive=True)

        # Randomly sample a subset of data (if random_subset_count > 0)
        if self.random_subset_count > 0:
            random.shuffle(self.data_boxes_paths)
            self.data_boxes_paths = self.data_boxes_paths[:self.random_subset_count]

        self.data_boxes_paths = list(sorted(self.data_boxes_paths))

        self.list_next_junction_id_minus_one = []

        # added for b2d
        self.map_file_dir = args.path_maps
        self.town_name = None
        self.map = None

        # camera infos
        self.CAMERA_FRONT = {}
        self.CAMERA_FRONT_LEFT = {}
        self.CAMERA_FRONT_RIGHT = {}
        self.CAMERA_BACK = {}
        self.CAMERA_BACK_LEFT = {}
        self.CAMERA_BACK_RIGHT = {}

        # to look into future
        self.current_measurement_path = None
        self.current_measurement_index = 0

        # to memorize progress
        self.processed_paths_file = "processed_paths.txt"
        self.processed_paths = self.load_processed_paths()

        # for [debug]
        self.appended_measurements = {}

        # config
        self.frame_rate = 10 # collect 10 data every 1s

        # global variables for calculation
        self.inf_num = 999
        self.leftmost_pos_of_left_hazard = {}
        self.last_full_scenario_name = 'last_name'
    
    def load_processed_paths(self):
        """从文件加载已处理的路径"""
        return self.load_marked_paths(self.processed_paths_file)

    def load_marked_paths(self, file_path):
        """从文件加载已处理的路径"""
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                return set(file.read().splitlines())
        return set()

    def save_processed_path(self, path):
        """将处理完成的路径追加到文件"""
        with open(self.processed_paths_file, 'a') as file:
            file.write(path + '\n')

    def reset_qa_stats(self):
        # Initialize data structures
        self.vqa_llava_format = {'image_paths': [], 'llava_format': [], 'image_subset': []}
        self.min_num_questions = 100000
        self.total_num_questions = 0
        self.total_num_objects = 0
        self.num_questions_per_category = {
            'parked_vehicles': 0,
            'dynamic_vehicles': 0,
            'roadlayout': 0,
            'stopsign': 0,
            'trafficlight': 0,
            'pedestrian': 0,
            'ego': 0,
        }
        self.stats_p3 = {'perception': 0, 'planning': 0, 'prediction': 0}

        self.frame_num = 0
        self.skipped_frames = 0

    def create_qa_pairs(self):
        """
        Create all question answer pairs in llava format, convert them to NuScenes afterwards and finally save them
        """

        self.reset_qa_stats()

        # Load keyframes list if sampling keyframes
        if self.sample_frame_mode == 'keyframes':
            # print("mode = keyframes, parsing keyframes.") # debug
            keyframes_list_path = self.path_keyframes
            with open(keyframes_list_path, 'r', encoding="utf-8") as f:
                keyframes_list = f.readlines()
            keyframes_list = [x.strip() for x in keyframes_list]
            keyframes_list = [x.replace('rgb', 'boxes').replace('.jpg', '.json.gz') for x in keyframes_list]

        # For debug, used when wanting a subset
        do_subset = int(os.environ.get('SUBSET', 0))
        if do_subset:
            subset_keys = self.load_marked_paths('./subset.txt')
        
        # Process each frame
        for path in tqdm.tqdm(self.data_boxes_paths):

            # skip if path or path's root path is in processed_paths.txt
            if any(processed_path in path for processed_path in self.processed_paths):
                # print(f"[info] skipping {path}, which has already marked processed.")
                continue

            if do_subset:
                if not (any(marked_path in path for marked_path in subset_keys)):
                    # print(f"[info] skipping {path}, which has already marked processed.")
                    continue

             # Skip frames based on keyframes list
            if self.sample_frame_mode == 'keyframes':
                if path not in keyframes_list:
                    self.skipped_frames += 1
                    continue

            vqa_llava_entry = {}
            frame_number = int(path.split('/')[-1].split('.')[0])

            # Skip frames if sampling uniformly and frame number does not match
            if self.sample_frame_mode == 'uniform' and frame_number % self.sample_uniform_interval != 0:
                continue

            # print("analysing path") # debug
            route_dir = '/'.join(path.split('/')[:-2])
            scenario_name = route_dir.split('/')[-1]
            town_name = route_dir.split('/')[-1].split('_')[1]
            route_number = route_dir.split('/')[-1].split('_')[0] + '_' + route_dir.split('/')[-1].split('_')[1] + '_' + route_dir.split('/')[-1].split('_')[2]

            self.current_measurement_path = path
            dir_name, file_name = os.path.split(path)
            base_name, _ = os.path.splitext(file_name)
            base_name, _ = os.path.splitext(base_name)
            self.current_measurement_index = int(base_name)
            
            # print(f"path: {path}, route_dir: {route_dir}, scenario_name: {scenario_name}, town_name: {town_name}, route_number: {route_number}")
            if self.map is None or (self.town_name is None or self.town_name is not town_name):
                self.town_name = town_name
                if os.path.exists(os.path.join(self.map_file_dir, f'OpenDrive/{self.town_name}.xodr')):
                    with open(os.path.join(self.map_file_dir, f'OpenDrive/{self.town_name}.xodr'), 'r') as fp:
                        self.map = carla.Map('{self.town_name}', fp.read())
                else:
                    with open(os.path.join(self.map_file_dir, f'{self.town_name}/OpenDrive/', f'{self.town_name}.xodr'), 'r') as fp:
                        self.map = carla.Map('{self.town_name}', fp.read())

            # # Skip this scenario because it is not annotated correctly
            # if 'InterurbanAdvancedActorFlow' in route_dir:
            #     continue

            # # Skip this scenario because it is not annotated correctly
            # # and we cannot differentiate between entry and exit properly
            # if 'MergerIntoSlowTraffic' in route_dir:
            #     continue

            # # Skip this scenario because language labels are not adjusted
            # if 'VehicleTurningRoute' == route_dir:
            #     continue

            # # Skip scenarios with pedestrians if specified
            # if self.remove_pedestrian_scenarios:
            #     if 'DynamicObjectCrossing' in route_dir:
            #         continue
            #     if 'ParkingCrossingPedestrian' in route_dir:
            #         continue
            #     if 'PedestrianCrossing' in route_dir:
            #         continue
            #     if 'VehicleTurningRoutePedestrian' in route_dir:
            #         continue

            # Skip frames if RGB image does not exist
            if not os.path.isfile(path.replace('anno', 'camera/rgb_front').replace('.json.gz', '.jpg')):
                self.skipped_frames += 1
                continue

            # Check if files exist
            if not os.path.exists(path):
                continue
            # there's only 'anno' folder in B2D dataset.
            # if not os.path.exists(path_measurements):
            #     continue

            # Read data and measurements files
            with gzip.open(path, 'rb') as f:
                file_content = f.read()
                data = json.loads(file_content.decode('utf-8'))

            # with gzip.open(path_measurements, 'rb') as f:
            #     file_content = f.read()
            #     measurements = json.loads(file_content.decode('utf-8'))

            # Get perception questions
            # print("generation reached here") # debug
                
            sensor_data = data['sensors']

            # Read all camera datas
            if 'CAM_FRONT' in sensor_data:
                self.CAMERA_FRONT = sensor_data['CAM_FRONT']
                self.CAM_INTRINSIC_FRONT = sensor_data['CAM_FRONT']['intrinsic']
                self.WORLD2CAM_FRONT = sensor_data['CAM_FRONT']['world2cam']
                self.CAMERA_MATRIX = self.CAM_INTRINSIC_FRONT
            if 'CAM_FRONT_LEFT' in sensor_data:
                self.CAMERA_FRONT_LEFT = sensor_data['CAM_FRONT_LEFT']
                self.CAM_INTRINSIC_FRONT = sensor_data['CAM_FRONT_LEFT']['intrinsic']
                self.WORLD2CAM_FRONT_LEFT = sensor_data['CAM_FRONT_LEFT']['world2cam']
            if 'CAM_FRONT_RIGHT' in sensor_data:
                self.CAMERA_FRONT_RIGHT = sensor_data['CAM_FRONT_RIGHT']
                self.CAM_INTRINSIC_FRONT = sensor_data['CAM_FRONT_RIGHT']['intrinsic']
                self.WORLD2CAM_FRONT_RIGHT = sensor_data['CAM_FRONT_RIGHT']['world2cam']
            if 'CAM_BACK' in sensor_data:
                self.CAMERA_BACK = sensor_data['CAM_BACK']
                self.CAM_INTRINSIC_FRONT = sensor_data['CAM_BACK']['intrinsic']
                self.WORLD2CAM_BACK = sensor_data['CAM_BACK']['world2cam']
            if 'CAM_BACK_LEFT' in sensor_data:
                self.CAMERA_BACK_LEFT = sensor_data['CAM_BACK_LEFT']
                self.CAM_INTRINSIC_FRONT = sensor_data['CAM_BACK_LEFT']['intrinsic']
                self.WORLD2CAM_BACK_LEFT = sensor_data['CAM_BACK_LEFT']['world2cam']
            if 'CAM_BACK_RIGHT' in sensor_data:
                self.CAMERA_BACK_RIGHT = sensor_data['CAM_BACK_RIGHT']
                self.CAM_INTRINSIC_FRONT = sensor_data['CAM_BACK_RIGHT']['intrinsic']
                self.WORLD2CAM_BACK_RIGHT = sensor_data['CAM_BACK_RIGHT']['world2cam']

            image_path = path.replace('anno', 'camera/rgb_front').replace('.json.gz', '.jpg')
            relative_image_path = image_path

            res = self.generate_perception_questions(data, scenario_name, scenario_name, route_number, frame_number)
            qas, num_questions, num_objects, questions_per_category, key_object_infos = res
            for key, values in qas.items():
                for value in values:
                    self.stats_p3[value['type']] += 0.5  # We have questions and answers

            # Save examples if specified
            if self.save_examples:
                # Load and draw on the image
                image = Image.open(image_path)
                draw = ImageDraw.Draw(image)

                original_img_path = f'{self.output_graph_examples_directory}/original_images/{scenario_name}'
                Path(original_img_path).mkdir(parents=True, exist_ok=True)
                resized_img_path = f'{self.output_graph_examples_directory}/resized_images/{scenario_name}'
                Path(resized_img_path).mkdir(parents=True, exist_ok=True)
                anno_img_path = f'{self.output_graph_examples_directory}/anno_images/{scenario_name}'
                Path(anno_img_path).mkdir(parents=True, exist_ok=True)
                graph_path = f'{self.output_graph_examples_directory}/graphs/{scenario_name}'
                Path(graph_path).mkdir(parents=True, exist_ok=True)

                assert image.width == self.ORIGINAL_IMAGE_SIZE[0], f'{image.width} != {self.ORIGINAL_IMAGE_SIZE[0]}'
                assert image.height == self.ORIGINAL_IMAGE_SIZE[1], f'{image.height} != {self.ORIGINAL_IMAGE_SIZE[1]}'
                
                # Draw a point for each object (e.g, car, traffic light, ...) on the image
                if self.visualize_projection:
                    # print(data)
                    
                    if self.WORLD2CAM_FRONT is not None:
                        for single_object in data['bounding_boxes']:
                            # print(single_object)
                            if 'location' in single_object:
                                single_object['position'] = transform_to_ego_coordinates(single_object['center'], self.WORLD2CAM_FRONT)
                                
                                if single_object['class'] == 'ego_vehicle':
                                    continue
                                all_points_2d, _ = project_all_corners(single_object, self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)

                                if 'vehicle' in single_object['class']:
                                    # for debug
                                    # all_points_2d = []
                                    # projected_corners, visibility = get_vehicle_projected_corners(self.CAMERA_FRONT, single_object)
                                    # for idx, is_visible in enumerate(visibility):
                                    #     if is_visible:
                                    #         all_points_2d.append(projected_corners[idx])
                                    # debug section ends
                                    color = (255, 0, 0, 0)
                                elif 'traffic_light' in single_object['class'] or 'stop' in single_object['class']:
                                    color = (0, 255, 0, 0)
                                elif 'landmark' in single_object['class']:
                                    color = (0, 0, 255, 0)
                                else:
                                    color = (0, 0, 0, 0)
                                if all_points_2d is not None and len(all_points_2d) > 0:
                                    top_left_point = min(all_points_2d, key=lambda p: (p[0], p[1]))
                                    
                                    for points_2d in all_points_2d:
                                        draw.ellipse((points_2d[0]-5, points_2d[1]-5, points_2d[0]+5, points_2d[1]+5), 
                                                    fill=color)
                                    
                                    label = ""
                                    if 'type_id' in single_object:
                                        label = f"{label}{str(single_object['type_id'])}"
                                    if 'id' in single_object:
                                        label = f"{label}, id={str(single_object['id'])}"

                                    draw.text((top_left_point[0] + 5, top_left_point[1] - 10), label, 
                                                fill=color)
                    annotated_image_path = f'{anno_img_path}/{int(frame_number):05d}.png'
                    image.save(annotated_image_path)
                
                # Save QA data
                file_name = f'{self.output_graph_examples_directory}/graphs/' \
                            f'{scenario_name}/{int(frame_number):05d}.json'
                with open(file_name, 'w', encoding='utf-8') as f:
                    json.dump(qas, f, sort_keys=True, indent=4)

            # TODO: implement an offline method to do stats, 
            # since we do not always generate full dataset in a row.
                    
            # Update minimum number of questions
            self.min_num_questions = min(self.min_num_questions, num_questions)

            # Update statistics
            self.total_num_questions += num_questions
            self.total_num_objects += num_objects
            for key, value in questions_per_category.items():
                self.num_questions_per_category[key] += value

            # Append QA data to the list
            self.all_qa_pairs.append(qas)

            # Populate VQA LLAVA format data
            vqa_llava_entry['image'] = relative_image_path
            vqa_llava_entry['conversations'] = qas
            vqa_llava_entry['conversations']['key_object_infos'] = key_object_infos
            self.vqa_llava_format['image_paths'].append(relative_image_path)
            self.vqa_llava_format['llava_format'].append(vqa_llava_entry)

            self.frame_num += 1

            self.save_processed_path(path)

        # TODO: implement an offline method to do stats, 
        # since we do not always generate full dataset in a row.
            
        # Save statistics
        with open(os.path.join(self.output_graph_directory, 'stats.json'), 'w', encoding="utf-8") as f:
            json.dump({
                'num_frames': self.frame_num,
                'min_num_questions': self.min_num_questions,
                'avg_num_questions': self.total_num_questions / self.frame_num,
                'num_questions': self.total_num_questions, 
                'num_objects': self.total_num_objects, 
                'num_questions_per_category': self.num_questions_per_category,
                'stats_p3': self.stats_p3,
            }, f, indent=4)
        print("Stats saved.")

        # Convert and save VQA LLAVA format data
        return self.vqa_llava_format

    # def is_vehicle_visible_in_image(self, vehicle_obj):
    #     """
    #     Check if a vehicle is visible in the image.
    #     """
    #     # Project the 3D points of the vehicle onto the 2D image plane
    #     projected_2d_points = project_center_corners(vehicle_obj, self.CAMERA_MATRIX)

    #     # Check if any projected point is visible
    #     vehicle_is_visible = False
    #     if projected_2d_points is None:
    #         return False

    #     for point_2d in projected_2d_points:
    #         if (point_2d[0] > self.MIN_X and point_2d[0] < self.MAX_X and
    #             point_2d[1] > self.MIN_Y and point_2d[1] < self.MAX_Y):
    #             vehicle_is_visible = True
    #             break

    #     return vehicle_is_visible

    def project_object_center(self, obj):
        """
        Projects the provided objects center onto the 2D image plane.
        """
        object_position = obj['position']
        object_position_3d = np.array([object_position[1], -object_position[2], object_position[0]])
        rotation_vector = np.zeros((3, 1), np.float32)
        translation_vector = np.array([[0.0, 2.0, 1.5]], np.float32)

        # Define the distortion coefficients
        distortion_coeffs = np.zeros((5, 1), np.float32)

        object_center_2d, _ = cv2.projectPoints(object_position_3d, rotation_vector, translation_vector, 
                                                self.CAMERA_MATRIX, distortion_coeffs)
                
        return object_center_2d[0][0]

    def should_consider_vehicle(self, vehicle):
        """
        True, if it's in front and left cam
        False, if vehicle is not bicycle and the number of points on it are below a threshold
        False, if the vehicle is behind the ego vehicle
        False, if it's a parking vehicle, that does not cut in
        """
        # print('[debug] in should_consider vehicle, vehicle')
        # print(f"[debug] id = {vehicle['id']}, speed = {vehicle['speed']}, type_id = {vehicle['type_id']}, num_points = {vehicle['num_points']}")
        # If the vehicle is parked and not cutting in, exclude it from consideration
        if vehicle['state'] == "static":
            # print('[debug] is static, so no consider')
            return False
        # Max. distance is 25m and similar to the max. distance of junctions
        if  (
            vehicle['position'][0] < -1.5
            or (vehicle['class'] != 'bicycle' and vehicle['num_points'] < 15)
            or (vehicle['num_points'] < 10)
        ):
            # print('[debug] not satisfied')
            return False

        # Check if the vehicle is visible in the image
        # But here only vehicle in front is considered!
        # A pity...
        # but this is only used to identify obstacle in forward route
        # so it's ok
        vehicle_is_visible = is_vehicle_in_camera(self.CAMERA_FRONT, vehicle) or \
                             is_vehicle_in_camera(self.CAMERA_FRONT_LEFT, vehicle)
        # print(f'[debug] visibility: {vehicle_is_visible}')

        return vehicle_is_visible

    def is_object_in_front(self, vehicle):
        """
        True, if it's in front cam
        False, if vehicle is not bicycle and the number of points on it are below a threshold
        False, if the vehicle is behind the ego vehicle
        False, if it's a parking vehicle, that does not cut in
        """
        if vehicle['state'] == "static":
            # print('[debug] is static, so no consider')
            return False
        # Max. distance is 25m and similar to the max. distance of junctions
        if  (
            vehicle['position'][1] > 1.75
            or vehicle['position'][1] < -1.75
            or vehicle['position'][0] < -1.5
            or (vehicle['class'] != 'bicycle' and vehicle['num_points'] < 15)
            or (vehicle['num_points'] < 10)
        ):
            # print('[debug] not satisfied')
            return False

        # Check if the vehicle is visible in the image
        # But here only vehicle in front is considered!
        # A pity...
        # but this is only used to identify obstacle in forward route
        # so it's ok
        vehicle_is_visible = is_vehicle_in_camera(self.CAMERA_FRONT, vehicle)
        # print(f'[debug] visibility: {vehicle_is_visible}')

        return vehicle_is_visible

    def generate_2d_box_from_projected_points(self, projected_points):
        return [
                [round(projected_points[:, 0].min(), 1), round(projected_points[:, 1].min(), 1)],
                [round(projected_points[:, 0].max(), 1), round(projected_points[:, 1].max(), 1)],
            ]

    def generate_object_key_value(self, id, category, visual_description, object_count, 
                                  projected_points=None, projected_points_meters=None):
        """
        Generate a key-value pair representing an object detected in an image, including its category,
        visual description, 2D bounding box coordinates, and position.
        """
        # Create a dictionary to store the object's information
        object_info = {
            'id': id,
            'Category': category,
            'Status': None,
            'Visual_description': visual_description,
            '2d_bbox': None
        }

        # Add the 2D bounding box coordinates, if available
        if projected_points is not None and len(projected_points) > 0:
            object_info['2d_bbox'] = self.generate_2d_box_from_projected_points(projected_points)
            object_info['3d_bbox'] = projected_points_meters.round(1).tolist()

            mean = np.round(np.mean(object_info['2d_bbox'], axis=0), decimals=1)
            center_x = float(mean[0])
            center_y = float(mean[1])

        # Generate a unique key for the object
        if id is None:
            if projected_points is not None and len(projected_points) > 0:
                object_key = f"<e{object_count + 1},CAM_FRONT,{center_x},{center_y}>"
            else:
                object_key = f"<e{object_count + 1},CAM_FRONT>"
        else:
            if projected_points is not None and len(projected_points) > 0:
                object_key = f"<c{id},CAM_FRONT,{center_x},{center_y}>"
            else:
                object_key = f"<c{id},CAM_FRONT>"

        return object_key, object_info

    def add_qas_questions(self, qa_list, chain, layer, qa_type, connection_up, connection_down, 
                          question, answer, object_id=None, object_tags=[]):
        qa_list.append({'chain': chain, 
                        'layer': layer, 
                        'type': qa_type,
                        'object_id': object_id,
                        'connection_up': connection_up, 
                        'connection_down': connection_down, 
                        'from': 'human', 
                        'value': question,
                        'object_tags': object_tags})
        
        qa_list.append({'chain': chain, 
                        'layer': layer, 
                        'type': qa_type,
                        'object_id': object_id,
                        'connection_up': connection_up, 
                        'connection_down': connection_down, 
                        'from': 'gpt', 
                        'value': answer,
                        'object_tags': object_tags})

    def process_pedestrians(self, other_pedestrians, important_objects, object_infos):
        """
        How many pedestrians are there? And where they are roughly (front, left, ...)
        """
        qas_pedestrian = []
        close_pedestrians = []

        object_tags = []
        for pedestrian in other_pedestrians:
            if (
                pedestrian['num_points'] < 5  # Not enough LiDAR points
                or pedestrian['position'][0] < 1  # Too far behind ego vehicle
                or pedestrian['position'][0] > 40  # Too far ahead of ego vehicle
            ):
                # Not enough LiDAR points or far away or occluded
                continue

            close_pedestrians.append(pedestrian)

            # Determine rough position of pedestrian relative to ego vehicle
            important_object_str = get_pedestrian_str(pedestrian)

            important_objects.append(important_object_str)

            # Project pedestrian points and center onto the image plane
            projected_points, projected_points_meters = project_all_corners(pedestrian, self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)

            # Generate key-value pair for the pedestrian object
            key, value = self.generate_object_key_value(
                id=pedestrian['id'],
                category='Pedestrian',
                visual_description=f'Pedestrian',
                object_count=len(object_infos),
                projected_points=projected_points,
                projected_points_meters=projected_points_meters
            )
            object_infos[key] = value
            object_tags.append(key)

        question = "How many pedestrians are there?"
        s_or_no_s = 's' if len(close_pedestrians) > 1 else ''
        are_or_is = 'are' if len(close_pedestrians) > 1 else 'is'

        if len(close_pedestrians) == 0:
            answer = "There are no pedestrians."
        else:
            answer = f"There {are_or_is} {len(close_pedestrians)} pedestrian{s_or_no_s}."

        # Add the question and answer to the conversation
        self.add_qas_questions(qa_list=qas_pedestrian,
                                chain=5,
                                layer=0,
                                qa_type='perception',
                                connection_up=[(6,0)],
                                connection_down=-1,
                                question=question,
                                answer=answer,
                                object_tags=object_tags)

        return qas_pedestrian, important_objects, object_infos

    def process_stop_signs(self, stop_signs, important_objects, object_infos):
        """
        Answers the question:
        Is the ego vehicle affected by a stop sign? (affected by stop sign & distance < 40)
        """
        stop_sign_info = None
        qas_stop_sign = []
        stop_sign_affects_ego = False
        object_tags = []

        for stop_sign in stop_signs:
            if stop_sign['affects_ego'] and stop_sign['distance'] < 40:
                stop_sign_affects_ego = True
                important_objects.append('the stop sign')
                projected_points, projected_points_meters = project_all_corners(stop_sign, self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)
                key, value = self.generate_object_key_value(
                    id=stop_sign['id'],
                    category='Traffic element',
                    visual_description='Stop sign',
                    object_count=len(object_infos),
                    projected_points=projected_points,
                    projected_points_meters=projected_points_meters,
                )
                object_infos[key] = value
                object_tags = [key]
                stop_sign_info = stop_sign
                break

        question = "Is the ego vehicle affected by a stop sign?"
        if stop_sign_affects_ego:
            answer = "Yes, the ego vehicle is affected by a stop sign, which has not been cleared yet."
        else:
            cleared_stop_signs = [x for x in stop_signs if x['distance'] < 4 
                                                        and not x['affects_ego'] 
                                                        and x['position'][0] > -2.6]

            if cleared_stop_signs:
                answer = "Yes, the ego vehicle was affected by a stop sign, which has already been cleared."
            else:
                answer = "No, the ego vehicle is not affected by a stop sign."

        # Add the question and answer to the conversation
        self.add_qas_questions(qa_list=qas_stop_sign, 
                                chain=1,
                                layer=0,
                                qa_type='prediction',
                                connection_up=[(1, 1)], 
                                connection_down=[(3, 0)], 
                                question=question,
                                answer=answer,
                                object_tags=object_tags)

        return qas_stop_sign, important_objects, object_infos, stop_sign_info, object_tags

    def process_traffic_lights(self, traffic_lights, ego_vehicle, important_objects, 
                               object_infos):
        """
        Answers the questions:
        Is the ego vehicle affected by a traffic light?
        What is the state of the traffic light?
        """

        qas_traffic_light = []
        traffic_light_info = None
        object_tags = []

        traffic_light_affects_ego = False

        for traffic_light in traffic_lights:
            if traffic_light['affects_ego']: # and ego_vehicle['traffic_light_state'] != 'None':
                if traffic_light['distance'] < 45:
                    state = traffic_light['state']
                    # for traffic_light
                    # 0 - Red; 1 - Yellow; 2 - Green; 3 - Off; 4 - Unknown;
                    if state == 2:
                        ego_vehicle['traffic_light_state'] = "green"
                        state = "green"
                    elif state == 1:
                        ego_vehicle['traffic_light_state'] = "yellow"
                        state = "yellow"
                    elif state == 0:
                        ego_vehicle['traffic_light_state'] = "red"
                        state = "red"
                    elif state == 3:
                        state = "off"
                        continue
                    else:
                        state = "unknown"
                        continue
                    # state = state[:1].lower() + state[1:]
                    traffic_light_affects_ego = True
                    traffic_light_info = traffic_light
                    break

        question = "Is the ego vehicle affected by a traffic light?"
        if traffic_light_affects_ego:
            answer = "Yes, the ego vehicle is affected by a traffic light."
            important_objects.append(f'the {state} traffic light')
            projected_points, projected_points_meters = project_all_corners(traffic_light, self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)
            visual_description = f'{ego_vehicle["traffic_light_state"]} traffic light'
            key, value = self.generate_object_key_value(id=traffic_light['id'],
                                                        category='Traffic element',
                                                        visual_description=visual_description,
                                                        object_count=len(object_infos),
                                                        projected_points=projected_points,
                                                        projected_points_meters=projected_points_meters)
            object_infos[key] = value
            object_tags = [key]
        else:
            answer = "No, the ego vehicle is not affected by a traffic light."

        # Add the question and answer to the conversation
        self.add_qas_questions(qa_list=qas_traffic_light, 
                            chain=2,
                            layer=0,
                            qa_type='perception',
                            connection_up=[(2, 1)], 
                            connection_down=[(3, 0)], 
                            question=question,
                            answer=answer,
                            object_tags=object_tags)

        # Add the question about the traffic light state
        question = "What is the state of the traffic light?"
        if traffic_light_affects_ego:
            answer = f"The traffic light is {state}."
        else:
            answer = "There is no traffic light affecting the ego vehicle."

        # Add the question and answer to the conversation
        self.add_qas_questions(qa_list=qas_traffic_light, 
                            chain=2,
                            layer=1,
                            qa_type='prediction',
                            connection_up=[(2, 2)], 
                            connection_down=[(2, 0)], 
                            question=question,
                            answer=answer,
                            object_tags=object_tags)

        return qas_traffic_light, important_objects, object_infos, traffic_light_info, object_tags

    def get_key_of_key_object(self, key_object_infos, object_dict=None):
        if object_dict is not None:
            projected_points, _ = project_all_corners(object_dict, self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)
            if projected_points is not None and len(projected_points) > 0:
                two_d_box = self.generate_2d_box_from_projected_points(projected_points)
                keys = [k for k, v in key_object_infos.items() if two_d_box==v['2d_bbox']]

                return keys
        
        return []

    def generate_ego_vehicle_actions(self, ego_vehicle_data, pedestrians, ego_data, important_objects, key_object_infos, 
                                     vehicles_by_id, traffic_light_info, stop_sign_info, static_objects, 
                                     measurements, scene_data, scenario_name, stop_signs, stop_sign_object_tags, 
                                     traffic_light_object_tags):
        """
        Answers the questions:
        Does the ego vehicle need to brake? Why?
        What should the ego vehicle do based on the {actor_name}?
        What is the current speed limit?

        Args:
            ego_vehicle_data (dict): A dictionary containing information about the ego vehicle.
            ego_data (dict): A dictionary containing additional information about the ego vehicle.
            important_objects (list): A list of important objects in the scene.
            key_object_infos (dict): A dictionary containing information about key objects in the scene.
            vehicles_by_id (dict): A dictionary mapping vehicle IDs to vehicle information.
            traffic_light_info (dict): A dictionary containing information about the traffic light.
            stop_sign_info (dict): A dictionary containing information about the stop sign.
            static_objects (list): A list of static objects in the scene.
            landmarks (list): A list of landmarks in the scene.
            measurements (dict): A dictionary containing sensor measurements.
            scenario_name (str): The name of the current scenario.

        Returns:
            tuple: A tuple containing:
                - ego_actions (list): A list of dictionaries representing actions the ego vehicle should take.
                - important_objects (list): The updated list of important objects in the scene.
                - key_object_infos (dict): The updated dictionary containing information about key objects in the scene.
        """

        def add_speed_limit_question(qas_conversation_ego, measurements):        
            """
            Answers "What is the current speed limit?".

            Args:
                qas_conversation_ego (list): A list of dictionaries representing the conversation.
                measurements (dict): A dictionary containing sensor measurements.

            Returns:
                None
            """
            question = "What is the current speed limit?"

            speed_limit = int(measurements['speed_limit'])
            if speed_limit >= 999:
                answer = f"There's no speed limit now."
            else:
                answer = f"The current speed limit is {speed_limit} km/h."

            self.add_qas_questions(qa_list=qas_conversation_ego,
                                   chain=3,
                                   layer=7,
                                   qa_type='perception',
                                   connection_up=[(6, 0)] ,
                                   connection_down=[(3, 2)],
                                   question=question,
                                   answer=answer)


        def get_rough_position(actor):
            actor_rel_position = transform_to_ego_coordinates(actor['location'], ego_data['world2ego'])

            if -2 <= actor_rel_position[1] <= 2:
                rough_pos_str = 'to the front of it'
            elif actor_rel_position[1] > 2:
                rough_pos_str = 'to the front right'
            elif actor_rel_position[1] < -2:
                rough_pos_str = 'to the front left'
            else:
                rough_pos_str = 'at an unknown position'
            
            return rough_pos_str
    
        def get_vehicle_type(vehicle):
            return vehicle.get('base_type') if vehicle.get('base_type') is not None else 'vehicle'
        
        def get_vehicle_color(vehicle):
            color = rgb_to_color_name(vehicle.get("color")) + ' ' if vehicle.get("color") is not None and \
                                                vehicle.get("color") != 'None' else ''
            return color

        def determine_braking_requirement(qas_conversation_ego, pedestrians, measurements, scene_data, vehicles, ego_vehicle, 
                                          scenario_type, traffic_light_info, stop_sign_info, static_objects):
            """
            This function has massive modification because the agent we use is not rule-based.
            Answers "Does the ego vehicle need to brake? Why?".

            Args:
                qas_conversation_ego (list): A list of dictionaries representing the conversation.
                measurements (dict): A dictionary containing sensor measurements.
                vehicles (dict): A dictionary mapping vehicle IDs to vehicle information.
                ego_vehicle (dict): A dictionary containing information about the ego vehicle.
                is_highway (bool): Whether the scenario is on a highway or not.
                scenario_type (str): The type of scenario being evaluated.
                bicycle_scenario (bool): Whether the scenario involves a bicycle or not.
                bicycle_in_junction (bool): Whether the bicycle is in a junction or not.
                blocked_intersection_scenario (bool): Whether the scenario involves a blocked intersection or not.
            traffic_light_info (dict): A dictionary containing information about the traffic light.
            stop_sign_info (dict): A dictionary containing information about the stop sign.

            Returns:
                None
            """
            question = "Does the ego vehicle need to brake? Why?"
            answer = "There is no reason for the ego vehicle to brake."

            object_tags = []

            full_scenario_name = scenario_type
            if self.last_full_scenario_name != full_scenario_name:
                self.leftmost_pos_of_left_hazard = {}
            self.last_full_scenario_name = full_scenario_name
            scenario_type = scenario_type.split('_')[0]
            scenario_name = scenario_type # why is it not unified?

            acc = get_acceleration_by_future(self.current_measurement_path, 6)
            flags = get_affect_flags(scene_data)

            predict_second = 2.5
            predict_frame = int(predict_second * self.frame_rate)
            predict_distance = max(ego_vehicle['speed'] * predict_second, 10.0)
            static_threshold = 10.0

            print(f"[debug] current frame is {self.current_measurement_index}")
            # hazardous_walkers = get_walker_hazard_with_prediction(scene_data, prediction_time=15)
            # hazardous_actors = get_all_hazard_with_prediction_sorted(scene_data, prediction_time=15) 
            hazardous_walkers = get_hazard_by_future(self.current_measurement_path, self.map, 
                                                     k=predict_frame, filter='walker', max_distance=predict_distance)
            hazardous_actors = get_hazard_by_future(self.current_measurement_path, self.map, 
                                                     k=predict_frame, filter=None, max_distance=predict_distance)
            static_hazardous_actors = get_hazard_by_future(self.current_measurement_path, self.map, 
                                                     k=1, filter=None, max_distance=static_threshold)

            cuts_in_actor_ids = [actor['id'] for actor in scene_data if actor.get('vehicle_cuts_in') and actor['distance'] < 20.0]
            hazardous_actor_ids = [actor['id'] for actor in hazardous_actors]
            static_hazardous_ids = [actor['id'] for actor in static_hazardous_actors if actor['distance'] < static_threshold]

            combined_actors = [
                actor for actor in scene_data if actor.get('id') in hazardous_actor_ids or \
                    actor.get('id') in cuts_in_actor_ids or \
                    actor.get('id') in static_hazardous_ids
            ]
            hazardous_actors = sorted(combined_actors, key=lambda actor: actor.get('distance', float('inf')))
            print(f"[debug] hazardous_actors = {hazardous_actors}")

            hazardous = len(hazardous_actors) > 0
            measurements['speed_limit'] = get_speed_limit(scene_data) # km/h
            measurements['vehicle_hazard'] = hazardous and 'vehicle' in hazardous_actors[0]['class']
            if (hazardous):
                measurements['speed_reduced_by_obj'] = hazardous_actors[0]
                measurements['speed_reduced_by_obj_id'] = hazardous_actors[0]['id']
                measurements['speed_reduced_by_obj_type'] = hazardous_actors[0]['type_id']
                measurements['speed_reduced_by_obj_distance'] = hazardous_actors[0]['distance']
                ######## for [debug] ########
                self.appended_measurements['speed_reduced_by_obj'] = measurements['speed_reduced_by_obj']
                self.appended_measurements['speed_reduced_by_obj_id'] = measurements['speed_reduced_by_obj_id']
                self.appended_measurements['speed_reduced_by_obj_type'] = measurements['speed_reduced_by_obj_type']
                self.appended_measurements['speed_reduced_by_obj_distance'] = measurements['speed_reduced_by_obj_distance']
                ######## for [debug] ########

            ######## for [debug] ########
            self.appended_measurements['future_acceleration'] = acc
            self.appended_measurements['affect_flags'] = flags
            self.appended_measurements['hazardous_walkers'] = hazardous_walkers
            self.appended_measurements['hazardous_actors'] = hazardous_actors
            self.appended_measurements['hazardous_flag'] = hazardous
            self.appended_measurements['speed_limit'] = measurements['speed_limit']
            self.appended_measurements['vehicle_hazard_flag'] = measurements['vehicle_hazard']
            ######## for [debug] ########
            
            
            # speed / speed_limit > 1.031266635497984, done by the controller
            limit_speed = float(measurements['speed_limit']) / 3.6
            junction_speed = 64. / 3.6
            
            if measurements['speed'] / limit_speed > 1.031266635497984:
                answer = "The ego vehicle should brake because it is faster than speed limit."

            elif ego_vehicle['is_in_junction'] and measurements['speed'] / junction_speed > 1.031266635497984:
                answer = "The ego vehicle should brake because it should slow down in junction."

            elif flags['affected_by_stop_sign'] is True:
                answer = "The ego vehicle should stop because of the stop sign."
                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=stop_sign_info)
            
            elif flags['affected_by_red_light'] is True:
                answer = "The ego vehicle should stop because of the traffic light that is red."
                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=traffic_light_info)
            
            elif flags['affected_by_yellow_light'] is True:
                answer = "The ego vehicle should slow down because of the traffic light that is yellow."
                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=traffic_light_info)
            
            if (measurements['control_brake'] or acc is "Decelerate") or hazardous:
                if len(hazardous_walkers) and scenario_name not in \
                    ['DynamicObjectCrossing', 'ParkingCrossingPedestrian', 'PedestrianCrossing', 'VehicleTurningRoutePedestrian']:
                    closest_pedestrian_idx = np.argmin([x['distance'] for x in pedestrians])
                    closest_pedestrian = pedestrians[closest_pedestrian_idx]
                    closest_pedestrian_distance = closest_pedestrian['distance']
                    brake_or_slow_down = 'stop' if closest_pedestrian_distance < 10 else 'slow down'
                    object_tags = self.get_key_of_key_object(key_object_infos, object_dict=closest_pedestrian)
                    if len(pedestrians) > 1:
                        answer = f"The ego vehicle should {brake_or_slow_down} because of the pedestrians "\
                                    "that are crossing the road."
                    else:
                        answer = f"The ego vehicle should {brake_or_slow_down} because of the pedestrian "\
                                    "that is crossing the road."
                
                # beware! below contents are scenario-specified. take special care of them.
                else:
                    vehicles_in_front = []

                    lane_str = 'opposite lane' if ego_data['lane_change'] in [0] else 'nearby lane'
                    check_back = False if ego_data['lane_change'] in [0] else True
                    occupied_str = ', which is occupied,' if get_clear_distance_of_lane(vehicles_by_id.values(), -1, check_back) else ', which is busy,'
                    if not (ego_data['speed'] > 3.0 and abs(measurements['steer']) > 0.5):
                        vehicles_in_front = [x for x in vehicles_by_id.values() if self.is_object_in_front(x)]
                        vehicles_in_front = sorted(vehicles_in_front, key=lambda actor: actor.get('distance', float('inf')))

                    if hazardous and 'AccidentTwoWays' in scenario_type \
                            and 'vehicle.dodge.charger_police_2020' == measurements['speed_reduced_by_obj_type']:
                        police_cars = [x for x in hazardous_actors if x['type_id'] == 'vehicle.dodge.charger_police_2020']
                        car_id = [police_cars[0]['id']]
                        if car_id not in self.leftmost_pos_of_left_hazard:
                            self.leftmost_pos_of_left_hazard[car_id] = self.inf_num
                        self.leftmost_pos_of_left_hazard[car_id] = \
                            min(self.leftmost_pos_of_left_hazard[car_id], police_cars[0]['position'][1])
                        delta = 1.2 if self.leftmost_pos_of_left_hazard[car_id] < 1.2 else 2
                        police_cars = [x for x in police_cars if x['position'][1] < self.leftmost_pos_of_left_hazard[car_id] + delta]
                        if police_cars:
                            object_tags = self.get_key_of_key_object(key_object_infos, object_dict=police_cars[0])
                            answer = f"The ego vehicle should stop because it must invade the {lane_str}{occupied_str}"\
                                    " in order to bypass the accident."
                    elif hazardous and 'ConstructionObstacleTwoWays' in scenario_type \
                                    and 'static.prop.trafficwarning' == measurements['speed_reduced_by_obj_type']:
                        traffic_warnings = [x for x in hazardous_actors if x['type_id'] == 'static.prop.trafficwarning']
                        if traffic_warnings:
                            wid = traffic_warnings[0]['id']
                            if wid not in self.leftmost_pos_of_left_hazard:
                                self.leftmost_pos_of_left_hazard[wid] = self.inf_num
                            self.leftmost_pos_of_left_hazard[wid] = min(self.leftmost_pos_of_left_hazard[wid], traffic_warnings[0]['position'][1])
                            delta = 1.2 if self.leftmost_pos_of_left_hazard[wid] < 1.2 else 2
                            traffic_warnings = [x for x in traffic_warnings if x['position'][1] < self.leftmost_pos_of_left_hazard[wid] + delta]
                            if traffic_warnings:
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=traffic_warnings[0])

                            answer = f"The ego vehicle should stop because it must invade the {lane_str}{occupied_str}" \
                                        " in order to bypass the construction warning."
                        
                    elif hazardous and 'ParkedObstacleTwoWays' in scenario_type \
                                    and measurements['speed_reduced_by_obj_id'] in vehicles:
                        vehicle = vehicles[measurements['speed_reduced_by_obj_id']]
                        object_tags = self.get_key_of_key_object(key_object_infos, object_dict=vehicle)
                        answer = f"The ego vehicle should stop because it must invade the {lane_str}{occupied_str}" \
                                        " in order to bypass the parked vehicle."
                    elif 'VehicleOpensDoorTwoWays' in scenario_type:
                        vehicles_open_door = [v for v in vehicles_by_id.values() if v['type_id'] == 'vehicle.mercedes.coupe_2020' 
                                    and v['position'][0] > 0
                                    and v['position'][1] > 0.5 
                                    and (float(v['distance']) <= 22.0 and v['speed'] < 0.1) and is_vehicle_in_camera(self.CAMERA_FRONT, v)]
                        if vehicles_open_door:
                            vehicle = vehicles_open_door[0]
                            object_tags = self.get_key_of_key_object(key_object_infos, object_dict=vehicle)
                            answer = f"The ego vehicle should stop because it must invade the {lane_str}{occupied_str}" \
                                                        " in order to bypass the vehicle with the opened doors."
                    elif 'HazardAtSideLaneTwoWays' in scenario_type:
                        bicycles = [v for v in vehicles_by_id.values() if v['base_type'] == 'bicycle' 
                                                                        and self.should_consider_vehicle(v) 
                                                                        and float(v['distance']) < 40
                                                                        and v['lane_relative_to_ego'] == 0]
                        if bicycles:
                            bicycles.sort(key=lambda x:x['distance'])
                            closest_bicycle = bicycles[0]

                            brake_or_stop = 'stop' if measurements['speed'] < 0.5 else 'brake'
                            if closest_bicycle['distance'] < 40:
                                answer = f"The ego vehicle should {brake_or_stop} because it must invade the {lane_str}{occupied_str}" \
                                            " in order to bypass the bicycles."
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=closest_bicycle)
                    elif 'DynamicObjectCrossing' in scenario_name or 'ParkingCrossingPedestrian' in scenario_name or \
                        'PedestrianCrossing' in scenario_name or 'VehicleTurningRoutePedestrian' in scenario_name:
                        # in definition, DynamicObjectCrossing's obstacle might be a bicycle
                        # but it does not exist in b2d
                        peds = [x for x in pedestrians if 0.0 < x['position'][0] < 30.0
                                and abs(x['speed']) > 1.0
                                and -1.75 < x['position'][1] < 3.5] # carla lane width is 3.5m
                        if peds:
                            closest_pedestrian_idx = np.argmin([x['distance'] for x in peds])
                            closest_pedestrian = peds[closest_pedestrian_idx]
                            closest_pedestrian_distance = closest_pedestrian['distance']
                            brake_or_slow_down = 'stop' if closest_pedestrian_distance < 10.0 else 'slow down'
                            if len(peds) > 1:
                                answer = f"The ego vehicle should {brake_or_slow_down} because of the pedestrians "\
                                            "that are crossing the road."
                            else:
                                answer = f"The ego vehicle should {brake_or_slow_down} because of the pedestrian "\
                                            "that is crossing the road."
                            for ped in peds:
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=ped)

                        # print(f"[debug] relevant_objects = {relevant_objects}") # debug1

                    elif vehicles_in_front:
                        print(f'[debug] in branch in front, vehicles_in_front = {vehicles_in_front}')
                        brake_due_to_leading_vehicle = not measurements['vehicle_hazard']
                        is_highway = False

                        # List of highway scenarios
                        highway_scenarios = [
                            "EnterActorFlow", 
                            "EnterActorFlowV2", 
                            "HighwayCutIn", 
                            "HighwayExit", 
                            "MergerIntoSlowTraffic",
                            "MergerIntoSlowTrafficV2",
                            "YieldToEmergencyVehicle",
                        ]

                        speed_limit = int(measurements['speed_limit'])

                        if scenario_name in highway_scenarios and speed_limit > 50:
                            is_highway = True

                        bike_scenario = False
                        blocked_intersection_scenario = False
                        for vehicle in vehicles_in_front:
                            # find bicycles that are of type scenario
                            if 'bicycle' in vehicle['class'] and \
                                            (ego_data['distance_to_junction'] < 10 or ego_data['is_in_junction']) and \
                                            scenario_name == 'CrossingBicycleFlow':
                                bike_scenario = True
                                color = rgb_to_color_name(vehicle["color"]) + ' ' if vehicle["color"] is not None and \
                                                                                vehicle["color"] != 'None' else ''
                                vehicletype = vehicle['base_type']
                                vehicle_rel_position = transform_to_ego_coordinates(vehicle['location'], ego_data['world2ego'])
                                if vehicle_rel_position[1] < 2 and vehicle_rel_position[1] > -2:
                                    rough_pos_str = 'to the front of it'
                                elif vehicle_rel_position[1] > 2:
                                    rough_pos_str = 'to the front right'
                                elif vehicle_rel_position[1] < -2:
                                    rough_pos_str = 'to the front left'
                                else:
                                    rough_pos_str = 'at an unknown position'
                            elif vehicle['distance'] < 15 and scenario_name == 'BlockedIntersection':
                                blocked_intersection_scenario = True
                                print(f'[debug] blocked_intersection_scenario is True')
                                    

                        actor_hazard = vehicles_in_front[0]
                        
                        color = get_vehicle_color(actor_hazard)
                        vehicletype = get_vehicle_type(actor_hazard)
                        rough_pos_str = get_rough_position(actor_hazard)

                        consider_vehicle = self.should_consider_vehicle(actor_hazard)
                        if actor_hazard['speed'] < 0.5:
                            brake_stop_str = "stop"
                        else:
                            brake_stop_str = "brake"

                        # Determine if there is no reason for the ego vehicle to brake
                        if actor_hazard['num_points'] < 3 or not consider_vehicle:
                            answer = "There is no reason for the ego vehicle to brake."
                        # Handle the case where the hazard vehicle is a leading vehicle
                        elif brake_due_to_leading_vehicle and actor_hazard['distance'] < 20.:

                            if actor_hazard['speed'] < 0.5:
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                                answer = "The ego vehicle should stop because of the " + f"{color}{vehicletype} " +\
                                                                                            f"that is {rough_pos_str}."
                            else:
                                if actor_hazard['base_type'] == 'bicycle':
                                    object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                                    answer = "The ego vehicle should slow down because of the " +\
                                                                        f"{color}{vehicletype} that is {rough_pos_str}."
                                else:
                                    object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                                    answer = "The ego vehicle should adjust its speed to the speed of the " +\
                                                                        f"{color}{vehicletype} that is {rough_pos_str}."
                        # Handle the case where the scenario is on a highway
                        elif is_highway:
                            object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                            answer = f"The ego vehicle should {brake_stop_str} because of the " +\
                                                                    f"{color}{vehicletype} that is {rough_pos_str}."
                        # Handle the case where the scenario is not on a highway
                        else:
                            # Check if the ego vehicle is in a junction or near a junction, and the hazard vehicle is
                            # on a different road
                            if (ego_vehicle['is_in_junction'] or (ego_vehicle['distance_to_junction'] is not None and \
                                                                    ego_vehicle['distance_to_junction'] < 10)) and \
                                                                    actor_hazard['road_id'] != ego_vehicle['road_id']:
                                actor_rel_position = transform_to_ego_coordinates(actor_hazard['location'], ego_data['world2ego'])
                                # Determine the direction of the hazard vehicle relative to the junction
                                if actor_rel_position[1] < -8:
                                    direction_junction = "on the left side of the junction"
                                # right
                                elif actor_rel_position[1] > 8:
                                    direction_junction = "on the right side of the junction"
                                elif actor_hazard['road_id'] != ego_vehicle['road_id']:
                                    direction_junction = "on the opposite side of the junction"
                                else:
                                    # raise ValueError(f"Unknown position of vehicle {vehicle['id']}.")
                                    raise ValueError(f"Unknown position of vehicle.")
                                
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                                answer = f"The ego vehicle should {brake_stop_str} because of the {color}" \
                                                                        f"{vehicletype} that is {direction_junction}."
                            # Handle other cases
                            else:
                                if actor_hazard['vehicle_cuts_in']:
                                    object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                                    answer = f"The ego vehicle should {brake_stop_str} because of the {color}"\
                                                        f"{vehicletype} that is cutting into the ego vehicle's lane."
                                else:
                                    object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                                    answer = f"The ego vehicle should {brake_stop_str} because of the {color}" \
                                                                            f"{vehicletype} that is {rough_pos_str}."

                        # Special cases for specific scenarios
                        if scenario_type == 'BlockedIntersection' and blocked_intersection_scenario:
                            object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                            answer = f"The ego vehicle should slow down because of the {color}{vehicletype} that is " +\
                                                                    f"{rough_pos_str} and is blocking the intersection."

                        if hazardous:
                            if scenario_type == 'CrossingBicycleFlow' and bike_scenario:
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                                answer = f"The ego vehicle should slow down because of the {color}{vehicletype} " +\
                                                            f"that is {rough_pos_str} and is crossing the intersection."
                            
                            if scenario_type == "InterurbanActorFlow" and ego_vehicle['is_in_junction']:
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=actor_hazard)
                                answer = f"The ego vehicle should stop because of the {color}{vehicletype} that " +\
                                                f"is on the oncoming lane and is crossing paths with the ego vehicle."

            elif ego_vehicle['hazard_detected_40']:
                leading_vehicle_id = ego_vehicle['affects_ego_40']

                if leading_vehicle_id is not None:
                    leading_vehicle = vehicles[leading_vehicle_id]
                    consider_vehicle = self.should_consider_vehicle(leading_vehicle)
                    if consider_vehicle:
                        color = get_vehicle_color(leading_vehicle)
                        vehicletype = get_vehicle_type(leading_vehicle)
                        rough_pos_str = get_rough_position(leading_vehicle)
                        if measurements['speed'] < (1 / 3.6) * 0.9 * measurements['speed_limit'] \
                                                            and measurements['throttle'] < 0.9:
                            object_tags = self.get_key_of_key_object(key_object_infos, object_dict=leading_vehicle)
                            if leading_vehicle['base_type'] == 'bicycle':
                                answer = "The ego vehicle should slow down because of the " +\
                                                                    f"{color}{vehicletype} that is {rough_pos_str}."
                            else:       
                                answer = "The ego vehicle should adjust its speed to the speed of the " +\
                                                                    f"{color}{vehicletype} that is {rough_pos_str}."
                    
                    # Special cases for specific scenarios
                    if leading_vehicle['distance'] < 15 and scenario_name == 'BlockedIntersection':
                        object_tags = self.get_key_of_key_object(key_object_infos, object_dict=leading_vehicle)
                        answer = f"The ego vehicle should stop because of the {color}{vehicletype} that is " +\
                                                                    f"{rough_pos_str} and is blocking the intersection."

            # if answer == "There is no reason for the ego vehicle to brake." and (measurements['control_brake'] or acc is 'Decelerate'):
            if measurements['control_brake'] or acc is 'Decelerate':
                print(f"[debug] into second branch, scenario_name = {scenario_name}")
                if hazardous and scenario_name == 'Accident':
                    police_cars = [x for x in hazardous_actors if x['type_id'] == 'vehicle.dodge.charger_police_2020']
                    if police_cars:
                        pid = police_cars[0]['id']
                        if pid not in self.leftmost_pos_of_left_hazard:
                                self.leftmost_pos_of_left_hazard[pid] = self.inf_num
                        self.leftmost_pos_of_left_hazard[pid] = min(self.leftmost_pos_of_left_hazard[pid], police_cars[0]['position'][1])
                        delta = 1.2 if self.leftmost_pos_of_left_hazard[pid] < 1.2 else 2
                        police_cars = [x for x in police_cars if x['position'][1] < self.leftmost_pos_of_left_hazard[pid] + delta]
                        if police_cars:
                            police_car = list(sorted(police_cars, key=lambda x: x['distance']))[0]
                            brake_or_stop = 'stop' if measurements['speed'] < 0.5 else 'brake'
                            if police_car['distance'] < 40:
                                answer = f"The ego vehicle should {brake_or_stop} because it must change the lane to "\
                                                                                                    f"bypass the accident."
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=police_car)
                elif hazardous and scenario_name == 'ConstructionObstacle':
                    traffic_warnings = [x for x in hazardous_actors if 'type_id' in x 
                                                    and x['type_id'] == 'static.prop.trafficwarning']
                    if traffic_warnings:
                        wid = police_cars[0]['id']
                        if wid not in self.leftmost_pos_of_left_hazard:
                                self.leftmost_pos_of_left_hazard[wid] = self.inf_num
                        self.leftmost_pos_of_left_hazard[wid] = min(self.leftmost_pos_of_left_hazard[wid], traffic_warnings[0]['position'][1])
                        delta = 1.2 if self.leftmost_pos_of_left_hazard[wid] < 1.2 else 2
                        traffic_warnings = [x for x in traffic_warnings if x['position'][1] < self.leftmost_pos_of_left_hazard[wid] + delta]
                        if traffic_warnings:
                            assert len(traffic_warnings) == 1
                            traffic_warning = traffic_warnings[0]

                            brake_or_stop = 'stop' if measurements['speed'] < 0.5 else 'brake'
                            if traffic_warning['distance'] < 40:
                                answer = f"The ego vehicle should {brake_or_stop} because it must change the lane to "\
                                            f"bypass the construction warning."
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=traffic_warning)
                elif scenario_name == 'HazardAtSideLane':
                    bicycles = [v for v in vehicles_by_id.values() if v['base_type'] == 'bicycle' 
                                                                        and self.should_consider_vehicle(v) 
                                                                        and float(v['distance']) < 40
                                                                        and v['lane_relative_to_ego'] == 0]
                    print(f'[debug] in HazardAtSideLane, bicycles = {bicycles}')
                    if bicycles:
                        bicycles.sort(key=lambda x:x['distance'])
                        closest_bicycle = bicycles[0]

                        brake_or_stop = 'stop' if measurements['speed'] < 0.5 else 'brake'
                        if closest_bicycle['distance'] < 40:
                            answer = f"The ego vehicle should {brake_or_stop} because it must change the lane " \
                                        "to bypass the two bicycles."
                            object_tags = self.get_key_of_key_object(key_object_infos, object_dict=closest_bicycle)
                elif hazardous and scenario_name == 'ParkedObstacle':
                    parked_vehicles = [x for x in hazardous_actors if 'speed' in x and abs(x['speed']) < 0.5]
                    if parked_vehicles:
                        pid = parked_vehicles[0]['id']
                        if pid not in self.leftmost_pos_of_left_hazard:
                                self.leftmost_pos_of_left_hazard[pid] = self.inf_num
                        self.leftmost_pos_of_left_hazard[pid] = min(self.leftmost_pos_of_left_hazard[pid], parked_vehicles[0]['position'][1])
                        delta = 1.2 if self.leftmost_pos_of_left_hazard[pid] < 1.2 else 2
                        print(f'[debug] delta = {delta} left_most = {self.leftmost_pos_of_left_hazard}')
                        parked_vehicles = [x for x in parked_vehicles if x['position'][1] < self.leftmost_pos_of_left_hazard[pid] + delta]
                        if parked_vehicles:
                            print(f'[debug] finally, parked_obstacles = {parked_vehicles}')
                            # assert len(parked_vehicles) == 1
                            parked_vehicle = parked_vehicles[0]

                            brake_or_stop = 'stop' if measurements['speed'] < 0.5 else 'brake'
                            if parked_vehicle['distance'] < 40:
                                answer = f"The ego vehicle should {brake_or_stop} because it must change the lane to " \
                                                "bypass the parked vehicle."
                                object_tags = self.get_key_of_key_object(key_object_infos, object_dict=parked_vehicle)

            
            if answer == "There is no reason for the ego vehicle to brake.":
                if stop_sign_info and stop_sign_info['affects_ego'] and stop_sign_info['distance'] < 40:
                    answer = "The ego vehicle should slow down and stop at the stop sign."
                    object_tags = self.get_key_of_key_object(key_object_infos, object_dict=stop_sign_info)
            
            if answer == "There is no reason for the ego vehicle to brake.":
                if abs(ego_data['speed']) <= 0.01:
                    answer = "There is no reason for the ego vehicle to brake because ego vehicle is already static."


            self.add_qas_questions(qa_list=qas_conversation_ego, 
                                chain=6,
                                layer=0,
                                qa_type='planning',
                                connection_up=-1,
                                connection_down=[(1,1), (2,2), (3,5), (3,6), (3,7), (3,8), (4,3), (5,0)],
                                question=question,
                                answer=answer,
                                object_tags=object_tags)

        def determine_ego_action_based_on_actor(actor, actor_type, ego_speed, ego_vehicle, qas_conversation_ego, 
                                                stop_signs, object_tags):
            """
            Answers "What should the ego vehicle do based on the {actor_type}?".

            Args:
                actor (dict): A dictionary containing information about the actor (traffic light or stop sign).
                actor_type (str): The type of actor ('traffic light' or 'stop sign').
                ego_speed (float): The current speed of the ego vehicle.
                ego_vehicle (dict): A dictionary containing information about the ego vehicle.
                measurements (dict): A dictionary containing sensor measurements.
                qas_conversation_ego (list): A list of dictionaries representing the conversation.

            Returns:
                None
            """
            question = f"What should the ego vehicle do based on the {actor_type}?"

            # Check if the actor is present
            if actor is None:
                if actor_type == 'traffic light':
                    answer = f"There is no {actor_type} affecting the ego vehicle."
                else:
                    cleared_stop_signs = [x for x in stop_signs if x['distance'] < 4 
                                                                and not x['affects_ego'] 
                                                                and x['position'][0] > -2.6]

                    if cleared_stop_signs:
                        answer = "The ego vehicle was affected by a stop sign, which has already been cleared."
                    else:
                        answer = f"There is no {actor_type} affecting the ego vehicle."
            else:
                answer = f"The ego vehicle should follow the {actor_type}."
                distances = [10, 15, 20, 40]

                # Determine the action based on the ego vehicle's speed for red and green states
                if ego_speed > 5:
                    red_str_speed = f'slow down and stop at the {actor_type}' 
                else: 
                    red_str_speed = 'remain stopped'
                    
                if ego_speed < 5:
                    green_str_speed = 'accelerate'
                else:
                    green_str_speed = 'maintain its speed'

                for dist in distances:
                    # Initialize the actor's state string if it doesn't exist
                    if 'state_str' not in actor:
                        actor['state_str'] = ''

                    # Check if the actor is within the current distance
                    if actor['distance'] < dist:
                        # Check if there is a leading vehicle affecting the ego vehicle
                        if ego_vehicle[f'hazard_detected_{dist}']:
                            # Handle the case where there is a leading vehicle
                            if actor_type == 'traffic light':
                                # Handle different traffic light states
                                if actor['state_str'] == 'Green':
                                    answer = f"Based on the green traffic light the ego vehicle can " +\
                                        f"{green_str_speed} and continue driving but should pay attention to the " +\
                                        f"vehicle in front and adjust its speed accordingly."
                                elif actor['state_str'] == 'Yellow':
                                    answer = f"The ego vehicle should slow down and prepare to stop at the " +\
                                                                                                f"traffic light."
                                elif actor['state_str'] == 'Red':
                                    answer = f"The ego vehicle should {red_str_speed} and stay behind other " +\
                                                                    f"vehicles that are standing at the red light."
                                else:
                                    answer = "The ego vehicle should follow the traffic light."
                            else:
                                # Handle the stop sign case
                                answer = f"The ego vehicle should {red_str_speed} and stay behind other vehicles " +\
                                                                                f"that are standing at the stop sign."
                        else:
                            # Handle the case where there is no leading vehicle
                            if actor_type == 'traffic light':
                                # Handle different traffic light states
                                if actor['state_str'] == 'Green':
                                    answer = f"The ego vehicle can {green_str_speed} and continue driving because " +\
                                                                                        f"the traffic light is green."
                                elif actor['state_str'] == 'Yellow':
                                    answer = f"The ego vehicle should slow down and prepare to stop at the " +\
                                                                                                    f"traffic light."
                                elif actor['state_str'] == 'Red':
                                    answer = f"The ego vehicle should {red_str_speed}."
                                else:
                                    answer = "The ego vehicle should follow the traffic light."
                            else:
                                # Handle the stop sign case
                                if ego_speed <0.1 and not actor['affects_ego'] and actor['distance'] < 3:
                                    answer = f"The ego vehicle can accelerate and continue driving if the " +\
                                                "intersection is clear because it has already stopped at the stop sign."
                                else:
                                    answer = f"The ego vehicle should {red_str_speed}."
                        break # Break out of the loop since the actor has been handled
                    else:
                        answer = f"The {actor_type} is too far away to affect the ego vehicle."

            # Set the chain, layer, and connection values based on the actor type
            if actor_type == 'traffic light':
                chain = 2
                layer = 2
                connection_up = [(6,0)]
                connection_down = [(2,1)]
            else:
                chain = 1
                layer = 1
                connection_up = [(6,0)]
                connection_down = [(1,0)]

            # Add the question and answer to the conversation
            self.add_qas_questions(qa_list=qas_conversation_ego, 
                                chain=chain, 
                                layer=layer, 
                                qa_type='planning',
                                connection_up=connection_up, 
                                connection_down=connection_down, 
                                question=question,
                                answer=answer,
                                object_tags=object_tags)

        def determine_whether_ego_needs_to_change_lanes_due_to_obstruction(qas_conversation_ego,
                                                                            scenario_name,
                                                                            vehicles_by_id,
                                                                            static_objects,
                                                                            measurements,
                                                                            ego_data,
                                                                            important_objects, key_object_infos): 

            relevant_objects = []
            multiple_cones = False
            relevant_obj = None
            object_tags = []
            scenario_name = scenario_name.split('_')[0]

            if 'ConstructionObstacle' in scenario_name:
                relevant_objects = [x for x in static_objects if x['type_id'] == 'static.prop.trafficwarning' 
                                                                and x['distance'] < 40 
                                                                and x['position'][0] > 0.0 and is_vehicle_in_camera(self.CAMERA_FRONT, x)]
                
            elif 'VehicleOpensDoorTwoWays' in scenario_name:

                relevant_objects = [v for v in vehicles_by_id.values() if v['type_id'] == 'vehicle.mercedes.coupe_2020' 
                                    and v['position'][0] > 0
                                    and v['position'][1] > 0.5 
                                    and (float(v['distance']) < 40 and v['speed'] < 0.1) and is_vehicle_in_camera(self.CAMERA_FRONT, v)]
                
            elif 'InvadingTurn' in scenario_name:
                relevant_objects = list(filter(lambda x: x['type_id'] == 'static.prop.constructioncone' \
                                            and x['position'][0] >= 1.5 \
                                            and x['distance'] <= 40 and is_vehicle_in_camera(self.CAMERA_FRONT, x), static_objects))
            
            elif 'ParkingExit' in scenario_name:
                if ego_data['lane_type_str'] == 'Parking':
                    relevant_objects = [x for x in vehicles_by_id.values() if x['lane_type_str']=='Parking' 
                                        and x['position'][0]>0]

            elif 'DynamicObjectCrossing' in scenario_name or 'ParkingCrossingPedestrian' in scenario_name or \
                'PedestrianCrossing' in scenario_name or 'VehicleTurningRoutePedestrian' in scenario_name:
                # in definition, DynamicObjectCrossing's obstacle might be a bicycle
                # but it does not exist in b2d
                # in PedestrianCrossing, there are 3 pedestrians
                # but since relevant_objects just saves the neareset one
                # we only reserve 1 nearest pedestrian, but in description, we describe them as 3
                relevant_objects = [x for x in pedestrians if 0.0 < x['position'][0] < 30.0
                                    and abs(x['speed']) > 1.0
                                    and -1.75 < x['position'][1] < 3.5] # carla lane width is 3.5m
                # print(f"[debug] relevant_objects = {relevant_objects}") # debug
            
            elif 'VehicleTurningRoute' == scenario_name:
                # bicycle length is roughly 1.6m
                relevant_objects = [x for x in vehicles_by_id.values() if 0.0 < x['position'][0] < 30.0
                                    and abs(x['speed']) > 1.0
                                    and -2.75 < x['position'][1] < 4.5
                                    and 'bicycle' == x['base_type']] # carla lane width is 3.5m

            elif 'OppositeVehicleRunningRedLight' in scenario_name or 'OppositeVehicleTakingPriority' in scenario_name:
                # including OppositeVehicleTakingPriority and OppositeVehicleRunningRedLight
                # left -> right and right -> left all exists
                relevant_objects = [x for x in vehicles_by_id.values() if 0.0 < x['position'][0] < 40.0
                                    and abs(x['speed']) > 3.0
                                    and ('police' in x['type_id'] or 'ambulance' in x['type_id'] or 'firetruck' in x['type_id'])
                                    and is_vehicle_in_junction(self.map, 
                                    carla.Location(x=x['location'][0], y=x['location'][1], z=x['location'][2]))]
                
            if self.current_measurement_index <= 5:
                # this is when background activity spawns vehicles around ego
                # speed of those vehicles can be wrong
                # so negelect
                relevant_objects = []

            # important object descriptions
            if relevant_objects:
                relevant_objects.sort(key=lambda x: x['distance'])
                relevant_obj = relevant_objects[0]

                # # Determine the rough position of the vehicle relative to the ego (front, front-left, front-right)
                if -2 <= relevant_obj['position'][1] <= 2:
                    rough_pos_str = 'to the front of the ego vehicle'
                elif relevant_obj['position'][1] > 2:
                    rough_pos_str = 'to the front right of the ego vehicle'
                else: #vehicle['position'][1] < -2
                    rough_pos_str = 'to the front left of the ego vehicle'

                def del_object_in_key_info(key_dict, obj_list):
                    # avoid repeat
                    for obj in obj_list:
                        obj_id = obj['id']
                        keys_to_remove = [k for k, v in key_dict.items() if v.get('id') == obj_id]
                        for key in keys_to_remove:
                            del key_dict[key]

                # Determine the type of vehicle based on its type_id
                if 'ConstructionObstacle' in scenario_name:
                    important_object_str = f'the construction warning {rough_pos_str}'
                    category = "Traffic element"
                    visual_description = "construction warning"
                    del_object_in_key_info(key_object_infos, [relevant_obj])
                elif 'InvadingTurn' in scenario_name:
                    multiple_cones = len(relevant_objects) > 1
                        
                    plural = 's' if multiple_cones else ''
                    important_object_str = f'the construction cone{plural} {rough_pos_str}'
                    category = "Traffic element"
                    visual_description = "construction cone"
                    del_object_in_key_info(key_object_infos, relevant_objects)
                elif 'VehicleOpensDoorTwoWays' in scenario_name or 'ParkingExit' in scenario_name or \
                    'OppositeVehicleTakingPriority' in scenario_name or 'OppositeVehicleRunningRedLight' in scenario_name:
                    # Determine the color of the vehicle
                    # color_str = relevant_obj["color_name"] + ' ' if relevant_obj.get("color_name") is not None \
                    #                                             and relevant_obj["color_name"] != 'None' else ''
                    if relevant_obj.get('color') is not None:
                        color_str = rgb_to_color_name(relevant_obj['color']) + ' '
                        if relevant_obj['color'] == [0, 28, 0] or relevant_obj['color'] == [12, 42, 12]:
                            color_str = 'dark green '
                        elif relevant_obj['color'] == [211, 142, 0]:
                            color_str = 'yellow '
                        elif relevant_obj['color'] == [145, 255, 181]:
                            color_str = 'blue '
                        elif relevant_obj['color'] == [215, 88, 0]:
                            color_str = 'orange '

                    category = "Vehicle"
                    visual_description = f"{color_str}{relevant_obj['base_type']}"

                    if 'VehicleOpensDoorTwoWays' in scenario_name and relevant_obj['distance'] <= 22.0:
                        # vehicle opens door at roughly 22m away
                        important_object_str = f"the {color_str}{relevant_obj['base_type']} with the open doors {rough_pos_str}"
                    elif 'OppositeVehicleTakingPriority' in scenario_name or 'OppositeVehicleRunningRedLight' in scenario_name:
                        if 'police' in relevant_obj['type_id']:
                            important_object_str = f'the {color_str}police car taking priority {rough_pos_str}'
                        elif 'ambulance' in relevant_obj['type_id']:
                            important_object_str = f'the {color_str}ambulance taking priority {rough_pos_str}'
                        elif 'firetruck' in relevant_obj['type_id']:
                            important_object_str = f'the {color_str}firetruck taking priority {rough_pos_str}'
                        else:
                            important_object_str = f'the {color_str}vehicle taking priority {rough_pos_str}'
                    else:
                        important_object_str = f"the {color_str}{relevant_obj['base_type']} parking {rough_pos_str}"
                    del_object_in_key_info(key_object_infos, [relevant_obj])
                    old_str, _, _ = get_vehicle_str(relevant_obj)
                    while old_str in important_objects:
                        important_objects.remove(old_str)
                elif 'DynamicObjectCrossing' in scenario_name or 'ParkingCrossingPedestrian' in scenario_name or 'VehicleTurningRoutePedestrian' in scenario_name:
                    category = "Pedestrian"
                    visual_description = "walking pedestrian"
                    important_object_str = f'the pedestrian crossing the road {rough_pos_str}'
                    del_object_in_key_info(key_object_infos, [relevant_obj])
                    old_str = get_pedestrian_str(relevant_obj)
                    while old_str in important_objects:
                        important_objects.remove(old_str)
                elif 'PedestrianCrossing' in scenario_name:
                    category = "Pedestrian"
                    visual_description = "3 pedestrians"
                    important_object_str = f'3 pedestrians crossing the road {rough_pos_str}'
                    # avoid repeat
                    keys_to_remove = [k for k, v in key_object_infos.items() if v.get('category') == 'Pedestrian']
                    for p in pedestrians:
                        old_str = get_pedestrian_str(p)
                        while old_str in important_objects:
                            important_objects.remove(old_str)
                    for key in keys_to_remove:
                        del key_object_infos[key]
                elif 'VehicleTurningRoute' == scenario_name:
                    category = "Vehicle"
                    visual_description = 'bicycle'

                    if relevant_obj.get('color') is not None:
                        color_str = rgb_to_color_name(relevant_obj['color']) + ' '
                        if relevant_obj['color'] == [0, 28, 0] or relevant_obj['color'] == [12, 42, 12]:
                            color_str = 'dark green '
                        elif relevant_obj['color'] == [211, 142, 0]:
                            color_str = 'yellow '
                        elif relevant_obj['color'] == [145, 255, 181]:
                            color_str = 'blue '
                        elif relevant_obj['color'] == [215, 88, 0]:
                            color_str = 'orange '

                    important_object_str = f'the {color_str}bicycle crossing the road {rough_pos_str}'
                    del_object_in_key_info(key_object_infos, [relevant_obj])
                    old_str = get_bicycle_str(relevant_obj)
                    for i in range(len(important_objects) - 1, -1, -1):
                        if old_str in important_objects[i]:
                            del important_objects[i]

                important_objects.append(important_object_str)

                if scenario_name in ['ConstructionObstacle', 'ConstructionObstacleTwoWays', 'InvadingTurn', 
                                     'ParkingExit', 'VehicleOpensDoorTwoWays',
                                     'DynamicObjectCrossing', 'ParkingCrossingPedestrian', 'PedestrianCrossing',
                                     'VehicleTurningRoutePedestrian', 'VehicleTurningRoute',
                                     'OppositeVehicleTakingPriority', 'OppositeVehicleRunningRedLight'
                                     ]:
                    projected_points, projected_points_meters = project_all_corners(relevant_obj, self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)
                    # Generate a unique key and value for the vehicle object
                    key, value = self.generate_object_key_value(
                        id=relevant_obj['id'],
                        category = category,
                        visual_description = visual_description,
                        object_count = len(key_object_infos),
                        projected_points = projected_points,
                        projected_points_meters=projected_points_meters
                    )
                    key_object_infos[key] = value
                    object_tags = [key]

            # print(f"[debug] [2] relevant_objects = {relevant_objects}") # debug
            question = "Does the ego vehicle need to change lanes or deviate from the lane center due to an "\
                        "upcoming obstruction?"
            answer = "No, the ego vehicle can stay on its current lane."

            question2 = 'Is there an obstacle on the current road?'
            answer2 = 'No, there is no obstacle on the current route.'
            
            if self.current_measurement_index > 5 and \
                scenario_name in ['Accident', 'AccidentTwoWays', 'ConstructionObstacle', 'ConstructionObstacleTwoWays',
                                    'InvadingTurn', 'HazardAtSideLane', 'HazardAtSideLaneTwoWays', 'ParkedObstacle',
                                    'ParkedObstacleTwoWays', 'VehicleOpensDoorTwoWays', 
                                    'DynamicObjectCrossing', 'ParkingCrossingPedestrian', 'PedestrianCrossing', 'VehicleTurningRoutePedestrian', 'VehicleTurningRoute',
                                    'OppositeVehicleTakingPriority', 'OppositeVehicleRunningRedLight']:
                
                # print(f"[debug] [3] relevant_objects = {relevant_objects}") # debug
                obstacle = {'Accident': 'accident',
                            'AccidentTwoWays': 'accident', 
                            'ConstructionObstacle': 'construction warning', 
                            'ConstructionObstacleTwoWays': 'construction warning', 
                            'InvadingTurn': 'invading vehicles on the opposite lane', 
                            'HazardAtSideLane': 'two bicycles',
                            'HazardAtSideLaneTwoWays': 'two bicycles',
                            'ParkedObstacle': 'parked vehicle', 
                            'DynamicObjectCrossing': 'walking pedestrian',
                            'ParkingCrossingPedestrian': 'walking pedestrian',
                            'VehicleTurningRoutePedestrian': 'walking pedestrian',
                            'PedestrianCrossing': '3 walking pedestrians',
                            'VehicleTurningRoute': 'bicycle',
                            'OppositeVehicleTakingPriority': 'running vehicle',
                            'OppositeVehicleRunningRedLight': 'running vehicle',
                            'ParkedObstacleTwoWays': 'parked vehicle', 
                            'VehicleOpensDoorTwoWays': 'vehicle with the opened door'}[scenario_name]
                
                changed_route = measurements["changed_route"]
                
                if 'HazardAtSideLane' in scenario_name:
                    relevant_objects = [v for v in vehicles_by_id.values() if v['base_type'] == 'bicycle' 
                                                                        and self.should_consider_vehicle(v) 
                                                                        and float(v['distance']) < 40
                                                                        and v['lane_relative_to_ego'] == 0]
                    if len(relevant_objects) == 1:
                        obstacle = 'bicycle'
                elif scenario_name not in ['VehicleOpensDoorTwoWays', 'ConstructionObstacle', 
                                           'ConstructionObstacleTwoWays', 'InvadingTurn',
                                           'DynamicObjectCrossing', 'ParkingCrossingPedestrian', 'PedestrianCrossing', 'VehicleTurningRoutePedestrian', 'VehicleTurningRoute',
                                           'OppositeVehicleTakingPriority', 'OppositeVehicleRunningRedLight']:
                    relevant_objects = [v for v in vehicles_by_id.values() if self.should_consider_vehicle(v) 
                                                                        and abs(v['speed']) <= 0.0001
                                                                        and float(v['distance']) < 40]
                    if 'Accident' in scenario_name:
                        relevant_objects = [v for v in relevant_objects if 'police' in v['type_id']]
                
                relevant_objects.sort(key = lambda x: float(x['distance']))

                if relevant_objects:
                    if 'Accident' in scenario_name: 
                        object_tags = [k for k, v in key_object_infos.items() if 'police' in v['Visual_description']]      
                    elif 'HazardAtSideLane' in scenario_name: 
                        object_tags = [k for k, v in key_object_infos.items() if 'bicycle' in v['Visual_description']]    
                    elif 'ParkedObstacle' in scenario_name:
                        assert len(relevant_objects), relevant_objects

                        relevant_obj = relevant_objects[0]
                        projected_points, projected_points_meters = project_all_corners(relevant_obj, 
                                                                                        self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)
                        if projected_points is not None and len(projected_points) > 0:
                            two_d_box = self.generate_2d_box_from_projected_points(projected_points)
                            keys = [k for k, v in key_object_infos.items() if two_d_box==v['2d_bbox']]

                            assert len(keys)==1, keys

                            object_tags = keys
                        
                    elif 'VehicleOpensDoorTwoWays' in scenario_name:
                        assert len(relevant_objects), relevant_objects

                        relevant_obj = relevant_objects[0]
                        projected_points, projected_points_meters = project_all_corners(relevant_obj, 
                                                                                        self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)
                        if projected_points is not None and len(projected_points) > 0:
                            two_d_box = self.generate_2d_box_from_projected_points(projected_points)
                            keys = [k for k, v in key_object_infos.items() if two_d_box==v['2d_bbox']]

                            object_tags = keys
                    
                    changed_route = relevant_objects[0]['lane_id'] != ego_data['lane_id']
                    if changed_route and relevant_obj and\
                        scenario_name not in ['DynamicObjectCrossing', 'ParkingCrossingPedestrian', 'PedestrianCrossing', 'VehicleTurningRoutePedestrian', 'VehicleTurningRoute',
                                           'OppositeVehicleTakingPriority', 'OppositeVehicleRunningRedLight', 'VehicleOpensDoorTwoWays']:
                        if 'InvadingTurn' == scenario_name:
                            answer = f"The ego vehicle has already shifted to the side to avoid {obstacle}."
                        else:
                            # print(f'[debug] path = {self.current_measurement_path}')
                            # print("[debug] TODO: line 1463: lane_change related, has to be implemented.")
                            # route_start = np.array(measurements['route_original'][0])
                            # route_end = np.array(measurements['route_original'][1])
                            
                            # route_vector = route_end - route_start
                            # ego_to_route_start = route_start  # Assuming ego vehicle is at [0, 0]

                            # # Calculate the projection of ego_to_route_start onto route_vector
                            # projection_length = np.dot(route_vector, ego_to_route_start) / np.linalg.norm(route_vector)
                            
                            # # Calculate lateral distance using Pythagorean theorem
                            # distance_to_route_start = np.linalg.norm(ego_to_route_start)
                            # lateral_distance = np.sqrt(distance_to_route_start**2 - projection_length**2)

                            lateral_distance = 3.5

                            # usually roads in carla are 3.5 wide
                            changing_or_has_changed = "has already changed" if lateral_distance > 3.5/2. else "is "\
                                                                                                            "changing"
                            answer = f"The ego vehicle {changing_or_has_changed} to another lane to "\
                                        f"circumvent the {obstacle}."
                    else:
                        keywords = ['Accident', 'ConstructionObstacle', 'HazardAtSideLane', 'ParkedObstacle']
                        if any(keyword in scenario_name for keyword in keywords):
                            if ego_data['lane_change'] == 1:
                                side = 'the right lane'
                            elif ego_data['lane_change'] == 2:
                                side = 'the left lane'
                            elif ego_data['lane_change'] == 3:
                                side = 'either side'
                            
                            if ego_data['lane_change'] in [1, 2, 3]:
                                answer = f"The ego vehicle must change to {side} to circumvent the {obstacle}."

                                if not obstacle.startswith('two'):
                                    obstacle2 = 'an '+obstacle if obstacle[0] in ['a', 'e', 'i', 'o', 'u'] else 'a '+obstacle
                                else:
                                    obstacle2 = obstacle
                                obstacle2 = 'are '+obstacle2 if obstacle2.startswith('two') else 'is '+obstacle2
                                answer2 = f'Yes, there {obstacle2} on the current road.'
                            elif ego_data['lane_change'] in [0]:
                                if 'TwoWays' in scenario_name:
                                    answer = f"The ego vehicle must change to the opposite lane to circumvent the {obstacle}."
                                
                                    if not obstacle.startswith('two'):
                                        obstacle2 = 'an '+obstacle if obstacle[0] in ['a', 'e', 'i', 'o', 'u'] else 'a '+obstacle
                                    else:
                                        obstacle2 = obstacle
                                    obstacle2 = 'are '+obstacle2 if obstacle2.startswith('two') else 'is '+obstacle2
                                    answer2 = f'Yes, there {obstacle2} on the current road.'
                                else:
                                    answer = f"No, the ego vehicle can stay on its current lane. But the ego vehicle must stop because there's no way to circumvent the {obstacle}."
                                
                                    obstacle2 = 'an '+obstacle if obstacle[0] in ['a', 'e', 'i', 'o', 'u'] else 'a '+obstacle
                                    obstacle2 = 'are '+obstacle2 if obstacle2.startswith('two') else 'is '+obstacle2
                                    answer2 = f'Yes, there {obstacle2} on the current road.'

                        elif 'VehicleOpensDoor' in scenario_name:
                                if relevant_obj['distance'] > 22:
                                    answer = "No, the ego vehicle can stay on its current lane."
                                    answer2 = 'No, there is no obstacle on the current route.'
                                else:
                                    lane_str = 'opposite lane' if ego_data['lane_change'] in [0] else 'left_lane'
                                    answer = f"The ego vehicle must change to the {lane_str} to circumvent the {obstacle}."

                                    if not obstacle.startswith('two'):
                                        obstacle2 = 'an '+obstacle if obstacle[0] in ['a', 'e', 'i', 'o', 'u'] else 'a '+obstacle
                                    else:
                                        obstacle2 = obstacle
                                    obstacle2 = 'are '+obstacle2 if obstacle2.startswith('two') else 'is '+obstacle2
                                    answer2 = f'Yes, there {obstacle2} on the current road.'
                        elif scenario_name == 'InvadingTurn':
                            answer = f"The ego vehicle must shift slightly to the right side to avoid {obstacle}."

                            answer2 = f'Yes, there might be invading vehicles from the opposite lane on the current road.'
                        # 'AccidentTwoWays', 'ConstructionObstacleTwoWays', 'HazardAtSideLaneTwoWays', 
                        # 'ParkedObstacleTwoWays', 'VehicleOpensDoorTwoWays'
                        elif 'DynamicObjectCrossing' in scenario_name or 'ParkingCrossingPedestrian' in scenario_name or 'VehicleTurningRoutePedestrian' in scenario_name:
                            answer = f"No, the ego vehicle can stay on its current lane. But the ego vehicle must stop because there's a pedestrian crossing the road."
                            answer2 = f'Yes, there is a pedestrian crossing the road.'
                        elif 'PedestrianCrossing' in scenario_name:
                            answer = f"No, the ego vehicle can stay on its current lane. But the ego vehicle must stop because there are 3 pedestrians crossing the road."
                            answer2 = f'Yes, there are 3 pedestrians crossing the road.'
                        elif scenario_name == 'VehicleTurningRoute':
                            answer = f"No, the ego vehicle can stay on its current lane. But the ego vehicle must stop because there's a bicycle crossing the road."
                            answer2 = f'Yes, there is a bicycle crossing the road.'
                        elif 'OppositeVehicleTakingPriority' in scenario_name or 'OppositeVehicleRunningRedLight' in scenario_name:
                            obstacle2 = 'a vehicle'
                            if 'police' in relevant_obj['type_id']:
                                obstacle2 = 'a police car'
                            elif 'ambulance' in relevant_obj['type_id']:
                                obstacle2 = 'an ambulance'
                            elif 'firetruck' in relevant_obj['type_id']:
                                obstacle2 = 'a firetruck'
                            answer = f"No, the ego vehicle can stay on its current lane. But the ego vehicle must stop because there's {obstacle2} taking priority."
                            answer2 = f'Yes, there is {obstacle2} taking priority.'
                        else: 
                            answer = f"The ego vehicle must change to the opposite lane to circumvent the {obstacle}."
                            
                            obstacle2 = 'an '+obstacle if obstacle[0] in ['a', 'e', 'i', 'o', 'u'] else 'a '+obstacle
                            obstacle2 = 'are '+obstacle2 if obstacle2.startswith('two') else 'is '+obstacle2
                            answer2 = f'Yes, there {obstacle2} on the current road.'
        
                # if changed_route \
                #         and answer == "No, the ego vehicle can stay on its current lane." \
                #         and scenario_name != 'ParkingExit':
                #     answer = "The ego vehicle must change back to the original lane after passing the obstruction."

            elif 'ParkingExit' in scenario_name:
                if ego_data['lane_type_str'] == 'Parking':
                    answer = "The ego vehicle must change to the left to exit the parking lot."

            # print(f"[debug] important_objects = {important_objects}") # debug
            self.add_qas_questions(qa_list=qas_conversation_ego,
                                   chain=3,
                                   layer=8,
                                   qa_type='planning',
                                   connection_up=[(6, 0)] ,
                                   connection_down=[(3, 9)],
                                   question=question,
                                   answer=answer,
                                   object_tags=object_tags)
            
            self.add_qas_questions(qa_list=qas_conversation_ego,
                                   chain=3,
                                   layer=9,
                                   qa_type='perception',
                                   connection_up=[(3,8)],
                                   connection_down=-1,
                                   question=question2,
                                   answer=answer2,
                                   object_tags=object_tags)

        qas_conversation_ego = []
        
        determine_whether_ego_needs_to_change_lanes_due_to_obstruction(qas_conversation_ego,
                                                                       scenario_name,
                                                                       vehicles_by_id,
                                                                       static_objects,
                                                                       measurements,
                                                                       ego_data, important_objects, key_object_infos)

        determine_braking_requirement(qas_conversation_ego,
                                      pedestrians,
                                      measurements,
                                      scene_data,
                                      vehicles_by_id,
                                      ego_data,
                                      scenario_name,
                                      traffic_light_info,
                                      stop_sign_info,
                                      static_objects)

        ego_speed = ego_vehicle_data['speed']

        actor_stop = [traffic_light_info, stop_sign_info]
        object_tags = [traffic_light_object_tags, stop_sign_object_tags]
        actor_names = ['traffic light', 'stop sign']
        for actor, actor_name, tags in zip(actor_stop, actor_names, object_tags):
            determine_ego_action_based_on_actor(actor,
                                                actor_name,
                                                ego_speed,
                                                ego_data,
                                                qas_conversation_ego, 
                                                stop_signs,
                                                tags)

        add_speed_limit_question(qas_conversation_ego,
                                 measurements)

        return qas_conversation_ego, important_objects, key_object_infos

    def generate_vehicle_information(self, other_vehicles, ego_vehicle, important_objects, key_object_infos, 
                                            scene_data, vehicles_by_id, current_measurement, scenario):
        """
        Generates information and question-answer pairs for vehicles in the scene.

        Answers the questions:
        - Where on the road is the vehicle located?
        - Where is the vehicle going?
        - What is the moving status of the vehicle?
        - The ego vehicle {command_str}. Is {vehicle_location_description} potentially crossing the 
            path of the ego vehicle?

        Args:
            other_vehicles (list): List of dictionaries containing information about other vehicles in the scene.
            ego_vehicle (dict): Dictionary containing information about the ego vehicle.
            important_objects (list): List to store important objects in the scene.
            key_object_infos (dict): Dictionary to store information about objects in the scene.
            num_lanes_ego (int): Number of lanes for the ego vehicle.
            vehicles_by_id (dict): Dictionary mapping vehicle IDs to vehicle information.
            current_measurement (dict): Dictionary containing current measurement data.
            scenario (str): The current scenario.

        Returns:
            qas_conversation_vehicle (list): List of question-answer pairs related to vehicles.
            important_objects (list): Updated list of important objects in the scene.
            key_object_infos (dict): Updated dictionary of object information.
        """

        def determine_path_crossing(current_measurement, ego_vehicle, other_vehicle_location_description, 
                            other_vehicle, vehicles_by_id, other_vehicle_description, scenario, 
                            ego_distance_to_junction, other_vehicle_points_towards_ego, 
                            other_vehicle_heading_angle_deg, pointing_towards_junction, is_ego_on_highway, 
                            is_ego_in_accel_lane, is_other_veh_in_accel_lane, qas_conversation_vehicle, object_tags):
            """
            Answers: "The ego vehicle {command_str}. Is {vehicle_location_description} potentially crossing the 
                path of the ego vehicle?"
            
            Args:
                current_measurement (dict): Current measurement data.
                ego_vehicle (dict): Information about the ego vehicle.
                other_vehicle_location_description (str): Description of the other vehicle's location.
                other_vehicle (dict): Information about the other vehicle.
                vehicles_by_id (dict): Dictionary of vehicles by their ID.
                other_vehicle_description (str): Description of the other vehicle.
                scenario (str): The current scenario.
                ego_distance_to_junction (float): Distance of the ego vehicle to the next junction.
                other_vehicle_points_towards_ego (bool): True if the other vehicle is pointing towards the ego vehicle.
                other_vehicle_heading_angle_deg (float): Heading angle of the other vehicle in degrees.
                pointing_towards_junction (bool): True if the other vehicle is pointing towards the junction.
                is_ego_on_highway (bool): True if the ego vehicle is on a highway.
                is_ego_in_accel_lane (bool): True if the ego vehicle is in an acceleration lane.
                is_other_veh_in_accel_lane (bool): True if the other vehicle is in an acceleration lane.
                qas_conversation_vehicle (list): List of question-answer pairs for the vehicle.

            Returns:
                None
            """            

            scenario = scenario.split('_')[0]
            # print(f"[debug] scenario = {scenario}")
            # Map command integers to their corresponding string descriptions
            command_int = current_measurement['next_command']
            if ego_vehicle['is_in_junction']:
                command_int = current_measurement['command']
            x = (current_measurement['target_point'][0] - current_measurement['x'])**2
            y = (current_measurement['target_point'][1] - current_measurement['y'])**2
            if ego_vehicle['is_in_junction']:
                x = (current_measurement['x_command_near'] - current_measurement['x'])**2
                y = (current_measurement['y_command_near'] - current_measurement['y'])**2
            command_distance = np.sqrt(x + y)
            if command_distance > 25:
                command_int = 4

            # the next intersection -> the intersection
            command_map = {
                1: 'turns left at the intersection',
                2: 'turns right at the intersection',
                3: 'drives straight at the intersection',
                4: 'follows the road',
                5: f'does a lane change to the left in {int(command_distance)} m',
                6: f'does a lane change to the right in {int(command_distance)} m',
            }
            command_str = command_map[command_int]

            # Update command string if the ego vehicle is already in a junction
            if ego_vehicle['is_in_junction']:
                command_str = command_str.replace('turns', 'continues turning').replace('drives', 'continues driving')
                command_str = command_str.replace('next intersection', 'current intersection')

            question = f"The ego vehicle {command_str}. Is {other_vehicle_location_description} potentially " \
                            "crossing the path of the ego vehicle?"
            actor_in_front = None
            same_future_road = any([x==y for x in other_vehicle['next_road_ids'] for y in ego_vehicle['next_road_ids']])

            # Find the actor in front of the ego vehicle
            if ego_vehicle['hazard_detected_20']:
                # If the car in front is in an intersection this might not work perfectly, however, given the labels
                # it can't be improved
                if ego_vehicle['hazard_detected_10']:
                    actor_in_front = vehicles_by_id[ego_vehicle['affects_ego_10']]
                elif ego_vehicle['hazard_detected_15']:
                    actor_in_front = vehicles_by_id[ego_vehicle['affects_ego_15']]
                else:
                    actor_in_front = vehicles_by_id[ego_vehicle['affects_ego_20']]

            # Check if the other vehicle is right in front of the ego vehicle
            if actor_in_front is not None and actor_in_front['id'] == other_vehicle['id'] and \
                                                                            ego_vehicle['lane_type_str'] != 'Parking':
                # if vehicle is in front of ego vehicle on the same lane
                answer = f"Yes, the {other_vehicle_description} is right to the front of the ego vehicle, so the " +\
                                                            "ego vehicle should pay attention to not crash into it."
            
            # Check for the BlockedIntersection scenario
            elif scenario == 'BlockedIntersection' and \
                                        other_vehicle['distance'] < 40 and not other_vehicle['same_direction_as_ego']:
                answer = f"Yes, the {other_vehicle_description} is behind the intersection on the road the ego " +\
                                    "vehicle will enter, so the ego vehicle should pay attention to not crash into it."
            # Check if the other vehicle is on the same road as the ego vehicle's next or next-next road
            elif (other_vehicle['road_id'] in ego_vehicle['next_road_ids'] or \
                other_vehicle['road_id'] in ego_vehicle['next_next_road_ids_ego'] or same_future_road) and \
                    other_vehicle['distance']< 25 and not other_vehicle['same_direction_as_ego']:
                # handles vehicles that are in front but in an intersection
                if other_vehicle['is_in_junction']:
                    answer = f"Yes, the {other_vehicle_description} is inside the upcoming junction on the same " +\
                                "road as the ego vehicle, so the ego vehicle should pay attention to not crash into it."
                else:
                    answer = f"Yes, the {other_vehicle_description} is behind the intersection on the road the " +\
                                "ego vehicle will enter, so the ego vehicle should pay attention to not crash into it."

            # Check if the other vehicle is in a junction and pointing towards the ego vehicle
            elif other_vehicle['is_in_junction'] and ego_distance_to_junction is not None and \
                                                    ego_distance_to_junction < 40 and other_vehicle_points_towards_ego:
                # intersection and vehicle points towards ego
                answer = f"Yes, the {other_vehicle_description} is crossing the path of the ego vehicle."
                if (other_vehicle_heading_angle_deg > 160 \
                            and other_vehicle_heading_angle_deg < 200) \
                            and command_int == 3:
                    answer = f"If the other vehicle keeps going straight, the routes will not cross."

            # Check if the other vehicle is cutting into the ego vehicle's lane
            elif 'vehicle_cuts_in' in other_vehicle and other_vehicle['vehicle_cuts_in']:
                answer = f"Yes, the routes will cross since the {other_vehicle_description} is cutting into the " +\
                                                                                            "lane of the ego vehicle."
            
            # Check if the other vehicle is pointing towards the junction and the ego vehicle is close to the junction
            elif pointing_towards_junction and ego_distance_to_junction is not None and ego_distance_to_junction < 40:
                answer = f"Yes, the {other_vehicle_description} might cross the path of the ego vehicle, depending " +\
                                                                            "on which way the vehicle is going to turn."
                
            # Check if the ego vehicle is exiting a parking spot and the other vehicle is on the lane the ego vehicle 
            # wants to enter
            elif ego_vehicle['lane_type_str'] == 'Parking' and \
                                        other_vehicle['lane_relative_to_ego'] == -1 and other_vehicle['distance'] < 10:
                answer = f"Yes, because the ego vehicle wants to exit the parking spot and the " +\
                                        f"{other_vehicle_description} is on the lane the ego vehicle wants to enter."
            else:
                answer = f"No, the {other_vehicle_description} is not crossing paths with the ego vehicle."

            # Check for lane changes of the ego vehicle
            if other_vehicle['lane_relative_to_ego'] == -1 and command_int == 5:
                answer = f"Yes, the {other_vehicle_description} is crossing paths with the ego vehicle because the " +\
                        f"ego vehicle does a lane change to the left onto the lane of the {other_vehicle_description}."
                   
            elif other_vehicle['lane_relative_to_ego'] == 1 and command_int == 6:
                answer = f"Yes, the {other_vehicle_description} is crossing paths with the ego vehicle because the " +\
                        f"ego vehicle does a lane change to the right onto the lane of the {other_vehicle_description}."\
            
            elif other_vehicle['lane_relative_to_ego'] == 0 and \
                other_vehicle['base_type'] == 'bicycle' and 0.0 < other_vehicle['position'][0] < 30.0 and is_vehicle_in_camera(self.CAMERA_FRONT, other_vehicle):
                answer = f"Yes, the {other_vehicle_description} is potentially crossing paths with the ego vehicle because the " +\
                        f"{other_vehicle_description} doesn't match the speed of ego vehicle."\
                
            # Check for scenarios involving acceleration lanes
            elif is_ego_on_highway and is_ego_in_accel_lane:
                # special case if ego is still on acceleration lane and the lane of the other vehicle is not considered
                # as the same road -> lane_relative_to_ego is None
                answer = f"The routes of the ego vehicle and the {other_vehicle_description} might cross as " +\
                    f"the {other_vehicle_description} is on the highway and the ego vehicle is on the acceleration " +\
                                                                                    f"lane about to enter the highway."
            elif is_ego_on_highway and is_other_veh_in_accel_lane:
                answer = f"The routes of the ego vehicle and the {other_vehicle_description} might cross as " +\
                    f"the {other_vehicle_description} is on the acceleration lane about to enter the highway, " +\
                    f"potentially cutting into the lane of the ego vehicle."

            # Check for the CrossingBicycleFlow scenario
            if other_vehicle['base_type'] == 'bicycle' \
                        and scenario == "CrossingBicycleFlow":
                # CrossingBicycleFlow scenario
                if command_int == 4:
                    # Special case if the ego vehicle is not close enough to the junction and the command is still 
                    # follow the road
                    command_str = 'turns at the next intersection'
                answer = f"Yes, the bike lane on which the {other_vehicle_description} is currently riding on is " +\
                                                f"crossing paths with the ego vehicle if the ego vehicle {command_str}."
            elif other_vehicle['base_type'] == 'bicycle' \
                        and scenario == "VehicleTurningRoute":
                answer = f"Yes, the {other_vehicle_description} will cross paths with the ego vehicle if " \
                            f"the ego vehicle {command_str}."

            elif scenario == "HighwayCutIn" \
                        and other_vehicle['lane_relative_to_ego'] == 1:
                answer = f"Yes, the routes of the ego vehicle and the {other_vehicle_description} might cross as " +\
                    f"the {other_vehicle_description} is on the acceleration lane, potentially cutting into the " +\
                    f"lane of the ego vehicle."

            # Add the question-answer pair to the conversation
            self.add_qas_questions(qa_list=qas_conversation_vehicle,
                                    chain=4,
                                    layer=3,
                                    qa_type='planning',
                                    connection_up=[(6,0)],
                                    connection_down=[(4,0), (4,1), (4,2), (3,1)],
                                    question=question,
                                    answer=answer,
                                    object_id=other_vehicle['id'],
                                    object_tags=object_tags)

        def determine_vehicle_motion_status(other_vehicle_location_description, other_vehicle, 
                                           other_vehicle_description, qas_conversation_vehicle, object_tags):
            """
            Answers: "What is the moving status of {other_vehicle_location_description}?".
            
            Args:
                other_vehicle_location_description (str): Description of the other vehicle's location.
                other_vehicle (dict): Information about the other vehicle.
                other_vehicle_description (str): Description of the other vehicle.
                qas_conversation_vehicle (list): List of question-answer pairs for the vehicle.

            Returns:
                None
            """
            question = f"What is the moving status of {other_vehicle_location_description}?"

            # Determine motion status based on speed and vehicle type
            if other_vehicle['speed'] < 0.2:
                answer = f"The {other_vehicle_description} is not moving."
            elif other_vehicle['speed'] < 5:
                if other_vehicle['base_type'] == 'bicycle':
                    answer = f"The {other_vehicle_description} is moving slowly."
                else:
                    answer = f"The {other_vehicle_description} is driving slowly."
            else:
                if other_vehicle['base_type'] == 'bicycle':
                    answer = f"The {other_vehicle_description} is moving."
                else:
                    answer = f"The {other_vehicle_description} is driving."

            # Add the question-answer pair to the conversation
            self.add_qas_questions(qa_list=qas_conversation_vehicle,
                                    chain=4,
                                    layer=2,
                                    qa_type='prediction',
                                    connection_up=[(4,3)],
                                    connection_down=[(4,0)],
                                    question=question,
                                    answer=answer,
                                    object_id=other_vehicle['id'],
                                    object_tags=object_tags)

        def determine_vehicle_trajectory(other_vehicle_location_description, other_vehicle, other_vehicle_description,
                                                                                qas_conversation_vehicle, object_tags):
            """
            Answer: "Where is {other_vehicle_location_description} going?".

            Args:
                other_vehicle_location_description (str): Description of the other vehicle's location.
                other_vehicle (dict): Information about the other vehicle.
                other_vehicle_description (str): Description of the other vehicle.
                qas_conversation_vehicle (list): List of question-answer pairs for the vehicle.

            Returns:
                None
            """

            # TODO: use future to predict!
            question = f"Where is {other_vehicle_location_description} going?"
            answer = "TODO: we don't have steer, so should use future to predict other vehicle's trajetory"

            steer = other_vehicle['steer']

            # Determine trajectory based on steer angle
            if steer < -0.9:
                answer = f"The {other_vehicle_description} is turning left."
            elif steer < -0.2:
                answer = f"The {other_vehicle_description} is turning slightly left."
            elif steer > 0.9:
                answer = f"The {other_vehicle_description} is turning right."
            elif steer > 0.2:
                answer = f"The {other_vehicle_description} is turning slightly right."
            else:
                answer = f"The {other_vehicle_description} is going straight."

            # Check if the other vehicle is cutting into the ego vehicle's lane
            if 'vehicle_cuts_in' in other_vehicle:
                if other_vehicle['vehicle_cuts_in']:
                    answer = f"The {other_vehicle_description} is cutting into the lane of the ego vehicle."

            # Add the question-answer pair to the conversation
            self.add_qas_questions(qa_list=qas_conversation_vehicle,
                                    chain=4,
                                    layer=1,
                                    qa_type='prediction',
                                    connection_up=[(4,3)],
                                    connection_down=[(4,0)],
                                    question=question,
                                    answer=answer,
                                    object_id=other_vehicle['id'],
                                    object_tags=object_tags)

        def determine_other_vehicle_position(other_vehicle_location_description, other_vehicle, ego_vehicle,
                                        is_ego_on_highway, is_ego_in_accel_lane, is_ego_in_exit_lane,
                                        other_vehicle_description, is_ego_in_entry_lane, ego_about_to_exit_highway,
                                        scenario, qas_conversation_vehicle, object_tags):
            """
            Answers: "Where on the road is {other_vehicle_location_description} located?".

            Args:
                other_vehicle_location_description (str): Description of the other vehicle's location.
                other_vehicle (dict): Information about the other vehicle.
                ego_vehicle (dict): Information about the ego vehicle.
                is_ego_on_highway (bool): True if the ego vehicle is on a highway.
                is_ego_in_accel_lane (bool): True if the ego vehicle is in an acceleration lane.
                is_ego_in_exit_lane (bool): True if the ego vehicle is in an exit lane.
                other_vehicle_description (str): Description of the other vehicle.
                is_ego_in_entry_lane (bool): True if the ego vehicle is in an entry lane.
                ego_about_to_exit_highway (bool): True if the ego vehicle is about to exit the highway.
                scenario (str): The current scenario.
                qas_conversation_vehicle (list): List of question-answer pairs for the vehicle.

            Returns:
                pointing_towards_junction (bool or None): True if the other vehicle is pointing towards the junction, 
                    False if pointing away from the junction, or None if the direction is unknown.
            """

            scenario = scenario.split("_")[0]
            question = f"Where on the road is {other_vehicle_location_description} located?"
            answer = ''
            pos = other_vehicle['position']

            # Check if the other vehicle is on the same road as the ego vehicle
            same_road = other_vehicle['same_road_as_ego']

            # Check if the other vehicle is moving in the same direction as the ego vehicle
            same_direction = other_vehicle['same_direction_as_ego']
            pointing_towards_junction = None

            # Determine the other vehicle's orientation relative to the ego vehicle
            orientation_relative_to_ego = other_vehicle['yaw']
            orientation_relative_to_ego = orientation_relative_to_ego * 180 / np.pi
            
            # Categorize the orientation into 4 bins: leftwards, straight, rightwards, oncoming
            if -135 < orientation_relative_to_ego < -45:
                orientation_str = 'is pointing leftwards'
            elif 45 < orientation_relative_to_ego < 135:
                orientation_str = 'is pointing rightwards'
            elif 135 < orientation_relative_to_ego or orientation_relative_to_ego < -135:
                orientation_str = 'is pointing towards the ego vehicle'
            else:
                orientation_str = 'is pointing in the same direction as the ego vehicle'

            # Handle cases where the other vehicle is in a junction or on another entry of the junction
            if other_vehicle['is_in_junction'] and (other_vehicle['junction_id'] == ego_vehicle['next_junction_id'] or \
                        other_vehicle['junction_id'] == ego_vehicle['junction_id'] or \
                        (ego_vehicle['junction_id'] == -1 and ego_vehicle['next_junction_id'] == -1)):
                if is_ego_on_highway and (is_ego_in_accel_lane or is_ego_in_exit_lane):
                    # Handle cases where the ego vehicle is in the merging or exit area of the highway
                    if is_ego_in_accel_lane: 
                        lane_str='merging area'
                    if is_ego_in_exit_lane: 
                        lane_str='exit area'
                    if other_vehicle['same_road_as_ego'] and other_vehicle['same_direction_as_ego'] and \
                                                                            other_vehicle['lane_relative_to_ego'] == 0:
                        answer = f"The {other_vehicle_description} is in the {lane_str} of the highway in front " +\
                                                                                            f"of the ego vehicle."
                    elif other_vehicle['lane_id'] == -1:
                        answer = f"The {other_vehicle_description} is close to the {lane_str} but on the leftmost " +\
                                                                                                f"lane of the highway."
                    elif other_vehicle['lane_id'] == -2:
                        answer = f"The {other_vehicle_description} is close to the {lane_str} but on the second " +\
                                                                                f"lane from the left on the highway."
                    else:
                        answer = f"The {other_vehicle_description} is on the highway near the {lane_str}."
                elif is_ego_in_entry_lane:
                    answer = f"The {other_vehicle_description} is on the lane that leads to the highway."
                elif ego_about_to_exit_highway:
                    answer = f"The {other_vehicle_description} is on the exit lane of the highway."
                elif is_ego_on_highway:
                    answer = f"The {other_vehicle_description} is on the highway."
                elif other_vehicle['road_id'] not in ego_vehicle['next_road_ids']:
                    answer = f"The {other_vehicle_description} is inside the upcoming junction and {orientation_str}."
                else:
                    answer = f"The {other_vehicle_description} is inside the upcoming junction and {orientation_str}."

                # Handle the MergerIntoSlowTrafficV2 scenario
                if scenario == "MergerIntoSlowTrafficV2": # or scenario == "MergerIntoSlowTraffic":
                    if ego_vehicle['num_lanes_same_direction'] == 1 and other_vehicle['same_road_as_ego'] or \
                                    (ego_vehicle['num_lanes_same_direction'] - ego_vehicle['ego_lane_number']-1 == 0 and \
                                                    ego_vehicle['distance_to_junction']<25 and \
                                                    (other_vehicle['road_id'] in ego_vehicle['next_road_ids'] or \
                                                    other_vehicle['road_id'] == ego_vehicle['road_id'] or \
                                                    other_vehicle['road_id'] in ego_vehicle['next_next_road_ids_ego'])):
                        answer = f"The {other_vehicle_description} is on the exit lane of the highway."
                    elif ego_vehicle['num_lanes_same_direction'] == 1 and not other_vehicle['same_road_as_ego'] or \
                                        (ego_vehicle['num_lanes_same_direction'] > 1 and \
                                        (ego_vehicle['is_in_junction'] or ego_vehicle['distance_to_junction'] < 25)):
                        answer = f"The {other_vehicle_description} is on the highway near the exit area."
                    else:
                        answer = f"The {other_vehicle_description} is on the highway close to the merging area."

            # Handle cases where the other vehicle is not in a junction and not on the same road as the ego vehicle
            elif not other_vehicle['is_in_junction'] and (other_vehicle['road_id'] != ego_vehicle['road_id'] and \
                                                        other_vehicle['road_id'] not in ego_vehicle['next_road_ids']):
                # Determine if the other vehicle is pointing towards or away from the junction
                if ego_vehicle['junction_id'] == -1 or other_vehicle['junction_id'] == -1:
                    if pos[1] < -8 and orientation_relative_to_ego > 45 and orientation_relative_to_ego < 135:
                        to_or_away_junction = "is pointing towards the junction"
                        pointing_towards_junction = True
                    elif pos[1] > 8 and orientation_relative_to_ego < -45 and orientation_relative_to_ego > -135:
                        to_or_away_junction = "is pointing towards the junction"
                        pointing_towards_junction = True
                    elif pos[1] < -8 and orientation_relative_to_ego < -45 and orientation_relative_to_ego > -135:
                        to_or_away_junction = "is pointing away from the junction"
                        pointing_towards_junction = False
                    elif pos[1] > 8 and orientation_relative_to_ego > 45 and orientation_relative_to_ego < 135:
                        to_or_away_junction = "is pointing away from the junction"
                        pointing_towards_junction = False
                    elif pos[1] < 8 and pos[1] > -8 and orientation_relative_to_ego > 135 or \
                                                                                    orientation_relative_to_ego < -135:
                        to_or_away_junction = "is pointing towards the junction"
                        pointing_towards_junction = True
                    elif pos[1] < 8 and pos[1] > -8 and orientation_relative_to_ego < 45 and \
                                                                                    orientation_relative_to_ego > -45:
                        to_or_away_junction = "is pointing away from the junction"
                        pointing_towards_junction = False
                    else:
                        to_or_away_junction = "is pointing in an unknown direction"
                        pointing_towards_junction = False

                elif other_vehicle['next_junction_id'] == ego_vehicle['next_junction_id'] or \
                                                        other_vehicle['next_junction_id'] == ego_vehicle['junction_id']:
                    to_or_away_junction = "is pointing towards the junction"
                # Away from junction
                elif other_vehicle['next_junction_id'] != ego_vehicle['next_junction_id'] and \
                                                        other_vehicle['next_junction_id'] != ego_vehicle['junction_id']:
                    to_or_away_junction = "is pointing away from the junction"

                # Determine the direction of the other vehicle relative to the junction
                if pos[1] < -8:
                    direction_junction = "on the left side of the junction"
                elif pos[1] > 8:
                    direction_junction = "on the right side of the junction"
                elif not other_vehicle['same_road_as_ego']:
                    direction_junction = "on the opposite side of the junction"
                else:
                    raise ValueError(f"Unknown position of vehicle {other_vehicle['id']}.")

                # Add information if the other vehicle is on a bike lane
                bike_lane_str = ''
                if other_vehicle['lane_type_str'] == 'Biking':
                    bike_lane_str = ' on the bike lane'

                answer = f"The {other_vehicle_description} is {direction_junction}{bike_lane_str} and " +\
                                                                                            f"{to_or_away_junction}."

                # Handle cases where the ego vehicle is on the highway
                if is_ego_on_highway:
                    if other_vehicle['lane_id'] == -1:
                        answer = f"The {other_vehicle_description} is driving on the leftmost lane on the highway."
                    elif other_vehicle['lane_id'] == -2:
                        answer = f"The {other_vehicle_description} is driving on the second lane from the left " +\
                                                                                                f"on the highway."
                    elif other_vehicle['lane_id'] == -3:
                        answer = f"The {other_vehicle_description} is driving on the third lane from the left " +\
                                                                                                f"on the highway."
                    elif other_vehicle['lane_id'] == -4:
                        answer = f"The {other_vehicle_description} is driving on the fourth lane from the left " +\
                                                                                                f"on the highway."
                    else:   
                        answer = f"The {other_vehicle_description} is driving on the highway."

            # Handle cases where the ego vehicle is in a junction, and the other vehicle is on the road the
            # ego vehicle will enter
            elif (ego_vehicle['is_in_junction'] and other_vehicle['road_id'] in ego_vehicle['next_road_ids']):
                if is_ego_on_highway:
                    answer = f"The {other_vehicle_description} is on the highway."
                else:
                    answer = f"The {other_vehicle_description} is after the junction on the road the ego vehicle " +\
                                                                                f"will enter. It {orientation_str}."

            # Handle cases where neither vehicle is in a junction, and both are on the same road
            elif not other_vehicle['is_in_junction'] and same_road:
                value = int(other_vehicle['lane_relative_to_ego'])
                s_or_no_s = 's' if abs(value) > 1 else ''
                if value == 0:
                    # value = 'on the same lane as'
                    pass
                elif value > 0:
                    value = f"{number_to_word(abs(value))} lane{s_or_no_s} to the right of"
                else: # value < 0
                    value = f"{number_to_word(abs(value))} lane{s_or_no_s} to the left of"

                bike_lane_str = ''
                if other_vehicle['lane_type_str'] == 'Biking':
                    bike_lane_str = ' on the bike lane'

                moving_action = 'standing' if other_vehicle['speed'] < 0.5 else 'driving'

                if same_direction:
                    if value == 0:
                        answer = f"The {other_vehicle_description} is on the same road {moving_action} on the " +\
                                                                        f"lane of the ego vehicle."
                    else:
                        answer = f"The {other_vehicle_description} is on the same road {moving_action} in the " +\
                                                    f"same direction. It is{bike_lane_str} {value} the ego vehicle."
                else:
                    answer = f"The {other_vehicle_description} is on the same road {moving_action} in the opposite " +\
                                                            f"direction. It is{bike_lane_str} {value} the ego vehicle."

                if is_ego_in_entry_lane:
                    answer = f"The {other_vehicle_description} is in the same lane leading to the highway as " +\
                                                                                                f"the ego vehicle."

            
            # Handle the HighwayCutIn scenario
            if scenario == "HighwayCutIn" and \
                                                        other_vehicle['lane_relative_to_ego'] != 0:
                # Currently we can't differentiate between acceleration lane and entry lane
                answer = f"The {other_vehicle_description} is on the acceleration lane of the highway to the right " +\
                                                                                                f"of the ego vehicle."

            # Add the question-answer pair to the conversation
            self.add_qas_questions(qa_list=qas_conversation_vehicle,
                                    chain=4,
                                    layer=0,
                                    qa_type='perception',
                                    connection_up=[(4,1), (4,2), (4,3)],
                                    connection_down=[(3,0),(3,2),(3,3)],
                                    question=question,
                                    answer=answer,
                                    object_id=other_vehicle['id'],
                                    object_tags=object_tags)
        
            return pointing_towards_junction


        # main contents of this function starts here 
        qas_conversation_vehicle = []

        # Initialize the distance to the next junction for the ego vehicle
        ego_distance_to_junction = ego_vehicle['distance_to_junction']
        if ego_distance_to_junction is None:
            ego_distance_to_junction = 1000 # Set a default value if distance to junction is not available

        speed_limit_kmh = int(get_speed_limit(scene_data))
        # this value only used to indicate specified scenario
            
        # Flags to indicate if the ego vehicle is in an acceleration lane, exit lane, or entry lane
        is_ego_on_highway = False
        is_ego_in_accel_lane = False
        is_other_veh_in_accel_lane = False
        is_ego_in_exit_lane = False
        is_ego_in_entry_lane = False

        # Flag to indicate if the ego vehicle is about to exit the highway
        ego_about_to_exit_highway = False

        # List of scenario names that are considered highway scenarios
        highway_scenario_names = [
            "EnterActorFlow", 
            "EnterActorFlowV2", 
            "HighwayCutIn", 
            "HighwayExit", 
            "MergerIntoSlowTraffic",
            "MergerIntoSlowTrafficV2",
            "YieldToEmergencyVehicle",
        ]

        # Checks depend on scenario type and set flags accordingly
        if scenario == "HighwayCutIn":
            is_ego_on_highway = True
            if ego_vehicle['is_in_junction'] or ego_distance_to_junction < 25:
                is_other_veh_in_accel_lane = True
        elif scenario == "HighwayExit" or scenario == "MergerIntoSlowTrafficV2": 
            is_ego_on_highway = True
            if ego_vehicle['is_in_junction'] or ego_distance_to_junction < 25:
                is_ego_in_exit_lane = True
            if ego_vehicle['num_lanes_same_direction'] - ego_vehicle['ego_lane_number'] - 1 == 0 and \
                        current_measurement['command'] == 6 and ego_distance_to_junction < 40:
                ego_about_to_exit_highway = True
        elif scenario in highway_scenario_names and speed_limit_kmh > 50:
            is_ego_on_highway = True
            if scenario == 'MergerIntoSlowTraffic' and ego_vehicle['num_lanes_same_direction'] == 1 and \
                                                                    ego_vehicle['num_lanes_opposite_direction'] == 1:
                is_ego_in_entry_lane = True
                is_ego_in_accel_lane = False
            elif scenario == 'MergerIntoSlowTraffic' and ego_vehicle['num_lanes_same_direction'] > 1:
                is_ego_in_entry_lane = False
                is_ego_in_accel_lane = False
            elif ego_vehicle['is_in_junction'] or ego_distance_to_junction < 25:
                is_ego_in_accel_lane = True
            elif ego_vehicle['num_lanes_same_direction'] == 1 and ego_vehicle['num_lanes_opposite_direction'] == 0:
                is_ego_in_entry_lane = True


        for vehicle in other_vehicles:
            # Check if the vehicle should be considered based on some criteria
            consider_vehicle = self.should_consider_vehicle(vehicle)
            if not consider_vehicle:
                continue

            # Get the position of the ego vehicle (every other vehicles positions are in the local coordinate system
            # of the ego vehicle)
            pos_ego = np.array([0,0,0])
            
            # Get the position of the current vehicle
            pos_vehicle = np.array(vehicle['position'])

            # Calculate the angle between the vehicle and the ego vehicle
            angle_rad = np.arctan2(pos_vehicle[1] - pos_ego[1], pos_vehicle[0] - pos_ego[0])
            angle_deg = angle_rad * 180. / np.pi
            angle_deg = angle_deg % 360. # Normalize the angle to [0, 360]

            # Get the yaw angle (heading) of the vehicle
            vehicle_heading_angle_rad = vehicle['yaw']
            vehicle_heading_angle_deg = vehicle_heading_angle_rad * 180 / np.pi
            vehicle_heading_angle_deg = vehicle_heading_angle_deg % 360. # Normalize the angle to [0, 360]

            other_vehicle_points_towards_ego = abs(vehicle_heading_angle_deg - angle_deg + 180) % 360 < 90

            important_object_str, vehicle_description, vehicle_location_description = get_vehicle_str(vehicle)

            important_objects.append(important_object_str)

            # Project the vehicle's bounding box points onto the image plane
            projected_points, projected_points_meters = project_all_corners(vehicle, self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)
            projected_points_meters[:, 2] -= vehicle['position'][2]

            # Generate a unique key and value for the vehicle object
            key, value = self.generate_object_key_value(
                id=vehicle['id'],
                category='Vehicle',
                visual_description = vehicle_description,
                object_count = len(key_object_infos),
                projected_points = projected_points,
                projected_points_meters=projected_points_meters
            )
            key_object_infos[key] = value
            object_tags = [key]

            pointing_towards_junction = determine_other_vehicle_position(vehicle_location_description, 
                                                                                vehicle, ego_vehicle, 
                                                                                is_ego_on_highway, 
                                                                                is_ego_in_accel_lane, 
                                                                                is_ego_in_exit_lane, 
                                                                                vehicle_description, 
                                                                                is_ego_in_entry_lane,
                                                                                ego_about_to_exit_highway, 
                                                                                scenario,
                                                                                qas_conversation_vehicle,
                                                                                object_tags)
            
            determine_vehicle_trajectory(vehicle_location_description, vehicle, 
                                              vehicle_description, 
                                              qas_conversation_vehicle,
                                              object_tags)
            
            determine_vehicle_motion_status(vehicle_location_description, 
                                                 vehicle, 
                                                 vehicle_description, 
                                                 qas_conversation_vehicle,
                                                 object_tags)
            
            determine_path_crossing(current_measurement, 
                                            ego_vehicle, 
                                            vehicle_location_description, 
                                            vehicle, 
                                            vehicles_by_id, 
                                            vehicle_description, 
                                            scenario, 
                                            ego_distance_to_junction, 
                                            other_vehicle_points_towards_ego, 
                                            vehicle_heading_angle_deg,
                                            pointing_towards_junction, 
                                            is_ego_on_highway, 
                                            is_ego_in_accel_lane, 
                                            is_other_veh_in_accel_lane, 
                                            qas_conversation_vehicle,
                                            object_tags)

        return qas_conversation_vehicle, important_objects, key_object_infos
    
    def generate_perception_questions(self, measurements, scenario, scenario_name, route_number, frame_number):
        """
        Generates perception-based questions and answers based on the given scene data, current measurements,
        and scenario. It processes various objects in the scene, such as vehicles, pedestrians, traffic lights,
        stop signs, and landmarks, and generates questions and answers related to these objects.

        Args:
            scene_data (list): List of dictionaries containing information about objects in the scene.
            measurements (dict): Dictionary containing current measurement data.
            scenario (str): The current scenario.

        Returns:
            combined_qas (dict): Dictionary containing lists of question-answer pairs for different categories.
            num_questions (int): Total number of questions generated.
            num_objects (int): Total number of objects in the scene.
            num_questions_per_category (dict): Dictionary containing the number of questions for each category.
            key_object_infos (dict): Dictionary containing information about objects in the scene.
        """

        # print(f"[debug] generating perception questions, path = {self.current_measurement_path}") # debug
        scene_data = measurements['bounding_boxes']
        
        ego_measurements = {k: v for k, v in measurements.items() if k != 'sensors'}
        # Initialize lists to store different types of objectss
        static_cars = []
        static_objects = []
        other_vehicles = []
        ego_vehicle = None
        pedestrians = []
        traffic_lights = []
        # old_traffic_lights = []
        traffic_signs = []
        stop_signs = []
        # landmarks = []
        # landmark_ids = []   # Needed to avoid duplicates of landmarks
        vehicles_by_id = {}

        # Find ego vehicle first
        for actor in scene_data:
            if actor['class'] == 'ego_vehicle':
                ego_vehicle = actor
                ego = actor

        # Preprocess, add some keys
        ego_location = carla.Location(x=ego['location'][0], y=ego['location'][1], z=ego['location'][2])

        for actor in scene_data:
            # relative position
            actor['position'] = transform_to_ego_coordinates(actor['location'], ego['world2ego'])
            actor['yaw'] = math.radians(ego_vehicle["rotation"][2])
            if actor['class'] == 'vehicle':
                other_vehicles_info = get_other_vehicle_info(self.current_measurement_path, self.map, actor, ego)
                for key, value in other_vehicles_info.items():
                    if key not in actor:
                        actor[key] = value
                actor['steer'] = get_steer_by_future(self.current_measurement_path, actor['id'])
                actor['vehicle_cuts_in'] = is_vehicle_cutting_in(ego, actor, self.map, self.current_measurement_path)

        # Categorize objects from the scene data
        # print(scene_data) # debug
                
        for actor in scene_data:
            if actor['class'] == 'vehicle':
                other_vehicles.append(actor)
                vehicles_by_id[actor['id']] = actor
                if actor['state'] == 'static':
                    static_cars.append(actor)
            elif actor['class'] == 'walker': 
                # actually, b2d don't have walker either.
                pedestrians.append(actor)
            # elif actor['class'] == 'landmark' and actor['id'] not in landmark_ids:
            #     landmarks.append(actor)
            #     landmark_ids.append(actor['id'])
            # elif actor['class'] == 'traffic_light_vqa':
            #     traffic_lights.append(actor)
            elif actor['class'] == 'traffic_light':
                traffic_lights.append(actor)
            elif actor['class'] == 'traffic_sign':
                traffic_signs.append(actor)
                if 'stop' in actor['type_id']:
                    # print("[debug] we have a stop sign here.") # debug
                    stop_signs.append(actor)
                if 'static' in actor['type_id']:
                    static_objects.append(actor)
                # pdm_lite only has stop sign, we have to fix this. (IMPORTANT)
            elif 'static' in actor['class']:
                static_objects.append(actor)

        important_objects = []
        key_object_infos = {}
        
        # _, ego['distance_to_junction'] = find_first_junction_in_direction(self.map, ego_location)
        # ego['is_in_junction'] = is_vehicle_in_junction(self.map, ego_location)

        lane_info = get_lane_info(self.map, ego_location)
        for key, value in lane_info.items():
            if key not in ego:
                ego[key] = value

        ego['virtual_steer'] = get_steer_by_future(self.current_measurement_path, ego['id'])
        ego['hazard_detected_10'] = False
        res = vehicle_obstacle_detected(ego, other_vehicles, self.map, 10)
        affected_by_vehicle_10, hazard_actor_10 = res
        if affected_by_vehicle_10:
                ego['hazard_detected_10'] = True
                ego['affects_ego_10'] = hazard_actor_10['id']
                ego['affects_ego_10_id'] = hazard_actor_10['id']
                ego['affects_ego_10_dis'] = hazard_actor_10['distance']

        ego['hazard_detected_15'] = False
        affected_by_vehicle_15, hazard_actor_15 = vehicle_obstacle_detected(ego, other_vehicles, self.map, 15)
        if affected_by_vehicle_15:
                ego['hazard_detected_15'] = True
                ego['affects_ego_15'] = hazard_actor_15['id']
                ego['affects_ego_15_id'] = hazard_actor_15['id']
                ego['affects_ego_15_dis'] = hazard_actor_15['distance']

        ego['hazard_detected_20'] = False
        affected_by_vehicle_20, hazard_actor_20 = vehicle_obstacle_detected(ego, other_vehicles, self.map, 20)
        if affected_by_vehicle_20:
                ego['hazard_detected_20'] = True
                ego['affects_ego_20'] = hazard_actor_20['id']
                ego['affects_ego_20_id'] = hazard_actor_20['id']
                ego['affects_ego_20_dis'] = hazard_actor_20['distance']        
        ego['hazard_detected_40'] = False
        affected_by_vehicle_40, hazard_actor_40 = vehicle_obstacle_detected(ego, other_vehicles, self.map, 40)
        if affected_by_vehicle_40:
                ego['hazard_detected_40'] = True
                ego['affects_ego_40'] = hazard_actor_40['id']
                ego['affects_ego_40_id'] = hazard_actor_40['id']
                ego['affects_ego_40_dis'] = hazard_actor_40['distance']

        # original only raise this flag when ego vehicle overcomes an obstacle
        # measurements['changed_route'] = is_ego_changing_lane_due_to_obstacle(ego_measurements, self.map, scene_data)
        _, _, measurements['changed_route'] = detect_lane_change_by_time(self.map, ego['id'], self.current_measurement_path)
        measurements['control_brake'] = measurements['should_brake']
        measurements['command'] = measurements['command_near']
        measurements['target_point'] = [measurements['x_target'], measurements['y_target']]

        # for [debug]
        for key, value in measurements.items():
            self.appended_measurements[key] = value

        # Generate questions and answers for different categories
        res = self.generate_vehicle_information(other_vehicles, ego, important_objects, key_object_infos,
                                                scene_data, vehicles_by_id, measurements, scenario)

        qas_conversation_vehicle, important_objects, key_object_infos = res
        
        res = self.analyze_road_layout(ego, other_vehicles, scene_data, important_objects, key_object_infos, measurements, scenario)
        qas_conversation_roadlayout, important_objects, key_object_infos = res
        
        res = self.process_stop_signs(stop_signs, important_objects, key_object_infos)
        qas_conversation_stopsign, important_objects, key_object_infos, ss_info, ss_object_tags = res

        res = self.process_traffic_lights(traffic_lights, ego, important_objects, key_object_infos)
        qas_conversation_trafficlight, important_objects, key_object_infos, tl_info, tl_object_tags = res
        
        res = self.process_pedestrians(pedestrians, important_objects, key_object_infos)
        qas_conversation_pedestrian, important_objects, key_object_infos = res
        
        res = self.generate_ego_vehicle_actions(ego_vehicle, pedestrians, ego, important_objects, key_object_infos,
                                                vehicles_by_id, tl_info, ss_info, static_objects, measurements, scene_data,
                                                scenario, stop_signs, ss_object_tags, tl_object_tags)
        qas_conversation_ego, important_objects, key_object_infos = res
        
        ######## for [debug] appendix ########
        append_dict = {
            'ego': ego,
            'measurements': self.appended_measurements,
            'scene_data': scene_data
        }

        file_name = f'{self.output_graph_directory}/appendix/' \
                            f'{scenario_name}/{int(frame_number):05d}.json'
        os.makedirs(os.path.dirname(file_name), exist_ok=True)
        with open(file_name, 'w', encoding='utf-8') as f:
            json.dump(append_dict, f, sort_keys=True, indent=4)
        ######## for [debug] appendix ########

        num_objects = len(important_objects)
        num_questions = len(qas_conversation_vehicle) + len(qas_conversation_roadlayout) + \
                        len(qas_conversation_stopsign) + len(qas_conversation_trafficlight) + \
                        len(qas_conversation_ego)
        num_questions = num_questions // 2 # Because we have two entries per question
        num_questions += 1 # Because we have the question about the important objects

        num_questions_per_category = {
            'dynamic_vehicles': len(qas_conversation_vehicle) // 2,
            'roadlayout': len(qas_conversation_roadlayout) // 2,
            'stopsign': len(qas_conversation_stopsign) // 2,
            'trafficlight': len(qas_conversation_trafficlight) // 2,
            'pedestrian': len(qas_conversation_pedestrian) // 2,
            'ego': len(qas_conversation_ego) // 2,
        }

        qas_conversation_objects = []
        question = 'What are the important objects in the scene?'
        concatenated_important_objects = ''

        # Merge same objects and count identical objects in the same direction
        grouped_items = {}
        keep_items = []
        # print(f'[debug] finally, current_index = {self.current_measurement_index}') # debug
        # print(f"[debug] finally, important_objects = {important_objects}") # [debug]
        for obj_idx, obj in enumerate(important_objects):
            item_parts = obj.split(" to the ")
            if item_parts[0].startswith('the '):
                item_parts[0] = item_parts[0][4:]
            if len(item_parts) == 1:
                keep_items.append(obj)
            else:
                if item_parts[1] not in grouped_items:
                    grouped_items[item_parts[1]] = []
                grouped_items[item_parts[1]].append(item_parts[0])

        result = []
        for key, values in grouped_items.items():
            counted_values = dict(Counter(values))
            organize = []
            for key1, values1 in counted_values.items():
                if values1 > 1:
                    organize.append((f'the {values1} {key1}s'))
                else:
                    organize.append((f'the {key1}'))

            res = ''
            for obj_idx, obj in enumerate(organize):
                separator = ', '
                if obj_idx+1 == len(organize)-1:
                    separator = ' and '
                if obj_idx == len(organize)-1:
                    separator = ''
                res += f'{obj}{separator}'
            result.append(res+ f' to the {key}')

        # Merge result with keep_items
        important_objects_merged = keep_items + result

        # Concatenate important objects for the answer
        for obj_idx, obj in enumerate(important_objects_merged):
            separator = ','
            if obj_idx+1 == len(important_objects_merged)-1:
                separator = ' and'
            if obj_idx == len(important_objects_merged)-1:
                separator = ''
            concatenated_important_objects += f' {obj}{separator}'

        if len(important_objects_merged) == 0:
            answer = 'There are no important objects in the scene.'
        elif len(important_objects) == 1:
            answer = f'The important object in the scene is{concatenated_important_objects}.'
        else:
            answer = f'The important objects in the scene are{concatenated_important_objects}.'

        # Add the question and answer to the conversation
        self.add_qas_questions(qa_list=qas_conversation_objects, 
                                chain=0,
                                layer=0,
                                qa_type='perception',
                                connection_up=-1,
                                connection_down=-1,
                                question=question,
                                answer=answer,
                                object_tags=list(key_object_infos.keys()))
        
        combined_qas = {
            'important_objects': qas_conversation_objects,
            'dynamic_vehicles': qas_conversation_vehicle,
            'roadlayout': qas_conversation_roadlayout,
            'stopsign': qas_conversation_stopsign,
            'trafficlight': qas_conversation_trafficlight,
            'pedestrian': qas_conversation_pedestrian,
            'ego': qas_conversation_ego,
        }

        return combined_qas, num_questions, num_objects, num_questions_per_category, key_object_infos

    def analyze_road_layout(self, ego_vehicle_info, vehicle_info, scene_data, important_objects, key_object_infos, current_measurement, scenario):
        """
        This method answers the following questions:
        - Is the ego vehicle at a junction?
        - The ego vehicle wants to {command_str}. Which lanes are important to watch out for?
        - How many lanes are there in the {name} direction {to_or_as} the ego car?
        - On which lane is the ego vehicle (left most lane of the lanes going in the same direction is
                                                                                                indicated with 0)?
        - What lane marking is on the {name} side of the ego car?
        - In which direction is the ego car allowed to change lanes?
        - From which side are other vehicles allowed to change lanes into the ego lane?

        Args:
            ego_vehicle_info (dict): A dictionary containing information about the ego vehicle's lane and 
                                                                                            surrounding conditions.
            important_objects (list): A list of important objects around the ego vehicle.
            key_object_infos (dict): A dictionary containing information about key objects around the ego vehicle.
            current_measurement (dict): A dictionary containing the current measurement data.
            scenario (str): The name of the current scenario.

        Returns:
            tuple: A tuple containing the following elements:
                - qas_conversation_roadlayout (list): A list of question-answer pairs related to the road layout.
                - important_objects (list): The updated list of important objects around the ego vehicle.
                - key_object_infos (dict): The updated dictionary containing information about key objects around 
                                                                                                    the ego vehicle.
        """

        def lane_change_analysis(is_acceleration_lane, command_int, ego_vehicle_info, is_junction, 
                                                                                    qas_conversation_roadlayout):
            """
            Answers "From which side are other vehicles allowed to change lanes into the ego lane?".

            Args:
                is_acceleration_lane (bool): Indicates if the ego vehicle is on an acceleration lane.
                command_int (int): An integer representing a command related to lane changes.
                ego_vehicle_info (dict): A dictionary containing information about the ego vehicle's lane and 
                                                                                                surrounding conditions.
                is_junction (bool): Indicates if the ego vehicle is in a junction.
                qas_conversation_roadlayout (list): A list to store question-answer pairs related to the road layout.
            """

            # Lane change analysis
            question = "From which side are other vehicles allowed to change lanes into the ego lane?"
            if is_acceleration_lane and command_int==5:
                answer = f"Vehicles could potentially change lanes from the left side but it is very unlikely since " +\
                                                                        f"the ego vehicle is on an acceleration lane."
            elif ego_vehicle_info['lane_change'] == carla.LaneChange.NONE:
                if ego_vehicle_info['num_lanes_same_direction'] == 1:
                    answer = "There are no lane changes possible since the ego vehicle is on a one lane road."
                else:
                    answer = "There are no lane changes allowed from another driving lane into the ego lane."
            elif ego_vehicle_info['lane_change'] == carla.LaneChange.Right:
                answer = "Vehicles are allowed to change lanes from the right side."
            elif ego_vehicle_info['lane_change'] == carla.LaneChange.Left:
                answer = "Vehicles are allowed to change lanes from the left side."
            elif ego_vehicle_info['lane_change'] == carla.LaneChange.Both:
                answer = "Vehicles are allowed to change lanes from both sides."
            else:
                raise NotImplementedError()

            # Handle parking lanes
            if ego_vehicle_info['parking_left'] and ego_vehicle_info['parking_right'] and \
                                                         ego_vehicle_info['lane_change'] == 0:
                if ego_vehicle_info['num_lanes_opposite_direction'] >= 1:
                    answer += " But vehicles that are parked on the right side of the road are allowed to change " +\
                                                                                        f"lanes into the ego lane."
                else:
                    answer += " But vehicles that are parked on the left and right side of the road are allowed to " +\
                                                                                    f"change lanes into the ego lane."
            elif ego_vehicle_info['parking_left'] and (ego_vehicle_info['lane_change'] != 2 and \
                        ego_vehicle_info['lane_change'] != 3) and ego_vehicle_info['num_lanes_opposite_direction'] == 0:
                if ego_vehicle_info['lane_change'] == 0:
                    answer += " But vehicles that are parked on the left side of the road are allowed to change " +\
                                                                                        f"lanes into the ego lane."
                else:
                    answer += " And vehicles that are parked on the left side of the road are allowed to change " +\
                                                                                        f"lanes into the ego lane."
            elif ego_vehicle_info['parking_right'] and (ego_vehicle_info['lane_change'] != 1 and \
                                                                                ego_vehicle_info['lane_change'] != 3):
                if ego_vehicle_info['lane_change'] == 0:
                    answer += " But vehicles that are parked on the right side of the road are allowed to " +\
                                                                                f"change lanes into the ego lane."
                else:
                    answer += " And vehicles that are parked on the right side of the road are allowed to " +\
                                                                                f"change lanes into the ego lane."

            if ego_vehicle_info['lane_type_str'] == 'Parking':
                answer = "The ego vehicle is on a parking lane and vehicles only enter the lane to park."

            # Handle junctions
            if is_junction:
                answer = "It is not possible to tell since the ego vehicle is in a junction."

            # if current_measurement['changed_route'] and ('TwoWays' in scenario or 'HazardAtSideLaneTwoWays' in scenario):
            #     answer = "The ego vehicle overtakes an obstruction. We do not expect vehicles to change " +\
            #                                                                             f"into the ego lane."

            # Store the question-answer pair
            self.all_qa_pairs.append((question, answer))

            # Add the question-answer pair to the conversation roadlayout
            self.add_qas_questions(qa_list=qas_conversation_roadlayout,
                                    chain=3,
                                    layer=6,
                                    qa_type='prediction',
                                    connection_up=[(6,0)],
                                    connection_down=[(3,2), (3,3), (3,4), (3,5)],
                                    question=question,
                                    answer=answer)

        def analyze_ego_lane_change_direction(is_acceleration_lane, command_int, ego_vehicle_info, vehicle_info, is_junction, 
                                                                                        qas_conversation_roadlayout):
            """
            Answer "In which direction is the ego car allowed to change lanes?".

            Args:
                is_acceleration_lane (bool): Indicates if the ego vehicle is on an acceleration lane.
                command_int (int): An integer representing a command related to lane changes.
                ego_vehicle_info (dict): A dictionary containing information about the ego vehicle's lane and 
                                                                                            surrounding conditions.
                is_junction (bool): Indicates if the ego vehicle is in a junction.
                qas_conversation_roadlayout (list): A list to store question-answer pairs related to the road layout.
            """
            # Lane change direction analysis
            question = "In which direction is the ego car allowed to change lanes?"
            if is_acceleration_lane and command_int==5:
                answer = f"The ego vehicle is allowed to change lanes to the left to enter the highway."
            elif ego_vehicle_info['lane_change'] == 0:
                if ego_vehicle_info['num_lanes_same_direction'] == 1:
                    answer = "The ego vehicle can not change lanes since it is on a one lane road."
                else:
                    answer = "The ego vehicle is not allowed to change lanes to another driving lane."
            elif ego_vehicle_info['lane_change'] == 1:
                answer = "The ego vehicle is allowed to change lanes to the right."
            elif ego_vehicle_info['lane_change'] == 2:
                answer = "The ego vehicle is allowed to change lanes to the left."
            elif ego_vehicle_info['lane_change'] == 3:
                answer = "The ego vehicle is allowed to change lanes to the left and right."
            else:
                raise NotImplementedError()
            
            left_clear_distance = get_clear_distance_of_lane(vehicle_info, -1, True)
            right_clear_distance = get_clear_distance_of_lane(vehicle_info, 1, True)

            append_sentence = ""
            occupy_threshold = 25.0
            if ego_vehicle_info['lane_change'] in [1, 3]: # right
                if right_clear_distance <= occupy_threshold:
                    append_sentence = " But it couldn't change to right lane because it is currently occupied."
            if ego_vehicle_info['lane_change'] in [2, 3]: # left
                if left_clear_distance <= occupy_threshold:
                    append_sentence = " But it couldn't change to left lane because it is currently occupied."
            if ego_vehicle_info['lane_change'] in [3]: # both
                if left_clear_distance <= occupy_threshold and right_clear_distance <= occupy_threshold:
                    append_sentence = " But it couldn't change to either side because they're all currently occupied." 

            answer += append_sentence

            # Handle parking lanes
            if ego_vehicle_info['parking_left'] and ego_vehicle_info['parking_right'] and \
                                                        ego_vehicle_info['lane_change'] == 0:
                if ego_vehicle_info['num_lanes_opposite_direction'] >= 1:
                    answer += " But it could change to the parking lane on the right side of the road."
                else:
                    answer += " But it could change to the parking lane on the left and right side of the road."
            elif ego_vehicle_info['parking_left'] and (ego_vehicle_info['lane_change'] != 2 and \
                                                        ego_vehicle_info['lane_change'] != 3) and \
                                                        ego_vehicle_info['num_lanes_opposite_direction'] == 0:
                if ego_vehicle_info['lane_change'] == 0:
                    answer += " But it could change to the parking lane on the left side of the road."
                else:
                    answer += " It could also change to the parking lane on the left side of the road."
            elif ego_vehicle_info['parking_right'] and (ego_vehicle_info['lane_change'] != 1 and \
                                                            ego_vehicle_info['lane_change'] != 3):
                if ego_vehicle_info['lane_change'] == 0:
                    answer += " But it could change to the parking lane on the right side of the road."
                else:
                    answer += " It could also change to the parking lane on the right side of the road."

            # Handle parking lane
            if ego_vehicle_info['lane_type_str'] == 'Parking':
                answer = "The ego vehicle is on a parking lane and is allowed to merge into the driving lane."

            # Handle junctions
            if is_junction:
                answer = "It is not possible to tell since the ego vehicle is in a junction."

            # if current_measurement['changed_route'] and ('TwoWays' in scenario or 'HazardAtSideLaneTwoWays' in scenario):
            #     answer = "The ego vehicle overtakes an obstruction. It is not expected to change lanes."

            # Store the question-answer pair
            self.all_qa_pairs.append((question, answer))

            # Add the question-answer pair to the conversation roadlayout
            self.add_qas_questions(qa_list=qas_conversation_roadlayout,
                                    chain=3,
                                    layer=5,
                                    qa_type='prediction',
                                    connection_up=[(6,0)],
                                    connection_down=[(3,2),(3,3),(3,4)],
                                    question=question,
                                    answer=answer)

        def analyze_lane_marking(ego_vehicle_info, qas_conversation_roadlayout):
            """
            Answer "What lane marking is on the {name} side of the ego car?".

            Args:
                ego_vehicle_info (dict): A dictionary containing information about the ego vehicle's lane and 
                                                                                                surrounding conditions.
                qas_conversation_roadlayout (list): A list to store question-answer pairs related to the road layout.
            """

            keys = ['left_lane', 'right_lane']
            names = ['left', 'right']
            for side_key, side_name in zip(keys, names):
                answer = None
                question = f"What lane marking is on the {side_name} side of the ego car?"

                # Determine the lane marking type
                if ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.NONE:
                    answer = f"There is no lane marking on the {side_name} side of the ego car."
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.Broken:
                    lanetype = "broken"
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.Solid:
                    lanetype = "solid"
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.SolidSolid:
                    lanetype = "double solid"
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.Curb:
                    lanetype = "curb"
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.SolidBroken:
                    lanetype = "solid broken"
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.BrokenSolid:
                    lanetype = "broken solid"
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.BrokenBroken:
                    lanetype = "broken broken"
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.Grass:
                    lanetype = "grass"
                elif ego_vehicle_info[f'{side_key}_marking_type'] is carla.LaneMarkingType.BottsDots:
                    lanetype = "botts dots"
                else:
                    raise ValueError(f"Unknown lane marking type {ego_vehicle_info[f'{side_key}_marking_type']}.")

                # Construct the answer string
                if answer is None:
                    color = ego_vehicle_info[f'{side_key}_marking_color_str']
                    # lower case
                    color = color[:1].lower() + color[1:]
                    if color == 'other':
                        description_str = f'{lanetype}'
                    else:
                        description_str = f'{color} {lanetype} lane'
                    answer = f"The lane marking on the {side_name} side of the ego car is a {description_str}."

                if current_measurement['changed_route'] and \
                            ('TwoWays' in scenario or 'HazardAtSideLaneTwoWays' in scenario):
                    
                    if side_name == 'right':
                        color = ego_vehicle_info[f'left_lane_marking_color_str']
                        # lower case
                        color = color[:1].lower() + color[1:]
                        if color == 'other':
                            description_str = f'{lanetype}'
                        else:
                            description_str = f'{color} {lanetype} lane'
                        answer = f"The lane marking on the right side of the ego car is a {description_str}."
                    else:
                        answer = "It is not possible to tell since the ego vehicle overtakes an obstruction."

                # Store the question-answer pair
                self.all_qa_pairs.append((question, answer))

                # Add the question-answer pair to the conversation roadlayout
                self.add_qas_questions(qa_list=qas_conversation_roadlayout,
                                        chain=3,
                                        layer=4,
                                        qa_type='perception',
                                        connection_up=[(3,5), (3,6)],
                                        connection_down=[(3,2), (3,3)],
                                        question=question,
                                        answer=answer)

        def identify_ego_lane(ego, is_junction, qas_conversation_roadlayout):
            """
            Answers "On which lane is the ego vehicle (left most lane of the lanes going in the same direction is 
                                                                                                    indicated with 0)?".

            Args:
                ego_vehicle_info (dict): A dictionary containing information about the ego vehicle's lane and 
                                                                                                surrounding conditions.
                is_junction (bool): Indicates if the ego vehicle is in a junction.
                qas_conversation_roadlayout (list): A list to store question-answer pairs related to the road layout.
            """

            question = "On which lane is the ego vehicle (left most lane of the lanes going in the same direction "\
                                                                                                "is indicated with 0)?"
            answer = f"The ego vehicle is on lane {ego['ego_lane_number']}."

            # Check if the ego vehicle is on a parking lane
            if ego['lane_type_str'] == 'Parking':
                answer = f"The ego vehicle is on lane {ego['ego_lane_number']} which is the parking lane."

            # Handle junctions
            if is_junction:
                answer = "It is not possible to tell since the ego vehicle is in a junction."

            # if current_measurement['changed_route'] and ('TwoWays' in scenario or 'HazardAtSideLaneTwoWays' in scenario):
            #     answer = f"The ego vehicle is on lane {ego['ego_lane_number']+1} since it overtakes an obstruction."

            # Add the question-answer pair to the conversation roadlayout
            self.add_qas_questions(qa_list=qas_conversation_roadlayout,
                                    chain=3,
                                    layer=3,
                                    qa_type='perception',
                                    connection_up=[(3,1), (3,4), (3,5), (3,6), (4,0)],
                                    connection_down=[(3,0)],
                                    question=question,
                                    answer=answer)

        def analyze_lanes_direction(ego_vehicle_info, is_junction, qas_conversation_roadlayout):
            """
            Answer "How many lanes are there in the {name} direction {to_or_as} the ego car?".

            Args:
                ego_vehicle_info (dict): A dictionary containing information about the ego vehicle's lane and 
                                                                                        surrounding conditions.
                is_junction (bool): Indicates if the ego vehicle is in a junction.
                qas_conversation_roadlayout (list): A list to store question-answer pairs related to the road layout.
            """

            keys = ['num_lanes_same_direction', 'num_lanes_opposite_direction']
            names = ['same', 'opposite']
            for direction_key, direction_name in zip(keys, names):
                lane_count = number_to_word(ego_vehicle_info[direction_key])
                lane_count_int = ego_vehicle_info[direction_key]

                # Handle parking lanes
                if ego_vehicle_info['lane_type_str'] == 'Parking' and direction_name == 'same':
                    lane_count = number_to_word(ego_vehicle_info[direction_key]-1)
                    lane_count_int = ego_vehicle_info[direction_key]-1

                s_or_no_s = 's' if lane_count_int > 1 else ''
                are_or_is = 'are' if lane_count_int > 1 else 'is'
                to_or_as = 'to' if direction_name == 'opposite' else 'as'

                question = f"How many lanes are there in the {direction_name} direction {to_or_as} the ego car?"

                if lane_count_int == 0:
                    answer = f"There are no lanes in the {direction_name} direction."
                else:
                    answer = f"There {are_or_is} {lane_count} lane{s_or_no_s} in the {direction_name} direction."

                # Handle junctions
                if is_junction:
                    answer = "It is not possible to tell since the ego vehicle is in a junction."

                # Store the question-answer pair
                self.all_qa_pairs.append((question, answer))

                # Add the question-answer pair to the conversation roadlayout
                self.add_qas_questions(qa_list=qas_conversation_roadlayout,
                                        chain=3,
                                        layer=2,
                                        qa_type='perception',
                                        connection_up=[(3,1), (3,3), (3,4), (3,5), (3,6), (3,7), (4,0)],
                                        connection_down=[(3,0)],
                                        question=question,
                                        answer=answer)
        
        def detect_junction_proximity(is_acceleration_lane, important_objects, key_object_infos, 
                                      is_other_acceleration_lane, is_exit_lane, about_to_exit, ego_vehicle_info, 
                                      is_highway, distance_to_junction, scenario, current_measurement, 
                                      qas_conversation_roadlayout):
            """
            Answer "Is the ego vehicle at a junction?".

            Args:
                is_acceleration_lane (bool): Indicates if the ego vehicle is on an acceleration lane.
                important_objects (list): A list to store important objects detected in the scene.
                key_object_infos (dict): A dictionary to store information about key objects detected.
                is_other_acceleration_lane (bool): Indicates if the ego vehicle is close to an entry lane 
                                                                                                        on the highway.
                is_exit_lane (bool): Indicates if the ego vehicle is on an exit lane.
                about_to_exit (bool): Indicates if the ego vehicle is about to exit the highway.
                ego_vehicle_info (dict): A dictionary containing information about the ego vehicle's lane and 
                                                                                                surrounding conditions.
                is_highway (bool): Indicates if the ego vehicle is on a highway.
                distance_to_junction (float): The distance to the nearest junction.
                scenario (str): The name of the current scenario.
                current_measurement (dict): A dictionary containing the current measurement data.
                qas_conversation_roadlayout (list): A list to store question-answer pairs related to the road layout.
            """
            
            scenario = scenario.split("_")[0]
            question = 'Is the ego vehicle at a junction?'

            if is_acceleration_lane:
                is_junction = False
                answer = 'The ego vehicle is on an acceleration lane and about to enter the highway.'
                important_objects.append('a highway entry')

                key, value = self.generate_object_key_value(
                    id=None,
                    category='Traffic element', 
                    visual_description='Junction',
                    object_count=len(key_object_infos)
                )
                key_object_infos[key] = value
            elif is_other_acceleration_lane:
                is_junction = False
                answer = "The ego vehicle is on the highway close to the entry lane."
            elif is_exit_lane:
                is_junction = False
                answer = 'The ego vehicle is on an exit lane and about to exit the highway.'
                important_objects.append('a highway exit')

                key, value = self.generate_object_key_value(
                    id=None,
                    category='Traffic element', 
                    visual_description='Junction',
                    object_count=len(key_object_infos)
                )
                key_object_infos[key] = value
            elif about_to_exit:
                is_junction = False
                answer = "The ego vehicle is on the highway close to the exit lane."
            elif is_highway and ego_vehicle_info['is_in_junction']:
                is_junction = False
                answer = 'The ego vehicle is on the highway potentially close to a junction.'
                important_objects.append('a junction')

                key, value = self.generate_object_key_value(
                    id=None,
                    category='Traffic element', 
                    visual_description='Junction',
                    object_count=len(key_object_infos)
                )
                key_object_infos[key] = value
            elif ego_vehicle_info['is_in_junction']:
                is_junction = True
                answer = 'The ego vehicle is in a junction.'
                important_objects.append('a junction')

                key, value = self.generate_object_key_value(
                    id=None,
                    category='Traffic element', 
                    visual_description='Junction',
                    object_count=len(key_object_infos)
                )
                key_object_infos[key] = value
            elif distance_to_junction < 25:
                is_junction = False
                answer = 'The ego vehicle is right before a junction.'
                important_objects.append('a junction')

                key, value = self.generate_object_key_value(
                    id=None,
                    category='Traffic element', 
                    visual_description='Junction',
                    object_count=len(key_object_infos)
                )
                key_object_infos[key] = value
            else:
                is_junction = False
                answer = 'No, the ego vehicle is not at a junction.'
            
            # Handle specific scenarios
            if scenario == 'InterurbanActorFlow':
                if current_measurement['command'] == 5 and current_measurement['next_command'] != 1:
                    answer = "The ego vehicle is on an interurban road close to a point where a new turning " +\
                                                                                                f"lane emerges."
                elif current_measurement['command'] == 5 and current_measurement['next_command'] == 1 and \
                                                                                            distance_to_junction < 35:
                    answer = "The ego vehicle is on a turning lane close to a junction."
                elif current_measurement['command'] == 5 and current_measurement['next_command'] == 1:
                    answer = "The ego vehicle is on a turning lane approaching a junction."

            # Add the question-answer pair to the conversation roadlayout
            self.add_qas_questions(qa_list=qas_conversation_roadlayout,
                                    chain=3,
                                    layer=0,
                                    qa_type='perception',
                                    connection_up=[(1,0), (2,0), (3,1), (3,2), (3,3), (4,0)],
                                    connection_down=-1,
                                    question=question,
                                    answer=answer)
            
            return is_junction

        def analyze_important_lanes(command_description, command_int, lane_change_soon, is_junction, ego_vehicle_info, 
                                    next_command_int, is_acceleration_lane, about_to_exit, about_to_exit_far, scenario, 
                                    current_measurement, qas_conversation_roadlayout, is_highway, 
                                    is_other_acceleration_lane):
            """
            Answer "The ego vehicle wants to {command_description}. Which lanes are important to watch out for?".

            Args:
                command_description (str): A string describing the ego vehicle's current command (e.g., "turn left", 
                                                                                                        "go straight").
                command_int (int): An integer representing the current command.
                lane_change_soon (bool): Indicates if the ego vehicle will change lanes soon.
                is_junction (bool): Indicates if the ego vehicle is at a junction.
                ego_vehicle_info (dict): A dictionary containing information about the ego vehicle's lane and 
                                                                                            surrounding conditions.
                next_command_int (int): An integer representing the next command after the current one.
                is_acceleration_lane (bool): Indicates if the ego vehicle is on an acceleration lane.
                about_to_exit (bool): Indicates if the ego vehicle is about to exit the highway.
                about_to_exit_far (bool): Indicates if the ego vehicle is far from the exit lane.
                scenario (str): The name of the current scenario.
                current_measurement (dict): A dictionary containing the current measurement data.
                qas_conversation_roadlayout (list): A list to store question-answer pairs related to the road layout.
            """

            question = f"The ego vehicle wants to {command_description}. Which lanes are important to watch out for?"
            answer = ''
            scenario = scenario.split("_")[0]

            if command_int == 1:
                answer = f"The ego vehicle should pay particular attention to traffic coming from the left side of " +\
                                "the intersection and is going straight or turning left, traffic coming from the " +\
                                "right and going straight or turning left and to oncoming traffic."
            elif command_int == 2:
                answer = f"The ego vehicle should pay particular attention to traffic coming straight ahead from " +\
                                f"the left side of the intersection and to oncoming traffic turning left."
            elif command_int == 3:
                if is_highway:
                    if ego_vehicle_info['lane_change'] == carla.LaneChange.NONE:
                        if is_other_acceleration_lane:
                            answer = f"The ego vehicle should pay particular attention to the vehicle on the " \
                                        "acceleration lane to the right."
                        else:
                            answer = f"Since there are no lane changes allowed, the ego does not need to pay " \
                                        "particular attention to any neighboring lane."
                    else:
                        if ego_vehicle_info['lane_change'] == carla.LaneChange.Right:
                            add_to_answer = "to the right lane of the highway."
                        elif ego_vehicle_info['lane_change'] == carla.LaneChange.Left:
                            add_to_answer = "to the left lane of the highway."
                        elif ego_vehicle_info['lane_change'] == carla.LaneChange.Both:
                            add_to_answer = "to both neighboring lanes of the highway."

                        if is_other_acceleration_lane:
                            answer = f"The ego vehicle should pay particular attention to the vehicle on the " \
                                        "acceleration lane to the right and " + add_to_answer
                        else:
                            answer = f"The ego vehicle should pay particular attention to " + add_to_answer
                else:
                    answer = f"The ego vehicle should pay particular attention to traffic coming from the left " +\
                                    f"side of the intersection and is going straight or turning left, traffic " +\
                                    f"coming from the right and going straight or turning right and to oncoming " \
                                    f"traffic turning left."
            elif command_int == 4 and not lane_change_soon:
                if is_junction:
                    answer = f"The ego vehicle should pay attention to other vehicles in the junction."
                elif ego_vehicle_info['num_lanes_same_direction'] == 1 and \
                                                                ego_vehicle_info['num_lanes_opposite_direction'] == 0:
                    if ego_vehicle_info['parking_left'] or ego_vehicle_info['parking_right']:
                        answer = f"There are no other driving lanes to watch out for since the ego vehicle is on a " +\
                                        f"one lane road. But the ego vehicle should watch out for the parking lane."
                    else:
                        answer = f"There are no other driving lanes to watch out for since the ego vehicle is on " +\
                                                                                                f"a one lane road."
                elif ego_vehicle_info['num_lanes_same_direction'] == 1 and \
                                                                ego_vehicle_info['num_lanes_opposite_direction'] >= 1:
                    if ego_vehicle_info['parking_left'] or ego_vehicle_info['parking_right']:
                        answer = f"The ego vehicle should watch out for traffic coming from the oncoming lane and " +\
                                                                                            f"for the parking lane."
                    else:
                        answer = f"The ego vehicle should watch out for traffic coming from the oncoming lane."
                elif ego_vehicle_info['num_lanes_same_direction'] > 1 and \
                                                                ego_vehicle_info['num_lanes_opposite_direction'] == 0:
                    answer = f"The ego vehicle should pay particular attention to traffic changing lanes " +\
                                                                                    f"from neighboring lanes."
                elif ego_vehicle_info['num_lanes_same_direction'] > 1 and \
                                                                ego_vehicle_info['num_lanes_opposite_direction'] >= 1:
                    if ego_vehicle_info['ego_lane_number'] == 0:
                        # ego driving at left most lane so uncoming traffic is important to watch
                        answer = f"The ego vehicle should pay particular attention to traffic changing lanes from " +\
                                                f"neighboring lanes and for traffic coming from the oncoming lane."
                    else:
                        # ego not driving at left most lane so uncoming traffic is not important to watch
                        answer = f"The ego vehicle should pay particular attention to traffic changing lanes from " +\
                                                                                                f"neighboring lanes."
                else:
                    raise ValueError(f"Unknown number of lanes {ego_vehicle_info['num_lanes_same_direction']} and " +\
                                                            f"{ego_vehicle_info['num_lanes_opposite_direction']}.")
            elif command_int == 5 or (next_command_int == 5 and lane_change_soon):
                if lane_change_soon:
                    answer = f"The ego vehicle should pay particular attention to traffic in the left-hand lane and " +\
                                    f"position itself so that no vehicle is driving on the same height to prepare " +\
                                    f"for the lane change."
                else:
                    answer = f"The ego vehicle should pay particular attention to traffic in the left-hand lane and " +\
                                                                                f"wait for a gap to change lanes."
            elif command_int == 6 or (next_command_int == 6 and lane_change_soon):
                if lane_change_soon:
                    answer = f"The ego vehicle should pay particular attention to traffic in the right-hand lane " +\
                                    f"and position itself so that no vehicle is driving on the same height to " +\
                                    f"prepare for the lane change."
                else:
                    answer = f"The ego vehicle should pay particular attention to traffic in the right-hand lane " +\
                                                                            f"and wait for a gap to change lanes."
            if ego_vehicle_info['bike_lane_left'] and ego_vehicle_info['num_lanes_opposite_direction'] == 0 and \
                                                                            ego_vehicle_info['ego_lane_number'] == 0:
                # no oncoming traffic and ego is on left most lane
                answer += " Additionally, the ego vehicle should have an eye on the bike lane on the left side."
            elif ego_vehicle_info['bike_lane_right']:
                answer += " Additionally, the ego vehicle should have an eye on the bike lane on the right side."

            if is_acceleration_lane and command_int==5:
                answer = f"The ego vehicle should pay particular attention to traffic on the rightmost lane of the " +\
                                f"highway, adjust its speed, and position itself so that no vehicle is driving on " +\
                                f"the same height to prepare for the lane change."
            elif is_acceleration_lane and command_int==6:
                raise ValueError("Lane change to the right on acceleration lane is not possible.")
            elif is_acceleration_lane:
                answer = "The ego vehicle should pay particular attention to the traffic on the highway, which is " +\
                                                                                f"close to the acceleration lane."
            elif about_to_exit:
                answer = "The ego vehicle should pay particular attention to the traffic on the exit lane, since " +\
                                                                                            f"they might slow down."
            elif about_to_exit_far:
                answer = "The ego vehicle is still far away from the exit lane, so it should pay attention to the " +\
                                                                                            f"traffic on the highway."

            if scenario == 'InterurbanActorFlow':
                if current_measurement['command'] == 5 and current_measurement['next_command'] != 1:
                    answer = "The ego vehicle should pay particular attention to the traffic on the turning lane, " +\
                                                                                        f"since they might slow down."
                elif current_measurement['command'] == 5 and current_measurement['next_command'] == 1:
                    answer = "The ego vehicle should pay particular attention to the traffic on the turning lane as " +\
                                    f"they might slow down and to oncoming traffic the ego vehicle needs to cross " +\
                                    f"in order to turn left."
                elif current_measurement['command'] == 1:
                    answer = "The ego vehicle should pay particular attention to oncoming traffic the ego vehicle " +\
                                    f"needs to cross in order to turn left."
                elif current_measurement['command'] == 2 or current_measurement['command'] == 6:
                    raise ValueError("Labels are not implemented yet.")

            if ego_vehicle_info['lane_type_str'] == 'Parking':
                answer = "The ego vehicle should pay particular attention to the traffic in the lane into which the " +\
                                                                f"ego vehicle wants to enter from the parking space."

            if current_measurement['changed_route'] and ('TwoWays' in scenario or 'HazardAtSideLaneTwoWays' in scenario):
                answer = f"The ego vehicle should keep driving regardless of other vehicles since it overtakes an " +\
                                                                                                        f"obstruction."

            # Add the question-answer pair to the conversation roadlayout
            self.add_qas_questions(qa_list=qas_conversation_roadlayout,
                                    chain=3,
                                    layer=1,
                                    qa_type='prediction',
                                    connection_up=[(4,3)],
                                    connection_down=[(3,0), (3,2), (3,3)],
                                    question=question,
                                    answer=answer)


        qas_conversation_roadlayout = []

        distance_to_junction = ego_vehicle_info['distance_to_junction']
        if distance_to_junction is None:
            distance_to_junction = 1000

        speed_limit = int(get_speed_limit(scene_data))

        # Determine if the scenario is a highway scenario and if the ego vehicle is on an acceleration lane
        is_highway = False
        is_acceleration_lane = False
        is_other_acceleration_lane = False
        is_exit_lane = False
        about_to_exit = False
        about_to_exit_far = False
        highway_scenarios = [
            "EnterActorFlow", 
            "EnterActorFlowV2", 
            "HighwayCutIn", 
            "HighwayExit", 
            "MergerIntoSlowTraffic",
            "MergerIntoSlowTrafficV2",
            "YieldToEmergencyVehicle",
        ]

        if scenario == "HighwayCutIn":
            is_highway = True
            if (ego_vehicle_info['is_in_junction'] or distance_to_junction <25):
                is_other_acceleration_lane = True
        elif scenario == "HighwayExit" or scenario == "MergerIntoSlowTrafficV2":
            is_highway = True
            if (ego_vehicle_info['is_in_junction'] or distance_to_junction <25):
                is_exit_lane = True
            if (ego_vehicle_info['num_lanes_same_direction'] - ego_vehicle_info['ego_lane_number']-1 == 0 and \
                                                current_measurement['command']==6 and distance_to_junction <40) or \
                                                ego_vehicle_info['is_in_junction'] or distance_to_junction <10:
                about_to_exit = True
            if (ego_vehicle_info['num_lanes_same_direction'] - ego_vehicle_info['ego_lane_number']-1 == 0 and \
                                                                                current_measurement['command']==6):
                about_to_exit_far = True

        elif scenario in highway_scenarios and speed_limit > 50:
            is_highway = True
            if scenario == 'MergerIntoSlowTraffic' and ego_vehicle_info['num_lanes_same_direction'] == 1 and \
                                                                ego_vehicle_info['num_lanes_opposite_direction'] == 1:
                is_acceleration_lane = False
            elif scenario == 'MergerIntoSlowTraffic' and ego_vehicle_info['num_lanes_same_direction'] > 1:
                is_acceleration_lane = False
            elif (ego_vehicle_info['is_in_junction'] or distance_to_junction <25):
                is_acceleration_lane = True

        is_junction = detect_junction_proximity(is_acceleration_lane, important_objects, key_object_infos, 
                                                is_other_acceleration_lane, is_exit_lane, about_to_exit, 
                                                ego_vehicle_info, is_highway, distance_to_junction, scenario, 
                                                current_measurement, qas_conversation_roadlayout)
        
        command_int = current_measurement['command']
        command_next_int = current_measurement['next_command']
        lane_change_soon = False
        map_command = {
            1: 'go left at the intersection',
            2: 'go right at the intersection',
            3: 'go straight at the intersection',
            4: 'follow the road',
            5: 'do a lane change to the left',
            6: 'do a lane change to the right',        
        }
        command_str = map_command[command_int]

        if current_measurement['next_command'] == 5 or current_measurement['next_command'] == 6:
            # get distance to current_measurement['target_point_next'], ego is at 0,0
            # print(current_measurement)
            # target_point_next = current_measurement['target_point_next']
            target_point_next = [current_measurement['x_command_far'] - current_measurement['x'], current_measurement['y_command_far'] - current_measurement['y']]
            distance_to_target_point_next = np.sqrt(target_point_next[0]**2 + target_point_next[1]**2)
            if distance_to_target_point_next < 20 and current_measurement['next_command'] == 5:
                command_str = 'do a lane change to the left soon'
                lane_change_soon = True

            elif distance_to_target_point_next < 20 and current_measurement['next_command'] == 6:
                command_str = 'do a lane change to the right soon'
                lane_change_soon = True

        if about_to_exit:
            command_str = 'exit the highway'
        elif about_to_exit_far:
            command_str = 'exit the highway'

        # Analyze important lanes based on the current command and road conditions
        analyze_important_lanes(command_str, command_int, lane_change_soon, is_junction, ego_vehicle_info, 
                                command_next_int, is_acceleration_lane, about_to_exit, about_to_exit_far, 
                                scenario, current_measurement, qas_conversation_roadlayout, is_highway, is_other_acceleration_lane)
        
        # Analyze the number of lanes in the same and opposite directions
        analyze_lanes_direction(ego_vehicle_info, is_junction, qas_conversation_roadlayout)

        # Identify the lane the ego vehicle is currently on
        identify_ego_lane(ego_vehicle_info, is_junction, qas_conversation_roadlayout)
    
        # Analyze the lane markings on the left and right sides of the ego vehicle
        analyze_lane_marking(ego_vehicle_info, qas_conversation_roadlayout)

        # Analyze the directions in which the ego vehicle is allowed to change lanes
        analyze_ego_lane_change_direction(is_acceleration_lane, command_int, ego_vehicle_info, vehicle_info, is_junction, 
                                                                                        qas_conversation_roadlayout)

        # Analyze the directions from which other vehicles are allowed to change lanes into the ego lane
        lane_change_analysis(is_acceleration_lane, command_int, ego_vehicle_info, is_junction, 
                                                                                        qas_conversation_roadlayout)

        return qas_conversation_roadlayout, important_objects, key_object_infos