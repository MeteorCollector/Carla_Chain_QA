## DATA-AGENT

boxes 是 data_agent 每一帧采的，
```python
with gzip.open(self.save_path / 'boxes' / (f'{frame:04}.json.gz'), 'wt', encoding='utf-8') as f:
            json.dump(tick_data['bounding_boxes'], f, indent=4)
```

measurements 是 autopilot 里每一帧采的，line 309 开始
```python
data = {
                'x': pos[0],
                'y': pos[1],
                'theta': theta,
                'speed': speed,
                'target_speed': target_speed,
                'x_command': far_node[0],
                'y_command': far_node[1],
                'command': self.commands[-2],
                'waypoints': waypoints, # not in b2d
                'steer': steer,
                'throttle': throttle,
                'brake': brake,
                'junction':         self.junction, # not in b2d, not used
                'vehicle_hazard':   self.vehicle_hazard, # not in b2d
                'light_hazard':     self.traffic_light_hazard, # not in b2d
                'walker_hazard':    self.walker_hazard, # not in b2d
                'stop_sign_hazard': self.stop_sign_hazard, # not in b2d
                'angle':            self.angle, # not in b2d # not used
                'ego_matrix': self._vehicle.get_transform().get_matrix() # not in b2d, not used
                }

data = {
            "pos_global": ego_position.tolist(), # not used
            "theta": ego_orientation,
            "speed": ego_speed,
            "target_speed": target_speed, # x
            "speed_limit": speed_limit,
            "target_point": ego_target_point # command_near
            "target_point_next": ego_next_target_point, # command_far
            "command": self.commands[-2], # command_near
            "next_command": self.next_commands[-2], # command_far
            "aim_wp": ego_aim_point, # not used, seems just target_point
            "route": dense_route,
            "route_original": dense_route_original, # 被用来判断车辆车道操作是否完毕，在 line1284 左右，route_original是车辆变道后新车道的路径起点。
            "changed_route": changed_route,
            "speed_reduced_by_obj_type": speed_reduced_by_obj_type,
            "speed_reduced_by_obj_id": speed_reduced_by_obj_id,
            "speed_reduced_by_obj_distance": speed_reduced_by_obj_distance,
            "steer": steering,
            "throttle": throttle,
            "brake": bool(brake),
            "control_brake": bool(control_brake),
            "junction": bool(self.junction),
            "vehicle_hazard": bool(self.vehicle_hazard),
            "vehicle_affecting_id": self.vehicle_affecting_id, # not used
            "light_hazard": bool(self.traffic_light_hazard),
            "walker_hazard": bool(self.walker_hazard),
            "walker_affecting_id": self.walker_affecting_id, # not used
            "stop_sign_hazard": bool(self.stop_sign_hazard),
            "stop_sign_close": bool(self.stop_sign_close), # not used
            "walker_close": bool(self.walker_close), 
            "walker_close_id": self.walker_close_id, # not used
            "angle": self.angle, # not used
            "augmentation_translation": self.augmentation_translation, # not used
            "augmentation_rotation": self.augmentation_rotation, # not used
            "ego_matrix": self._vehicle.get_transform().get_matrix() # not used
        }
        
        comments:
        target_point (numpy.ndarray): Coordinates of the target point.
        next_target_point (numpy.ndarray): Coordinates of the next target point.
        steering (float): The steering angle for the current frame.
        throttle (float): The throttle value for the current frame.
        brake (float): The brake value for the current frame.
        control_brake (bool): Whether the brake is controlled by the agent or not.
        target_speed (float): The target speed for the current frame.
        speed_limit (float): The speed limit for the current frame.
        tick_data (dict): Dictionary containing the current state of the vehicle.
        speed_reduced_by_obj (tuple): Tuple containing information about the object that caused speed reduction.

        measurements_file = self.save_path / 'measurements' / ('%04d.json' % frame)
        with open(measurements_file, 'w') as f:
            json.dump(data, f, indent=4)
```

## POST-PROCESS

#### 开始：create_qa_pairs(self):

其实也就用了两个文件(line 165 - 172)，也就是 data 和 measurement

上游：

```python
line 64: self.data_boxes_paths = glob.glob(os.path.join(self.data_directory, '**/boxes/*.json.gz'), recursive=True)

line 71: self.data_boxes_paths = list(sorted(self.data_boxes_paths))

line 111:
for path in tqdm.tqdm(self.data_boxes_paths):
            route_dir = '/'.join(path.split('/')[:-2])
            scenario_name = route_dir.split('/')[-2]
            route_number = route_dir.split('/')[-1].split('_')[0] + '_' + route_dir.split('/')[-1].split('_')[1]

path_measurements = path.replace('boxes', 'measurements')
```

```python
# Read data and measurements files
            with gzip.open(path, 'rb') as f:
                file_content = f.read()
                data = json.loads(file_content.decode('utf-8'))

            with gzip.open(path_measurements, 'rb') as f:
                file_content = f.read()
                measurements = json.loads(file_content.decode('utf-8'))
```

生成问答对：res = self.generate_perception_questions(data, measurements, scenario_name)

if self.save_examples: 这里先不看，感觉是可视化。如果 self.save_examples 为真，代码将处理图像并在上面绘制出每个对象的位置（如汽车、交通信号灯等）。然后将原图、调整大小后的图像和带有问题答案数据的图像分别保存到相应的路径。

然后是 Save QA Data，保存了之后，Update statistics, Append QA data to the list, Populate VQA LLAVA format data, 后两个先不用太管，可以用自己的pipeline。最后它保存了统计信息。返回的是vqa_llama_format

#### generate_perception_questions(self, scene_data, measurements, scenario) line 2120 - 2298

data->scene_data, measurements->measurements
初始化和分类对象：

初始化了一些空列表来分类场景中的不同对象，如静态车辆、其他车辆、行人、交通信号灯、停车标志等。
遍历场景数据，将每个对象按类别（如 ego_car、car、walker 等）分类到相应的列表中。
生成问题和答案：

方法通过调用多个生成问题的子方法来为不同类型的对象生成感知问题和答案：
generate_vehicle_information：生成与车辆相关的问题。
analyze_road_layout：生成与道路布局相关的问题。
process_stop_signs：生成与停车标志相关的问题。
process_traffic_lights：生成与交通信号灯相关的问题。
process_pedestrians：生成与行人相关的问题。
generate_ego_vehicle_actions：生成与主车（ego vehicle）动作相关的问题。
合并和整理问题：

每个子方法返回的结果被合并到一个大的问题集合中，针对每一类对象（例如动态车辆、道路布局、停车标志等）生成了多个问答对。
为了避免重复，特别是对于相同类型的对象，代码通过对对象进行分组、计数和排序，生成有条理的问题。
生成“重要物体”问题：

代码创建了一个关于“场景中重要物体”的问题，涵盖了场景中的关键对象（例如车辆、行人等），并通过合并和计数避免重复。
如果场景中没有重要物体，答案将是“场景中没有重要物体”，如果只有一个物体，则给出类似“场景中的重要物体是X”的答案，多个物体则是“场景中的重要物体是X、Y和Z”。
生成总的问答集合：

生成的所有问题和答案被整合到一个字典中，按照不同类别（如动态车辆、道路布局等）组织。
返回值包括：
combined_qas：包含每个类别的问题和答案。
num_questions：生成的总问题数量。
num_objects：场景中的对象数量。
num_questions_per_category：每个类别的问答对数量。
key_object_infos：场景中关键对象的相关信息。

#### 笔记

line 316:

```python
def should_consider_vehicle(self, vehicle):
        """
        True, if it's visible in the image and neither of the following applies
        False, if vehicle is not bicycle and the number of points on it are below a threshold
        False, if the vehicle is behind the ego vehicle
        False, if it's a parking vehicle, that does not cut in
        """
```

这里把后面的车都标成“不重要”了。但是变道的时候肯定要看后面的车啊！有点不妙。


#### anno

boxes: 

```json
"class": "ego_car",
        "extent": [
            2.44619083404541,
            0.9183566570281982,
            0.7451388239860535
        ],
        "position": [
            0.0,
            0.0,
            0.0
        ],
        "yaw": 0.0,
        "num_points": -1,
        "distance": -1,
        "speed": 2.5003206711425264e-05,
        "brake": 0.0,
        "id": 197,
        "matrix": [
            [
                -0.9999999403953552,
                -0.000326306966599077,
                2.7425779990153387e-05,
                77.37937927246094
            ],
            [
                0.00032630760688334703,
                -0.9999999403953552,
                2.342686457268428e-05,
                -2.0416219234466553
            ],
            [
                2.7418134777690284e-05,
                2.3435814000549726e-05,
                1.0,
                -0.010334701277315617
            ],
            [
                0.0,
                0.0,
                0.0,
                1.0
            ]
        ]
    },
    {
        "class": "car",
        "color_rgb": [
            0,
            0,
            0
        ],
        "color_name": "black",
        "next_action": "LaneFollow",
        "vehicle_cuts_in": false,
        "road_id": 3,
        "lane_id": -1,
        "lane_type": 2,
        "lane_type_str": "Driving",
        "is_in_junction": false,
        "junction_id": -1,
        "distance_to_junction": null,
        "next_junction_id": -1,
        "next_road_ids": [
            13
        ],
        "next_next_road_ids": [
            15
        ],
        "same_road_as_ego": true,
        "same_direction_as_ego": true,
        "lane_relative_to_ego": 0,
        "light_state": [],
        "traffic_light_state": "None",
        "is_at_traffic_light": false,
        "base_type": "car",
        "role_name": "background",
        "number_of_wheels": "4",
        "type_id": "vehicle.lincoln.mkz_2020",
        "extent": [
            2.44619083404541,
            0.9183566570281982,
            0.7451388239860535
        ],
        "position": [
            15.000020245640286,
            8.386989062361397e-06,
            0.016791981414935965
        ],
        "yaw": 2.663161087212984e-07,
        "num_points": 55,
        "distance": 15.000029644648329,
        "speed": 9.55122838783992e-09,
        "brake": 0.0,
        "steer": 0.01591918058693409,
        "throttle": 0.8500000238418579,
        "id": 216,
        "matrix": [
            [
                -0.9999999403953552,
                -0.00032606918830424547,
                0.0,
                62.37936019897461
            ],
            [
                0.00032606918830424547,
                -0.9999999403953552,
                -0.0,
                -2.0367352962493896
            ],
            [
                0.0,
                -0.0,
                1.0,
                0.006868552882224321
            ],
            [
                0.0,
                0.0,
                0.0,
                1.0
            ]
        ]
    },
```

measurements:

```json
{
    "pos_global": [
        77.37937927246094,
        -2.0416219234466553
    ],
    "theta": 3.141266508898463,
    "speed": 0.032888881862163544,
    "target_speed": 1.031720178842064,
    "speed_limit": 27.77777777777778,
    "target_point": [
        79.46140851141332,
        -126.97674944276457
    ],
    "target_point_next": [
        79.46693809562235,
        -304.97540670337855
    ],
    "command": 4,
    "next_command": 4,
    "aim_wp": [
        4.260015484342587,
        3.115656038245795e-6
    ],
    "route": [
        [
            4.360015489653725,
            3.1383065634844193e-6
        ],
        [
            5.3600155427650344,
            3.3648118140663322e-6
        ],
        [
            6.360015595876343,
            3.591317064648245e-6
        ],
        [
            7.360015648987652,
            3.817822315230375e-6
        ],
        [
            8.360015702098961,
            4.044327565812071e-6
        ],
        [
            9.36001575521027,
            4.270832816394201e-6
        ],
        [
            10.36001580832158,
            4.497338066975897e-6
        ],
        [
            11.360015861432887,
            4.723843317558027e-6
        ],
        [
            12.360015914544196,
            4.950348568139723e-6
        ],
        [
            13.360018714237386,
            5.177749601853986e-6
        ],
        [
            14.360023650160938,
            5.4058473557844905e-6
        ],
        [
            15.360023703272248,
            5.632352606367054e-6
        ],
        [
            16.360022383092616,
            5.8584099653829005e-6
        ],
        [
            17.36002136808874,
            6.084566855855608e-6
        ],
        [
            18.360023862606173,
            6.311868358112142e-6
        ],
        [
            19.36002254242654,
            6.537925717127989e-6
        ],
        [
            20.36002015413173,
            6.763634716035714e-6
        ],
        [
            21.360020207243036,
            6.990139966618278e-6
        ],
        [
            22.360021633617293,
            7.302923792535353e-6
        ],
        [
            23.36002412808496,
            7.682813177443092e-6
        ]
    ],
    "route_original": [
        [
            4.360015489653725,
            3.1383065634844193e-6
        ],
        [
            5.3600155427650344,
            3.3648118140663322e-6
        ],
        [
            6.360015595876343,
            3.591317064648245e-6
        ],
        [
            7.360015648987652,
            3.817822315230375e-6
        ],
        [
            8.360015702098961,
            4.044327565812071e-6
        ],
        [
            9.36001575521027,
            4.270832816394201e-6
        ],
        [
            10.36001580832158,
            4.497338066975897e-6
        ],
        [
            11.360015861432887,
            4.723843317558027e-6
        ],
        [
            12.360015914544196,
            4.950348568139723e-6
        ],
        [
            13.360018714237386,
            5.177749601853986e-6
        ],
        [
            14.360023650160938,
            5.4058473557844905e-6
        ],
        [
            15.360023703272248,
            5.632352606367054e-6
        ],
        [
            16.360022383092616,
            5.8584099653829005e-6
        ],
        [
            17.36002136808874,
            6.084566855855608e-6
        ],
        [
            18.360023862606173,
            6.311868358112142e-6
        ],
        [
            19.36002254242654,
            6.537925717127989e-6
        ],
        [
            20.36002015413173,
            6.763634716035714e-6
        ],
        [
            21.360020207243036,
            6.990139966618278e-6
        ],
        [
            22.360021633617293,
            7.302923792535353e-6
        ],
        [
            23.36002412808496,
            7.682813177443092e-6
        ]
    ],
    "changed_route": false,
    "speed_reduced_by_obj_type": "vehicle.lincoln.mkz_2020",
    "speed_reduced_by_obj_id": 216,
    "speed_reduced_by_obj_distance": 15.000029563903809,
    "steer": 0.0,
    "throttle": 1.0,
    "brake": false,
    "control_brake": false,
    "junction": false,
    "vehicle_hazard": false,
    "vehicle_affecting_id": null,
    "light_hazard": false,
    "walker_hazard": false,
    "walker_affecting_id": null,
    "stop_sign_hazard": false,
    "stop_sign_close": false,
    "walker_close": false,
    "walker_close_id": null,
    "angle": 4.6560587517429187e-7,
    "augmentation_translation": 0.3786207724011643,
    "augmentation_rotation": -2.1941769877631923,
    "ego_matrix": [
        [
            -0.9999999403953552,
            -0.000326306966599077,
            2.7425779990153387e-5,
            77.37937927246094
        ],
        [
            0.00032630760688334703,
            -0.9999999403953552,
            2.342686457268428e-5,
            -2.0416219234466553
        ],
        [
            2.7418134777690284e-5,
            2.3435814000549726e-5,
            1.0,
            -0.010334701277315617
        ],
        [
            0.0,
            0.0,
            0.0,
            1.0
        ]
    ]

}
```
