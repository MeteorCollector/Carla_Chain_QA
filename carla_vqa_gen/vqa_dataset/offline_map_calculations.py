import carla
import math
import numpy as np
import gzip
import os
import json
from shapely.geometry import Polygon, Point

# math utils

def calculate_distance(location1, location2):
    """
    Calculate Euclidean distance between two locations.
    """
    return math.sqrt((location1.x - location2.x) ** 2 +
                     (location1.y - location2.y) ** 2 +
                     (location1.z - location2.z) ** 2)

def rotate_point(point, center, angle):
    """
    Rotate a 2D point around a center by a given angle (in radians).
    """
    x, y = point
    cx, cy = center
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)

    x -= cx
    y -= cy

    x_new = x * cos_theta - y * sin_theta
    y_new = x * sin_theta + y * cos_theta

    x_new += cx
    y_new += cy

    return [x_new, y_new]

def is_point_in_rotated_box(point, box_center, box_extent, box_yaw):
    """
    Check if a 2D point is inside a rotated rectangular box.
    """

    local_point = rotate_point(point, box_center, -box_yaw)
    
    dx, dy = box_extent
    if (
        -dx <= local_point[0] - box_center[0] <= dx and
        -dy <= local_point[1] - box_center[1] <= dy
    ):
        return True
    return False

def get_rotated_vertices(center, extent, yaw):
    """
    Calculate the 2D vertices of a rotated rectangle (collision box).
    """
    cx, cy = center
    hw, hh = extent[0] / 2, extent[1] / 2  # Half-width and half-height

    local_vertices = [
        [-hw, -hh],
        [-hw, hh],
        [hw, hh],
        [hw, -hh],
    ]

    rotated_vertices = []
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    for vx, vy in local_vertices:
        rx = cx + vx * cos_yaw - vy * sin_yaw
        ry = cy + vx * sin_yaw + vy * cos_yaw
        rotated_vertices.append([rx, ry])

    return rotated_vertices

def transform_to_ego_coordinates(world_coordinates, world2ego_matrix):
    """
    Transform a point in world coordinates to ego_vehicle coordinates.

    Params:
        world_coordinates (list or tuple): The (x, y, z) world coordinates.
        world2ego_matrix (list): The 4x4 world-to-ego transformation matrix.

    Returns:
        tuple: The transformed (x', y', z') coordinates in ego_vehicle's coordinate system.
    """

    P_world = np.array(list(world_coordinates) + [1])
    M_world2ego = np.array(world2ego_matrix)

    P_ego_homogeneous = M_world2ego @ P_world

    return tuple(P_ego_homogeneous[:3])

def rgb_to_color_name(rgb_str):
    """
    Convert an RGB string (e.g., '255,0,0') to a natural language color description.

    Params:
        rgb_str (str): RGB value in the format 'R,G,B'.

    Returns:
        str: The corresponding natural language color description.
    """

    # Predefined common colors with their RGB values
    color_mapping = {
        "red": (255, 0, 0),
        "green": (0, 255, 0),
        "dark green": (0, 28, 0),
        "dark green ": (12, 42, 12),
        "blue": (0, 0, 255),
        "blue ": (145, 255, 181),
        "yellow": (255, 255, 0),
        "yellow ": (211, 142, 0),
        "cyan": (0, 255, 255),
        "magenta": (255, 0, 255),
        "black": (0, 0, 0),
        "white": (255, 255, 255),
        "gray": (128, 128, 128),
        "orange": (255, 165, 0),
        "orange ": (215, 88, 0),
        "pink": (255, 192, 203),
        "brown": (165, 42, 42),
        "purple": (128, 0, 128),
    }

    try:
        r, g, b = map(int, rgb_str.split(","))
    except ValueError:
        return "unknown"

    def euclidean_distance(rgb1, rgb2):
        return math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(rgb1, rgb2)))

    closest_color = min(color_mapping, key=lambda color: euclidean_distance((r, g, b), color_mapping[color]))

    return closest_color

# calculation functions

def print_waypoint_info(waypoint):
    """
    Print coordinate, road ID and lane ID of waypoint, just for debug
    """
    location = waypoint.transform.location
    road_id = waypoint.road_id
    lane_id = waypoint.lane_id
    
    print(f"Waypoint coordinates: x={location.x}, y={location.y}, z={location.z}")
    print(f"Route ID: {road_id}")
    print(f"Lane ID: {lane_id}")

def find_first_junction_in_direction(map, ego_location):
    """
    Finds the first junction in the direction of the ego vehicle.
    """
    waypoint = map.get_waypoint(ego_location, project_to_road=True, lane_type=carla.LaneType.Driving)

    while waypoint and not waypoint.is_junction:
        # check whether it is junction or not every 2 meters
        waypoint = waypoint.next(2.0)[0] if waypoint.next(2.0) else None

    if waypoint and waypoint.is_junction:
        junction_location = waypoint.transform.location
        distance_to_junction = calculate_distance(ego_location, junction_location)
        print(f"[debug] first junction of vehicle is at {junction_location}, {distance_to_junction}m away. ego is at {ego_location}") # debug
        return junction_location, distance_to_junction
    else:
        return None, None # represents no junction found
    
def get_speed_limit(scene_data):
    """
    Finds the speed limit of ego
    """
    minimum_speed_limit = float('inf')

    for actor in scene_data:
        if 'traffic_sign' in actor['class']:
            if actor['type_id'].startswith("traffic.speed_limit") and actor['affects_ego']:
                speed_limit = float(int(actor['type_id'].split('.')[-1]))
                if speed_limit < minimum_speed_limit:
                    minimum_speed_limit = speed_limit

    return minimum_speed_limit if minimum_speed_limit != float('inf') else 10000

def project_point_to_camera(intrinsic_matrix, world2cam_matrix, point, image_width, image_height):
    """
    Project point in world coordinate to camera 2d plane

    Returns:
        Tuple ([u, v], is_in_view) - Projection coordinate (u, v), bool indicates whether it is in camera's view
    """
    point_cam_location = world2cam_matrix @ np.append(point, 1)

    uv = intrinsic_matrix @ (point_cam_location[:3] / point_cam_location[2])
    u, v = uv[0], uv[1]

    # z <= 0. point is behind the camera
    if point_cam_location[2] <= 0:
        return [u, v], False

    is_in_view = 0 <= u < image_width and 0 <= v < image_height
    return [u, v], is_in_view

def get_vehicle_projected_corners(camera, vehicle):
    """
    Calculate vehicle's corners in camera's view

    Returns:
        - np.array which conveted from list of list [u, v] - Projection coordinate (u, v)
        - list of bool - Indicates whether it is in camera's view
    """
    intrinsic_matrix = np.array(camera['intrinsic'])
    world2cam_matrix = np.array(camera['world2cam'])
    image_width = camera['image_size_x']
    image_height = camera['image_size_y']
    vehicle_corners = np.array(vehicle['world_cord'])

    projected_corners = []
    in_view_list = []
    for corner in vehicle_corners:
        uv, is_in_view = project_point_to_camera(
            intrinsic_matrix, world2cam_matrix, corner, image_width, image_height
        )
        projected_corners.append(uv)
        in_view_list.append(is_in_view)

    return np.array(projected_corners), in_view_list

def is_vehicle_in_camera(camera, vehicle):
    """
    Check if the vehicle is in the view of camera
    """
    projected_corners, in_view_list = get_vehicle_projected_corners(camera, vehicle)

    for is_in_view in in_view_list:
        if is_in_view:
            return True
    
    # all vertices out of sight
    return False

def is_vehicle_in_junction(map, vehicle_location):
    """
    Check if the vehicle is at junction
    """
    waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if waypoint and waypoint.is_junction:
        return True
    else:
        return False

def get_lane_info(map, vehicle_location):
    """
    Get information keys of current lane
    """
    waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if not waypoint:
        return {}

    lane_direction = waypoint.lane_id
    leftmost_lane = lane_direction
    rightmost_lane = lane_direction
    lane_id_left_most_lane_same_direction = lane_direction
    vehicle_road_id = waypoint.road_id

    num_lanes_same_direction = 1
    num_lanes_opposite_direction = 0

    shoulder_left, parking_left, sidewalk_left, bikelane_left = False, False, False, False
    shoulder_right, parking_right, sidewalk_right, bikelane_right = False, False, False, False

    left_waypoint = waypoint.get_left_lane()
    while left_waypoint:
        if left_waypoint.lane_id == leftmost_lane or left_waypoint.road_id != vehicle_road_id:
            break

        lane_type = left_waypoint.lane_type
        if lane_type == carla.LaneType.Shoulder and left_waypoint.lane_width > 1.0:
            shoulder_left = True
        if lane_type == carla.LaneType.Parking:
            parking_left = True
        if lane_type == carla.LaneType.Sidewalk:
            sidewalk_left = True
        if lane_type == carla.LaneType.Biking:
            bikelane_left = True

        if left_waypoint.lane_id * lane_direction > 0:
            if lane_type == carla.LaneType.Driving:
                num_lanes_same_direction += 1
                lane_id_left_most_lane_same_direction = left_waypoint.lane_id
                # print_waypoint_info(left_waypoint)
            
            leftmost_lane = left_waypoint.lane_id
            left_waypoint = left_waypoint.get_left_lane()
        elif left_waypoint.lane_id * lane_direction < 0:
            if lane_type == carla.LaneType.Driving:
                num_lanes_opposite_direction += 1
                # print_waypoint_info(left_waypoint)
            
            leftmost_lane = left_waypoint.lane_id
            left_waypoint = left_waypoint.get_right_lane()
    
    right_waypoint = waypoint.get_right_lane()
    while right_waypoint:
        if right_waypoint.lane_id == rightmost_lane or right_waypoint.road_id != vehicle_road_id:
            break

        lane_type = right_waypoint.lane_type
        if lane_type == carla.LaneType.Shoulder and right_waypoint.lane_width > 1.0:
            shoulder_right = True
        if lane_type == carla.LaneType.Parking:
            parking_right = True
        if lane_type == carla.LaneType.Sidewalk:
            sidewalk_right = True
        if lane_type == carla.LaneType.Biking:
            bikelane_right = True

        if right_waypoint.lane_id * lane_direction > 0:
            if right_waypoint.lane_type == carla.LaneType.Driving:
                num_lanes_same_direction += 1
                # print_waypoint_info(right_waypoint)
            
            rightmost_lane = right_waypoint.lane_id
            right_waypoint = right_waypoint.get_right_lane()
        elif right_waypoint.lane_id * lane_direction < 0:
            if right_waypoint.lane_type == carla.LaneType.Driving:
                num_lanes_opposite_direction += 1
                # print_waypoint_info(right_waypoint)
            
            rightmost_lane = right_waypoint.lane_id
            right_waypoint = right_waypoint.get_left_lane()

    # get ego lane number counted from left to right
    # https://www.asam.net/standards/detail/opendrive/
    # most left should be always the smallest number
        ego_lane_number = abs(waypoint.lane_id - lane_id_left_most_lane_same_direction)
    
    # print(f"[debug] lanes at the same direction: {num_lanes_same_direction}, opposite direction: {num_lanes_opposite_direction}")
        
    lane_info = {
        "num_lanes_same_direction": num_lanes_same_direction,
        "num_lanes_opposite_direction": num_lanes_opposite_direction,
        "ego_lane_number": ego_lane_number,
        'lane_change': waypoint.lane_change,
        'lane_change_str': str(waypoint.lane_change),
        'lane_type': waypoint.lane_type,
        'lane_type_str': str(waypoint.lane_type),
        'left_lane_marking_color': waypoint.left_lane_marking.color,
        'left_lane_marking_color_str': str(waypoint.left_lane_marking.color),
        'left_lane_marking_type': waypoint.left_lane_marking.type,
        'left_lane_marking_type_str': str(waypoint.left_lane_marking.type),
        'right_lane_marking_color': waypoint.right_lane_marking.color,
        'right_lane_marking_color_str': str(waypoint.right_lane_marking.color),
        'right_lane_marking_type': waypoint.right_lane_marking.type,
        'right_lane_marking_type_str': str(waypoint.right_lane_marking.type),
        "shoulder_left": shoulder_left,
        "parking_left": parking_left,
        "sidewalk_left": sidewalk_left,
        "bikelane_left": bikelane_left,
        "shoulder_right": shoulder_right,
        "parking_right": parking_right,
        "sidewalk_right": sidewalk_right,
        "bikelane_right": bikelane_right
    }

    return lane_info

def is_changing_lane(x, y, theta, map):
    """
    Judge whether the object is changing its lane by its position
    """

    vehicle_location = carla.Location(x=x, y=y)
    waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if not waypoint:
        return False
    
    if waypoint.is_junction:
        return False # vehicle must leave its lane at junction, so omit
    
    lane_yaw = waypoint.transform.rotation.yaw
    lane_direction = math.radians(lane_yaw)
    
    vehicle_direction = math.radians(theta)
    
    angle_diff = abs(vehicle_direction - lane_direction)
    angle_diff = min(angle_diff, 2 * math.pi - angle_diff)
    
    # if the car is more than 7 degrees off the lane
    angle_threshold = math.radians(7)
    if angle_diff > angle_threshold:
        return True
    
    # check the distance to neiboring lane, if too close
    left_lane = waypoint.get_left_lane()
    right_lane = waypoint.get_right_lane()
    
    if left_lane and left_lane.lane_type == carla.LaneType.Driving:
        distance_to_left_lane = left_lane.transform.location.distance(vehicle_location)
        if distance_to_left_lane * 0.5 <= waypoint.transform.location.distance(vehicle_location):
            return True
    
    if right_lane and right_lane.lane_type == carla.LaneType.Driving:
        distance_to_right_lane = right_lane.transform.location.distance(vehicle_location)
        if distance_to_right_lane * 0.5 <= waypoint.transform.location.distance(vehicle_location):
            return True

    return False

def is_ego_changing_lane(vehicle_data, map):
    """
    Judge whether the ego vehicle is changing its lane
    
    Params:
        vehicle_data (dict): from measurements
        carla_map (carla.Map): CARLA's map object
    """
    
    x, y = vehicle_data['x'], vehicle_data['y']
    theta = vehicle_data['theta']
    
    return is_changing_lane(x, y, theta, map)

def is_vehicle_changing_lane(vehicle_data, map):
    """
    Judge whether the vehicle is changing its lane
    
    Params:
        vehicle_data (dict): from scene_data (actor format)
        carla_map (carla.Map): CARLA's map object
    """
    
    x, y = vehicle_data['location'][0], vehicle_data['location'][1]
    theta = vehicle_data['rotation'][1] # yaw
    
    return is_changing_lane(x, y, theta, map)

def is_ego_changing_lane_due_to_obstacle(vehicle_data, map, bbox_list):
    """
    Judge whether the car is changing its lane due to obstacles
    
    Params:
        vehicle_data (dict): from measurements
        carla_map (carla.Map): CARLA map
        bbox_list (list): bbox measurement list
    """

    if is_ego_changing_lane(vehicle_data, map) is False:
        return False

    vehicle_location = carla.Location(x=vehicle_data['x'], y=vehicle_data['y'])
    vehicle_theta = vehicle_data['theta']
    waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    lane_direction = waypoint.transform.rotation.yaw
    
    angle_diff = (vehicle_theta - lane_direction) % 360
    if angle_diff > 180:
        angle_diff -= 360
    
    lane_change_direction = None
    angle_threshold = 10.0
    if angle_diff > angle_threshold:
        lane_change_direction = 'right'
    elif angle_diff < -angle_threshold:
        lane_change_direction = 'left'
    else:
        return False

    left_waypoint = waypoint.get_left_lane()
    right_waypoint = waypoint.get_right_lane()
    
    def calculate_distance_to_waypoint(wp):
        return math.sqrt((vehicle_location.x - wp.transform.location.x) ** 2 +
                         (vehicle_location.y - wp.transform.location.y) ** 2)
    
    nearest_lane = None
    nearest_lane_distance = 1000 # max value
    
    if left_waypoint:
        left_distance = calculate_distance_to_waypoint(left_waypoint)
        if left_distance < nearest_lane_distance:
            nearest_lane = left_waypoint
            nearest_lane_distance = left_distance

    if right_waypoint:
        right_distance = calculate_distance_to_waypoint(right_waypoint)
        if right_distance < nearest_lane_distance:
            nearest_lane = right_waypoint
            nearest_lane_distance = right_distance
    
    if nearest_lane is None:
        return False # there is no other lane!
    
    if lane_change_direction is 'left':
        if nearest_lane == left_waypoint:
            target_lane = left_waypoint
            original_lane = waypoint
        else:
            target_lane = waypoint
            original_lane = right_waypoint
    elif lane_change_direction is 'right':
        if nearest_lane == right_waypoint:
            target_lane = right_waypoint
            original_lane = waypoint
        else:
            target_lane = waypoint
            original_lane = left_waypoint
    
    detection_distance = 20.0  # detect obstacle
    angle_threshold = 10.0
    
    for obj in bbox_list:
        if obj['class'] == 'ego_vehicle':
            continue
        
        obj_location = obj['location']
        obj_x, obj_y, obj_z = obj_location[0], obj_location[1], obj_location[2]
        
        vehicle_x = vehicle_data['x']
        vehicle_y = vehicle_data['y']

        distance_to_obj = math.sqrt((obj_x - vehicle_x) ** 2 + (obj_y - vehicle_y) ** 2)
        
        if distance_to_obj > detection_distance:
            continue
        
        angle_to_obj = math.degrees(math.atan2(obj_y - vehicle_y, obj_x - vehicle_x))
        
        relative_angle = (angle_to_obj - vehicle_theta) % 360
        if relative_angle > 180:
            relative_angle -= 360
        
        if abs(relative_angle) < angle_threshold:
            obj_location = carla.Location(x=obj_x, y=obj_y, z=obj_z)
            obj_waypoint = map.get_waypoint(obj_location, project_to_road=True, lane_type=carla.LaneType.Driving)

            if obj_waypoint:
                if (obj_waypoint.road_id == original_lane['road_id'] and
                    obj_waypoint.lane_id == original_lane['lane_id']):
                    return True
    
    return False

def is_vehicle_cutting_in(ego_data, vehicle_data, map):
    """
    Judge whether the car is cutting in ego's lane
    
    Params:
        ego_data (dict): from measurements
        vehicle_data (dict): from scene_data (actor)
        carla_map (carla.Map): CARLA map
    """

    if is_vehicle_changing_lane(vehicle_data, map) is False:
        return False

    vehicle_location = carla.Location(x=vehicle_data['location'][0], y=vehicle_data['location'][1])
    vehicle_theta = vehicle_data['rotation'][1]
    waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    lane_direction = waypoint.transform.rotation.yaw
    
    angle_diff = (vehicle_theta - lane_direction) % 360
    if angle_diff > 180:
        angle_diff -= 360
    
    lane_change_direction = None
    angle_threshold = 10.0
    if angle_diff > angle_threshold:
        lane_change_direction = 'right'
    elif angle_diff < -angle_threshold:
        lane_change_direction = 'left'
    else:
        return False

    left_waypoint = waypoint.get_left_lane()
    right_waypoint = waypoint.get_right_lane()
    
    def calculate_distance_to_waypoint(wp):
        return math.sqrt((vehicle_location.x - wp.transform.location.x) ** 2 +
                         (vehicle_location.y - wp.transform.location.y) ** 2)
    
    nearest_lane = None
    nearest_lane_distance = 1000 # max value
    
    if left_waypoint:
        left_distance = calculate_distance_to_waypoint(left_waypoint)
        if left_distance < nearest_lane_distance:
            nearest_lane = left_waypoint
            nearest_lane_distance = left_distance

    if right_waypoint:
        right_distance = calculate_distance_to_waypoint(right_waypoint)
        if right_distance < nearest_lane_distance:
            nearest_lane = right_waypoint
            nearest_lane_distance = right_distance
    
    if nearest_lane is None:
        return False # there is no other lane!
    
    if lane_change_direction is 'left':
        if nearest_lane == left_waypoint:
            target_lane = left_waypoint
            original_lane = waypoint
        else:
            target_lane = waypoint
            original_lane = right_waypoint
    elif lane_change_direction is 'right':
        if nearest_lane == right_waypoint:
            target_lane = right_waypoint
            original_lane = waypoint
        else:
            target_lane = waypoint
    
    ego_location = carla.Location(x=ego_data['location'][0], y=ego_data['location'][1])
    ego_waypoint = map.get_waypoint(ego_location, project_to_road=True, lane_type=carla.LaneType.Driving)

    return target_lane.road_id == ego_waypoint.road_id and target_lane.lane_id == ego_waypoint.lane_id


def get_acceleration_by_imu(speed, theta, acceleration):
    """
    Returns:
        str: "Accelerating" or
             "Decelerating" or
             "Constant Speed"
    """
    if len(acceleration) < 2:
        raise ValueError("Acceleration vector must have at least two components [ax, ay].")
    
    ax, ay = acceleration[0], acceleration[1]
    
    theta_rad = math.radians(theta)
    acc_in_direction = ax * math.cos(theta_rad) + ay * math.sin(theta_rad)
    
    epsilon = 1e-2
    
    if abs(acc_in_direction) < epsilon:
        return "Constant Speed"
    elif acc_in_direction * speed > 0:
        return "Accelerating"
    else:
        return "Decelerating"

def load_measurement(file_path):
    with gzip.open(file_path, 'rt', encoding='utf-8') as f:
        return json.load(f)

def get_acceleration_by_future(path, k):
    """
    Calculate acceleration trend by future k measurements
    
    Returns:
        str: "Accelerate", "Decelerate", "Constant", "Ambiguous"
    """

    dir_name, file_name = os.path.split(path)
    base_name, _ = os.path.splitext(file_name)
    base_name, _ = os.path.splitext(base_name)
    ext = ".json.gz"
    initial_index = int(base_name)

    speeds = []
    for i in range(k + 1):
        file_index = f"{initial_index + i:05d}"
        file_path = os.path.join(dir_name, f"{file_index}{ext}")

        if not os.path.exists(file_path):
            break
        
        data = load_measurement(file_path)
        speeds.append(data['speed'])

    if len(speeds) < 2:
        print("[debug] vehicle status is Ambiguous")  # debug
        return "Ambiguous"
    
    acceleration_trend = speeds[-1] - speeds[0]
    if acceleration_trend > 0:
        print("[debug] vehicle status is Accelerate")  # debug
        return "Accelerate"
    elif acceleration_trend < 0:
        print("[debug] vehicle status is Decelerate")  # debug
        return "Decelerate"
    else:
        print("[debug] vehicle status is Constant")  # debug
        return "Constant"


def get_affect_flags(bbox_data):
    """
    Analyze traffic and get affect flags

    Params:
        bbox_data (list): bounding_box data from measurements
    
    Returns:
        dict: flags
    """

    flags = {
        "affected_by_red_light": False,
        "affected_by_yellow_light": False,
        "affected_by_stop_sign": False
    }

    # for traffic_light
    # 0 - Red; 1 - Yellow; 2 - Green; 3 - Off; 4 - Unknown;
    for actor in bbox_data:
        if "traffic_light" in actor["class"]:
            if actor["state"] == 0 and actor["affects_ego"] is True:
                flags["affected_by_red_light"] = True
            if actor["state"] == 1 and actor["affects_ego"] is True:
                flags["affected_by_yellow_light"] = True

        if "traffic_sign" in actor["class"]:
            if "stop" in actor["type_id"].lower() and actor["affects_ego"] is True:
                flags["affected_by_stop_sign"] = True

    return flags

def get_walker_hazard_with_prediction(bbox_data, expansion=1.2, prediction_time=20.0, delta_time=0.1):
    """
    Determine if any walker will enter the ego_vehicle's collision box within the prediction time
    and return a list of IDs of walkers with collision risks.

    Params:
        bbox_data (list): List of dictionaries containing data for all actors.
        expansion (float): Scaling factor for the collision box, default is 1.2 (expand by 20%).
        prediction_time (float): Time window (in seconds) for collision prediction, default is 20.
        delta_time (float): Time step for prediction intervals, default is 0.1 seconds.

    Returns:
        list: List of walker IDs with collision risks.
    """

    hazardous_walkers = []

    ego_vehicle = next((actor for actor in bbox_data if actor["class"] == "ego_vehicle"), None)
    if not ego_vehicle:
        return hazardous_walkers 
    
    ego_center = ego_vehicle["location"][:2]
    ego_extent = ego_vehicle["extent"]
    ego_speed = ego_vehicle.get("speed", 0)  # Default speed is 0
    ego_yaw = math.radians(ego_vehicle["rotation"][2])  # Extract yaw angle and convert to radians

    ego_expanded_extent = [ego_extent[0] * expansion, ego_extent[1] * expansion]

    ego_future_position = [
        ego_center[0] + ego_speed * prediction_time * math.cos(ego_yaw),
        ego_center[1] + ego_speed * prediction_time * math.sin(ego_yaw),
    ]

    for actor in bbox_data:
        if actor["class"] == "walker":
            walker_id = actor["id"]
            walker_position = actor["location"][:2]  # Extract x, y coordinates
            walker_speed = actor.get("speed", 0)
            walker_yaw = math.radians(actor["rotation"][2])  # yaw

            # Simulate motion over time and check for collision
            time_elapsed = 0
            collision_detected = False
            while time_elapsed <= prediction_time:
                ego_future_position = [
                    ego_center[0] + ego_speed * time_elapsed * math.cos(ego_yaw),
                    ego_center[1] + ego_speed * time_elapsed * math.sin(ego_yaw),
                ]
                walker_future_position = [
                    walker_position[0] + walker_speed * time_elapsed * math.cos(walker_yaw),
                    walker_position[1] + walker_speed * time_elapsed * math.sin(walker_yaw),
                ]

                if is_point_in_rotated_box(walker_future_position, ego_future_position, ego_expanded_extent, ego_yaw):
                    collision_detected = True
                    break

                time_elapsed += delta_time

            if collision_detected:
                hazardous_walkers.append(walker_id)

    return hazardous_walkers

def get_all_hazard_with_prediction_sorted(bbox_data, expansion=1.2, prediction_time=20.0, delta_time=0.1):
    """
    Predict if the ego_vehicle will collide with any actor within the prediction time,
    using detailed collision box checks for all vertices, and sort the result by distance.

    Params:
        bbox_data (list): List of dictionaries containing data for all actors.
        expansion (float): Scaling factor for the ego_vehicle's collision box, default is 1.2 (expand by 20%).
        prediction_time (float): Time window (in seconds) for collision prediction, default is 20.
        delta_time (float): Time step for prediction intervals, default is 0.1 seconds.

    Returns:
        list: List of dictionaries with 'id' and 'distance' keys for hazardous actors, sorted by distance.
    """

    # Find the ego_vehicle data
    ego_vehicle = next((actor for actor in bbox_data if actor["class"] == "ego_vehicle"), None)
    if not ego_vehicle:
        return []

    # Extract ego_vehicle information
    ego_center = ego_vehicle["location"][:2]
    ego_extent = [ego_vehicle["extent"][0] * expansion, ego_vehicle["extent"][1] * expansion]
    ego_speed = ego_vehicle.get("speed", 0)
    ego_yaw = math.radians(ego_vehicle["rotation"][2])

    hazardous_actors = []

    # Iterate through all other actors
    for actor in bbox_data:
        if actor["class"] == "ego_vehicle":
            continue

        actor_center = actor["location"][:2]
        actor_extent = [actor["extent"][0], actor["extent"][1]]
        actor_speed = actor.get("speed", 0)
        actor_yaw = math.radians(actor["rotation"][2])

        time_elapsed = 0
        collision_detected = False
        while time_elapsed <= prediction_time:
            ego_future_position = [
                ego_center[0] + ego_speed * time_elapsed * math.cos(ego_yaw),
                ego_center[1] + ego_speed * time_elapsed * math.sin(ego_yaw),
            ]
            actor_future_position = [
                actor_center[0] + actor_speed * time_elapsed * math.cos(actor_yaw),
                actor_center[1] + actor_speed * time_elapsed * math.sin(actor_yaw),
            ]

            ego_vertices = get_rotated_vertices(ego_future_position, ego_extent, ego_yaw)
            actor_vertices = get_rotated_vertices(actor_future_position, actor_extent, actor_yaw)

            if any(is_point_in_rotated_box(vertex, actor_future_position, actor_extent, actor_yaw) for vertex in ego_vertices) or \
               any(is_point_in_rotated_box(vertex, ego_future_position, ego_extent, ego_yaw) for vertex in actor_vertices):
                collision_detected = True
                break

            time_elapsed += delta_time

        if collision_detected:
            hazardous_actors.append(actor)

    # Sort the actors by distance
    hazardous_actors.sort(key=lambda x: x["distance"])

    return hazardous_actors

def vehicle_obstacle_detected(ego_data, vehicle_list, carla_map, max_distance=30.0, junction_threshold=3.0):
    """
    Method to check if there is a vehicle in front of the agent blocking its path.
    Migrated from pdm_lite, but massive modification
        :param ego_vehicle (actor dict): ego vehicle object information.
        :param vehicle_list (list of actor dict): list contatining vehicle object information.
        :param carla_map (carla.map): current map.
        :param max_distance: max freespace to check for obstacles.
                If None, the base threshold value is used
        :param junction_threshold: when vehicle waypoint goes into junction, how much further is detected.
                If None, the base threshold value is used
        :return Tuple (bool: detected or not, dict(actor): nearest obstacle vehicle at front, if exist)
    """

    def compute_distance(loc1, loc2):
        return math.sqrt((loc1[0] - loc2[0])**2 + (loc1[1] - loc2[1])**2)

    def get_forward_vector(rotation):
        theta = math.radians(rotation[1])
        return [math.cos(theta), math.sin(theta)]

    ego_location = ego_data['location']
    ego_rotation = ego_data['rotation']
    ego_extent = ego_data['extent']
    ego_forward_vector = get_forward_vector(ego_rotation)

    ego_wpt = carla_map.get_waypoint(
        carla.Location(x=ego_location[0], y=ego_location[1], z=ego_location[2]),
        lane_type=carla.LaneType.Any
    )

    search_distance = max_distance

    def get_route_polygon():
        extent_y = ego_extent[1]
        r_ext = extent_y
        l_ext = -extent_y

        right_vector = [-ego_forward_vector[1], ego_forward_vector[0]]

        # two vertices start from vehicle's current location
        p1 = (ego_location[0] + r_ext * right_vector[0], ego_location[1] + r_ext * right_vector[1])
        p2 = (ego_location[0] + l_ext * right_vector[0], ego_location[1] + l_ext * right_vector[1])

        forward_points = []
        total_distance = 0
        current_wpt = ego_wpt
        last_wpt = None
        if ego_wpt.is_junction:
            last_wpt = ego_wpt
        else:
            while total_distance <= search_distance:
                next_wpt = current_wpt.next(2.0)[0]
                total_distance += 2.0
                if not next_wpt:
                    break

                if next_wpt.is_junction:
                    last_wpt = next_wpt
                    break

                forward_location = next_wpt.transform.location
                forward_vector = next_wpt.transform.get_right_vector()
                forward_p1 = (
                    forward_location.x + r_ext * forward_vector.x,
                    forward_location.y + r_ext * forward_vector.y
                )
                forward_p2 = (
                    forward_location.x + l_ext * forward_vector.x,
                    forward_location.y + l_ext * forward_vector.y
                )

                forward_points.append(forward_p1)
                forward_points.append(forward_p2)
                current_wpt = next_wpt

        route_bb = [p1, p2] + forward_points

        if len(route_bb) < 3:  # can't form a polygon
            return None, None
        return Polygon(route_bb), last_wpt

    route_polygon, last_wpt = get_route_polygon()

    for target_vehicle in vehicle_list:
        if target_vehicle['id'] == ego_data['id']:
            continue

        target_location = target_vehicle['location']
        target_bounding_box = target_vehicle['world_cord']

        if compute_distance(ego_location, target_location) > search_distance:
            continue

        target_polygon = Polygon([(v[0], v[1]) for v in target_bounding_box])

        if route_polygon and route_polygon.intersects(target_polygon):
            return True, target_vehicle

        if last_wpt:
            print("[debug] detect stopped before junction") # debug
            target_forward_vector = get_forward_vector(target_vehicle['rotation'])
            last_forward_vector = last_wpt.transform.get_front_vector()
            dot_product = last_forward_vector.x * target_forward_vector[0] + last_forward_vector.y * target_forward_vector[1]
            angle = math.degrees(math.acos(dot_product))
            distance = compute_distance([last_wpt.transform.x, last_wpt.transform.y], target_location)

            if distance < junction_threshold and abs(angle) <= 20:
                return True, target_vehicle

    return False, None