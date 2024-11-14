import carla
import math
import numpy as np

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

def calculate_distance(location1, location2):
    """
        Calculate Euclidean distance between two locations.
    """
    return math.sqrt((location1.x - location2.x) ** 2 +
                     (location1.y - location2.y) ** 2 +
                     (location1.z - location2.z) ** 2)

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

def is_vehicle_in_camera(camera, vehicle):
    """
    Check if the vehicle is in the view of camera
    :param camera: Camera's params in anno
    :param vehicle: Vehicle's params in anno
    :return: True - in the view; False - not
    """

    camera_location = np.array(camera['location'])
    camera_rotation = camera['rotation']
    intrinsic_matrix = np.array(camera['intrinsic'])
    world2cam_matrix = np.array(camera['world2cam'])
    image_width = camera['image_size_x']
    image_height = camera['image_size_y']
    fov = camera['fov']
    
    vehicle_location = np.array(vehicle['location'])
    vehicle_world_corners = np.array(vehicle['world_cord'])

    # translate vehicle's coordinate from world coord to camera coord
    vehicle_cam_location = world2cam_matrix @ np.append(vehicle_location, 1)
    
    # if z <= 0, it is behind the camera
    if vehicle_cam_location[2] <= 0:
        return False
    
    # check all vetices of the bounding box
    for corner in vehicle_world_corners:
        corner_cam_location = world2cam_matrix @ np.append(corner, 1)
        if corner_cam_location[2] <= 0:
            continue
        
        uv = intrinsic_matrix @ (corner_cam_location[:3] / corner_cam_location[2])
        u, v = uv[0], uv[1]
        if 0 <= u < image_width and 0 <= v < image_height:
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
        "lane_type_str": str(waypoint.lane_type),
        "shoulder_left": shoulder_left,
        "parking_left": parking_left,
        "sidewalk_left": sidewalk_left,
        "bikelane_left": bikelane_left,
        "shoulder_right": shoulder_right,
        "parking_right": parking_right,
        "sidewalk_right": sidewalk_right,
        "bikelane_right": bikelane_right
    }
    print(lane_info)
    return lane_info


import carla
import math

def is_vehicle_changing_lane(vehicle_data, map):
    """
    Judge whether the car is changing its lane
    
    Params:
        vehicle_data (dict): from measurements
        carla_map (carla.Map): CARLA's map object
    """
    
    x, y = vehicle_data['x'], vehicle_data['y']
    theta = vehicle_data['theta']
    
    vehicle_location = carla.Location(x=x, y=y)
    waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if not waypoint:
        return False
    
    lane_yaw = waypoint.transform.rotation.yaw
    lane_direction = math.radians(lane_yaw)
    
    vehicle_direction = math.radians(theta)
    
    angle_diff = abs(vehicle_direction - lane_direction)
    angle_diff = min(angle_diff, 2 * math.pi - angle_diff)
    
    # if the car is more than 10 degrees off the lane
    angle_threshold = math.radians(10)
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

import math

def is_changing_lane_due_to_obstacle(vehicle_data, map, bbox_list):
    """
    Judge whether the car is changing its lane due to obstacles
    
    Param:
        vehicle_data (dict): from measurements
        carla_map (carla.Map): CARLA map
        bbox_list (list): bbox measurement list
    """

    if is_vehicle_changing_lane(vehicle_data, map) is False:
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
