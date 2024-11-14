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

def get_num_lanes(map, vehicle_location):
    """
    Get the number of lanes of vehicle.
    """
    waypoint = map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if not waypoint:
        return 0

    lane_direction = waypoint.lane_id
    leftmost_lane = lane_direction
    rightmost_lane = lane_direction
    vehicle_road_id = waypoint.road_id

    num_lanes_same_direction = 1
    num_lanes_opposite_direction = 0

    left_waypoint = waypoint.get_left_lane()
    while left_waypoint:
        if left_waypoint.lane_id == leftmost_lane or left_waypoint.road_id != vehicle_road_id:
            break
        if left_waypoint.lane_id * lane_direction > 0:
            if left_waypoint.lane_type == carla.LaneType.Driving:
                num_lanes_same_direction += 1
                # print_waypoint_info(left_waypoint)
            
            leftmost_lane = left_waypoint.lane_id
            left_waypoint = left_waypoint.get_left_lane()
        elif left_waypoint.lane_id * lane_direction < 0:
            if left_waypoint.lane_type == carla.LaneType.Driving:
                num_lanes_opposite_direction += 1
                # print_waypoint_info(left_waypoint)
            
            leftmost_lane = left_waypoint.lane_id
            left_waypoint = left_waypoint.get_right_lane()
    
    right_waypoint = waypoint.get_right_lane()
    while right_waypoint:
        if right_waypoint.lane_id == rightmost_lane or right_waypoint.road_id != vehicle_road_id:
            break
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
    
    # print(f"[debug] lanes at the same direction: {num_lanes_same_direction}, opposite direction: {num_lanes_opposite_direction}")
    return num_lanes_same_direction, num_lanes_opposite_direction

