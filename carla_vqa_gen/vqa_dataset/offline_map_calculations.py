import carla
import math
import numpy as np

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

