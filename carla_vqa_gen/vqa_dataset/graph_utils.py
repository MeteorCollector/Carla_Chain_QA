import numpy as np
import cv2

def build_projection_matrix(w, h, fov):
    """
    Build a projection matrix based on image dimensions and field of view.
    
    Args:
        w (int): Image width
        h (int): Image height
        fov (float): Field of view in degrees
    
    Returns:
        np.ndarray: 3x3 projection matrix
    """
    focal = w / (2.0 * np.tan(np.radians(fov / 2)))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def project_center_corners(obj, K):
    """
    Project the center corners of an object onto the image plane.
    
    Args:
        obj (dict): Object dictionary containing position, extent, and yaw
        K (np.ndarray): Projection matrix
    
    Returns:
        np.ndarray: 2D array of projected corner points
    """
    pos = obj['position']
    if 'extent' not in obj:
        extent = [0.15,0.15,0.15]
    else:
        extent = obj['extent']
    if 'yaw' not in obj:
        yaw = 0
    else:
        yaw = -obj['yaw']
        
    # get bbox corners coordinates
    corners = np.array([[-extent[0], 0, 0.75],
                        [extent[0], 0, 0.75]])

    # rotate bbox
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                [np.sin(yaw), np.cos(yaw), 0],
                                [0, 0, 1]])
    corners = corners @ rotation_matrix.T

    # translate bbox
    corners = corners + np.array(pos)
    all_points_2d = []
    for corner in  corners:
        pos_3d = np.array([corner[1], -corner[2], corner[0]])
        rvec = np.zeros((3, 1), np.float32) 
        tvec = np.array([[0.0, 2.0, 1.5]], np.float32)
        # Define the distortion coefficients 
        dist_coeffs = np.zeros((5, 1), np.float32) 
        points_2d, _ = cv2.projectPoints(pos_3d, 
                            rvec, tvec, 
                            K, 
                            dist_coeffs)
        all_points_2d.append(points_2d[0][0])
        
    return np.array(all_points_2d)

def project_all_corners(obj, K, extrinsics):
    """
    Project all corners of a 3D bounding box onto the image plane, considering camera extrinsics.
    
    Args:
        obj (dict): Object dictionary containing position, extent, and optional world_cord.
        K (np.ndarray): Intrinsic camera matrix (3x3).
        extrinsics (np.ndarray): Camera extrinsics (4x4).
    
    Returns:
        tuple: (np.ndarray of 2D projected points, np.ndarray of 3D corner points in camera coordinates)
    """
    K = np.array(K)
    extrinsics = np.array(extrinsics)
    
    # 从相机外参中提取相机在世界坐标系中的 yaw
    R = extrinsics[:3, :3]  # 提取旋转矩阵
    camera_yaw = np.arctan2(-R[2, 0], R[2, 2])  # 提取相机的 yaw

    # 获取 bbox 顶点的世界坐标
    if 'world_cord' in obj and obj['world_cord']:
        world_corners = np.array(obj['world_cord'])
    else:
        extent = obj.get('extent', [0.15, 0.15, 0.15])
        position = np.array(obj['center'])
        corners = np.array([
            [-extent[0], -extent[1], 0],  # bottom left back
            [extent[0], -extent[1], 0],   # bottom right back
            [extent[0], extent[1], 0],    # bottom right front
            [-extent[0], extent[1], 0],   # bottom left front
            [-extent[0], -extent[1], 2 * extent[2]],   # top left back
            [extent[0], -extent[1], 2 * extent[2]],    # top right back
            [extent[0], extent[1], 2 * extent[2]],     # top right front
            [-extent[0], extent[1], 2 * extent[2]]     # top left front
        ])
        
        # 物体在世界坐标系中的 yaw
        world_yaw = obj.get('rotation', [0, 0, 0])[2]

        # 计算物体在相机坐标系中的 yaw
        yaw_camera = world_yaw - camera_yaw
        
        # 旋转 bbox
        rotation_matrix = np.array([
            [np.cos(yaw_camera), -np.sin(yaw_camera), 0],
            [np.sin(yaw_camera), np.cos(yaw_camera), 0],
            [0, 0, 1]
        ])
        corners = corners @ rotation_matrix.T

        # 平移 bbox
        world_corners = corners + position
    
    # 转换到相机坐标系
    world_corners_h = np.hstack((world_corners, np.ones((world_corners.shape[0], 1))))  # 转为齐次坐标
    camera_corners = (extrinsics @ world_corners_h.T).T[:, :3]  # 转到相机坐标系
    print(camera_corners)
    
    all_points_2d = []
    valid_corners = []
    
    for corner in camera_corners:
        # if corner[2] <= 0:  # 如果点在相机后方，跳过
        #     continue
        
        # 投影到图像平面
        pos_3d = np.array([corner[1], -corner[2], corner[0]])
        if pos_3d[2] <= 0:  # 如果点位于相机后方，跳过
            continue
        
        # 定义相机外参和内参
        rvec = np.zeros((3, 1), np.float32) 
        tvec = np.array([[0.0, 2.0, 1.5]], np.float32)
        dist_coeffs = np.zeros((5, 1), np.float32) 
        
        # 投影点
        points_2d, _ = cv2.projectPoints(
            pos_3d, rvec, tvec, K, dist_coeffs
        )
        all_points_2d.append(points_2d[0][0])
        valid_corners.append(corner)
    
    return np.array(all_points_2d), np.array(valid_corners)


def light_state_to_word(light_state):
    '''
    0: NONE
    All lights off.
    1: Position
    2: LowBeam
    3: HighBeam
    4: Brake
    5: RightBlinker
    6: LeftBlinker
    7: Reverse
    8: Fog
    9: Interior
    10: Special1
    This is reserved for certain vehicles that can have special lights, like a siren.
    11: Special2
    This is reserved for certain vehicles that can have special lights, like a siren.
    12: All
    All lights on.

    '''
    light_state_dict = {0: 'None', 1: 'position light', 2: 'low beam', 3: 'high beam', 4: 'brake light', 5: 'right blinker',
                        6: 'left blinker', 7: 'reverse light', 8: 'fog light', 9: 'interior light', 10: 'emergency lights', 11: 'emergency lights',
                        12: 'All'}
    # add "the" in front of the light state
    light_state = light_state_dict[light_state]
    light_state = 'the ' + light_state
    return light_state

def logical_xor(str1, str2):
    return bool(str1) ^ bool(str2)
    

def a_or_an(word):
    """
    Returns 'a' or 'an' depending on whether the word starts with a vowel or not.
    :param word: string
    :return: 'a' or 'an'
    """
    vowels = ['a', 'e', 'i', 'o', 'u']
    if word[0].lower() in vowels:
        return 'an'
    else:
        return 'a'

def number_to_word(number):
    """
    Returns the number as a word.
    :param number: int
    :return: string
    """
    number_dict = {0: 'zero', 1: 'one', 2: 'two', 3: 'three', 4: 'four', 5: 'five',
                   6: 'six', 7: 'seven', 8: 'eight', 9: 'nine', 10: 'ten'}
    return number_dict[number]