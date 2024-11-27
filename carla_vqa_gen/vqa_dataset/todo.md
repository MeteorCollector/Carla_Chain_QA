1. 除了 stop sign 其他的 sign 没有考虑

2. 其他 vehicle 的 steer 在 pdm 里是有的，这个要从未来 gt 获得了（已完成）

3. 注意 object key 会根据 bbox2d 来获取，现在 bbox2d 用的是浮点值，可能获取不到，应该改成整型。

4. 这些 relevent_object 的计算：

	```
	if relevant_objects:
		            if 'Accident' in scenario_name: 
		                object_tags = [k for k, v in key_object_infos.items() if 'police' in v['Visual_description']]      
		            elif 'HazardAtSideLane' in scenario_name: 
		                object_tags = [k for k, v in key_object_infos.items() if 'bicycle' in v['Visual_description']]    
		            elif 'ParkedObstacle' in scenario_name:
	```

	有一些应该并不可见，因为 `generate_2d_box_from_projected_points` 会报错，说明 `projected points` 并不存在。

	不过好像是 line 741 报错：

	```python
	def get_key_of_key_object(self, key_object_infos, object_dict=None):
		if object_dict is not None:
		    projected_points, _ = project_all_corners(object_dict, self.CAMERA_MATRIX, self.WORLD2CAM_FRONT)
		    if projected_points is not None and len(projected_points) > 0:
		        two_d_box = self.generate_2d_box_from_projected_points(projected_points)
		        keys = [k for k, v in key_object_infos.items() if two_d_box==v['2d_bbox']]

		        return keys
		
		return []
	```
	
5. lane_change 值的计算，好像是允许变的道？ 对 这个没错

6. Is there an obstacle on the current road? Accident_Town03_Route101_Weather23 就识别不到

7. QA 里面，为什么没有提到颜色？ 已经修复

8. cutting into the lane of the ego vehicle 判定有点宽

9. 根本没有 `behaviour` 问题类，要不要用 LLM 打标
