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

首先要做一个 general的：保持自己目前轨迹，会与未来k帧的该物体相撞。但是这样一来前面慢速行驶的车辆算不算？这些只需要减速...？注意如果 lane_change 为 0，就需要占对向车道超车了。TwoWays关键词就是这样。如果只有一条车道，那就只能停车了。

这个应该 check all:

Accident_Town03_Route101_Weather23 识别不到 [已解决]
AccidentTwoWays_Town12_Route1102_Weather10 识别不到 [已解决]
BlockedIntersection_Town03_Route134_Weather3 没有障碍
ConstructionObstacle_Town03_Route60_Weather8 识别不到  [已解决] static.prop.trafficwarning 
ConstructionObstacleTwoWays_Town12_Route1080_Weather14 识别不到 [已解决] trafficwarning 其实在真正的 warning 之前还有 static.prop.warningconstruction，DriveLM 并没有处理，也要弄一下的
ControlLoss_Town04_Route169_Weather13 不算 obstacle, 但是有 static.prop.dirtdebris01
CrossingBicycleFlow_Town12_Route1011_Weather23 过路口的时候 bicycle flow 已经过去了，建搞一个会挡住的情形
DynamicObjectCrossing_Town01_Route1_Weather1 识别不到  [已解决] walker.pedestrian
EnterActorFlow_Town03_Route132_Weather2 同样也是到地方 flow 都走完了
HardBreakRoute_Town01_Route30_Weather3 识别不到 前面车突然停了
HazardAtSideLane_Town03_Route105_Weather22 [已解决] 识别不到 vehicle.bh.crossbike
HazardAtSideLaneTwoWays_Town12_Route1128_Weather10 [已解决] 识别不到 vehicle.bh.crossbike
HighwayCutIn_Town06_Route298_Weather20 没有障碍
HighwayExit_Town06_Route291_Weather5 没有障碍
InterurbanActorFlow_Town06_Route294_Weather8 没有障碍
InterurbanAdvancedActorFlow_Town06_Route301_Weather15 没有障碍
InvadingTurn_Town02_Route95_Weather9 [已解决]
LaneChange_Town06_Route277_Weather9 没有障碍，为什么要变道？
MergerIntoSlowTraffic_Town06_Route317_Weather5 有比较碍事的车辆
NonSignalizedJunctionLeftTurnEnterFlow_Town12_Route1022_Weather8 没有障碍
NonSignalizedJunctionLeftTurn_Town03_Route122_Weather26 没有障碍
OppositeVehicleRunningRedLight_Town03_Route119_Weather12 有一个警车高速闯红横穿过去，需要非常注意
OppositeVehicleTakingPriority_Town03_Route128_Weather23 同上
ParkedObstacle_Town03_Route103_Weather25 [已解决] 识别不到 停着的车
ParkedObstacleTwoWays_Town12_Route1158_Weather14 [已解决] 识别不到 停着的车
ParkingCrossingPedestrian_Town12_Route758_Weather3 [已解决] 识别不到 pedestrian
ParkingCutIn_Town12_Route1300_Weather13 识别不到 车
ParkingExit_Town12_Route1305_Weather18 [已解决] 这个比较复杂，停着的车应该不算是 obstacle,因为 ego 一开始也是 static的，需要想一想。
PedestrianCrossing_Town12_Route1013_Weather25 [已解决] 识别不到 pedestrian
SignalizedJunctionLeftTurnEnterFlow_Town12_Route1019_Weather5 没有障碍
SignalizedJunctionLeftTurn_Town03_Route113_Weather26 没有障碍
SignalizedJunctionRightTurn_Town03_Route118_Weather14 没有障碍
StaticCutIn_Town03_Route109_Weather1 旁边车道的车突然别进来 这是障碍吗
TJunction_Town01_Route90_Weather12 没有障碍
VanillaNonSignalizedTurnEncounterStopsign_Town03_Route143_Weather13 没懂，没看见 stop_sign？
VanillaSignalizedTurnEncounterGreenLight_Town03_Route137_Weather7 没有障碍
VanillaSignalizedTurnEncounterRedLight_Town03_Route140_Weather10 没有障碍
VehicleOpensDoorTwoWays_Town12_Route1196_Weather0 识别不到 [已解决] 开门骑在马路牙子上的车
VehicleTurningRoutePedestrian_Town12_Route1027_Weather13 没懂这个行人在干嘛
VehicleTurningRoute_Town12_Route1026_Weather12 识别不到冲出来的自行车
YieldToEmergencyVehicle_Town03_Route148_Weather18 需要识别后方来车

7. QA 里面，为什么没有提到颜色？ 已经修复

8. cutting into the lane of the ego vehicle 判定有点宽

9. 根本没有 `behaviour` 问题类，要不要用 LLM 打标

10. 没有天气 复杂情况理解等等

11. brake 那里需要好好弄弄，因为 b2d 里面的 twoways 事件不一定要 invade opposite lane，和原版不一样。accident two ways的时候，接近了accident反倒不brake了，不太好，感觉这些的判定就是凡是brake且在情景内，都统一划进躲障碍的原因了。

12. important object 有的时候会重复？DynamicObjectCrossing 和 VehicleOpensDoorTwoWays都有这种情况。

13. DynamicObjectCrossing里面没有bicycle,PedestriansCrossing里面有的没等行人过马路就过去了

13. current speed limit 是怎么答的？
