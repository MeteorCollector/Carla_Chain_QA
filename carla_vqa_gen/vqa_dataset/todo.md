1. 除了 stop sign 其他的 sign 没有考虑

2. 其他 vehicle 的 steer 在 pdm 里是有的，这个要从未来 gt 获得了

```python
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
            # question = f"Where is {other_vehicle_location_description} going?"
            # answer = ''

            # steer = other_vehicle['steer']

            # # Determine trajectory based on steer angle
            # if steer < -0.1:
            #     answer = f"The {other_vehicle_description} is turning left."
            # elif steer < -0.03:
            #     answer = f"The {other_vehicle_description} is turning slightly left."
            # elif steer > 0.1:
            #     answer = f"The {other_vehicle_description} is turning right."
            # elif steer > 0.03:
            #     answer = f"The {other_vehicle_description} is turning slightly right."
            # else:
            #     answer = f"The {other_vehicle_description} is going straight."
```
