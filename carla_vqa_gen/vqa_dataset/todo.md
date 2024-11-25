1. 除了 stop sign 其他的 sign 没有考虑

2. 其他 vehicle 的 steer 在 pdm 里是有的，这个要从未来 gt 获得了（已完成）

3. 注意 object key 会根据 bbox2d 来获取，现在 bbox2d 用的是浮点值，可能获取不到，应该改成整型。
