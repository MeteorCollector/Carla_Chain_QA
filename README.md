# Carla to B2D
Generate dataset in nuscenes ([Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive) version) format using Carla! You can config your dataset in configs.

## 项目结构：

`dataset_generator` 里面放着我们改进的 Carla 的 nuScenes 格式数据集采集代码；剩下的部分是 `transfuser` 里面的一些代码。`legacy` 里面是之前开发时候遗留下来的代码，不必理会。

Real useful codes are placed under `dataset_generator` folder, rest of them are legacy codes from `transfuser` and prototype projects for development, just ignore them.

## 安装 Installation

目前我们用的是 `dataset_generator`，我们目前只配置它的环境。这个仓库使用的是最新版本的 `Carla`，因此请先配置 `Carla 0.9.15`（经过我的考察，`Carla >= 0.9.12`即可，但是我现在用的是最新版）：

We only set up environment for `dataset_generator`. Please install `Carla` beforehands, version 0.9.15 is the best choice in practice.

首先配置 `python 3.8` 版本的虚拟环境：

Firstly, create virtual environment with `python 3.8`

```bash
conda create --name NewCarla python=3.8 -y
conda activate NewCarla
```

配置包：

Install pygame and numpy

```bash
pip install --user pygame numpy
```

下载 Carla：

Download Carla, please refer to [Official Tutorial](https://carla.readthedocs.io/en/0.9.15/start_quickstart/)

请参考[官方教程](https://carla.readthedocs.io/en/0.9.15/start_quickstart/)，推荐的下载方式是github release。下载完成后，将文件转移到你想安装的目录，进行

```bash
tar -xf CARLA_0.9.15.tar.gz
tar -xf AdditionalMaps_0.9.15.tar.gz
```

(当然你的压缩包可能和我的有出入，版本对就好)

这样 Carla 就安装好了。

如果 `Carla` 报错 `X Error of failed request: BadDrawable / BadMatch`，大概率是你的 `vulkan` 配置有问题，请参考：

[My Blog](https://meteorcollector.github.io/2024/01/carla-setup/)

[Github Issue](https://github.com/carla-simulator/carla/issues/2232)

本项目还暂时没有弄 `requirements.txt`，报错少包就一直安装吧，以后可能更新一个。

Currently we don't have a `requirements.txt` to manage dependencies, please help yourself. We will implement it soon, hopefully.

## 配置项目设置 Set up configs

要让数据采集成功跑起来，还需要修改一些地方。

You must modify some configure information to get start.

首先是 `dataset_generator/generate_datasets.sh`，请务必将 `CARLA_ROOT` 配置成你的 `carla` 所在目录，并且请确保引用的 `.egg` 文件与你下载版本的 `.egg` 文件名称相同（对这个 `python` 包文件的引用可以在 `PYTHON_PATH` 附近找到）。

Firstly, open `dataset_generator/generate_datasets.sh`, please set `CARLA_ROOT` to your CARLA directory, then check out `PYTHON_PATH`, make sure the `.egg` file bash script refers to is exactly the same file you installed.

然后是 `dataset_generator/tasks.json` 这个配置文件。这里给出一个示例：

Then set config file `dataset_generator/tasks.json`, here's an example:

```json
[
    {
        "scenario_path": "./scenarios/Scenario3/Town06_Scenario3.json",
        "time_limit": 1,
        "shot_count": 2,
        "frame_rate": 60,
        "data_rate": 10,
        "vehicle_count": 20,
        "pedestrian_count": 20
    },
    {
        "scenario_path": "./scenarios/Scenario1/Town05_Scenario1.json",
        "time_limit": 1,
        "shot_count": 2,
        "frame_rate": 60,
        "data_rate": 10,
        "vehicle_count": 20,
        "pedestrian_count": 20
    }
]
```

在这个示例里，每组大括号内代表一个 task，`senario_path` 代表路径文件所在位置，这个文件决定了 `ego_vehicle` 的行驶路线。这些路径文件可以在 `transfuser` 项目中找到。`time_limit` 是每个任务在CARLA中执行时间的上限（单位：秒），当车辆到达终点或者达到时间限制时，会终止仿真和传感器采集。`shot_count` 是这个 task 执行的次数，每一个 shot 会生成一个数据集子集。`frame_rate` 代表游戏中仿真每秒的 tick 数，建议不要太少，否则每一步模拟的时间太长会导致问题。`data_rate` 代表传感器每秒采集的数据份数，`vehicle_count` 和 `pedestrain_count` 代表场景中NPC车辆、行人的数量。

生成的数据集会存储在 `dataset_generator/data`。数据结构请看[anno.md](./anno.md)。

In this example, each set of attributes inside a `{}` represents a task. `senario_path` links to the position of route config file, which decides the running route of ego vehicle. These route files can be found inside `transfuser` project. `time_limit` is the maximum running time (in seconds) inside CARLA of this task. Either when the ego vehicle reaches the destination or the clock inside CARLA exceeds time limit, simulation of this task will end. `shot_count` is the iteration count of the task, each shot will generate a subset of dataset independently. `frame_rate` is the tick count per second inside CARLA. We recommend a higher rate to guarantee the precision of simulation. `data_rate` defines frequancies of sensors, `vehicle_count` and `pedestrain_count` defines the count of NPC vehicles and pedestrains in simulated CARLA town.

Generated datased will be stored under `dataset_generator/data`. For the structure of data, please refer to [anno.md](./anno.md)

## 运行 Run

首先到 Carla 所在路径运行 Carla:

Before running data generation, cd to Carla's directory to run Carla:

```bash
   ./CarlaUE4.sh --world-port=2000
```

然后运行 `generate_datasets.sh`：

Then run `generate_datasets.sh`:

```bash
    cd ./dataset_generator
    chmod a+x ./generate_all_datasets.sh
    ./generate_all_datasets.sh
```

## 特别提醒 Special Reminder

 - 在无图形化界面服务器上运行新版 Carla 时，请添加参数 `-RenderOffScreen`，即

   ```bash
   ./CarlaUE4.sh -RenderOffScreen
   ```
 
 - 在使用 GNU-Screen 工具进行后台运行时，一定要先进入 screen 再进行 conda/source activate ，否则也会产生一些问题。
