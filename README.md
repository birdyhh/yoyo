# yoyo

## 为什么用行为树组织 MTC stages

* **灵活性**：行为树天然支持顺序、选择、并行、条件等复杂逻辑，适合描述机器人任务流程。
* **容错与回退**：行为树的 Decorator/Sequence 节点可实现自动回退和多策略尝试，类似 MTC 的 Fallbacks/Alternatives。
* **可扩展性**：行为树节点可以封装 MTC 的单个 stage 或子任务，便于组合和复用。

## 解决方案设计

### 方案思路
* **每个 MTC stage 封装为一个行为树 Action 节点**，节点内部调用对应的 MTC stage 执行。
* **行为树结构描述任务流程**，如顺序执行（Sequence）、回退（Fallback/Selector）、并行（Parallel）等。
* **行为树引擎负责调度节点**，根据执行结果（SUCCESS/FAILURE/RUNNING）决定流程走向。
* **支持条件判断和动态切换**，如检测碰撞、抓取失败自动切换策略等。

### 关键实现点

1. 使用 BehaviorTree.CPP（MoveIt 2 Pro 默认集成）作为行为树引擎。
2. 每个 MTC stage 封装为 BT 的自定义 ActionNode。
3. 行为树节点内部调用 MTC stage 执行，并返回结果给行为树。
4. 使用行为树的 Sequence 节点实现 MTC 的 serial container
5. 使用行为树的 Decorator 节点实现 MTC 的 wrapper
6. 行为树 XML/DSL 文件描述任务逻辑，机器人运行时加载并执行。
7. 可以通过 ROS2 action/service/topic 与 MTC 任务交互。
8. BT 节点只负责配置和添加 stage，不负责执行，这样可以灵活组合和复用 stage。
9. 主程序统一调用 task->init()、plan()、execute()，保证所有 stage 都已添加后再整体初始化和规划，避免重复初始化和资源冲突。
10. 参数通过黑板传递，如 ROS2 节点、MTC Task、目标状态等，便于解耦和扩展。
11. 行为树 tick 只做“拼装”，不做实际运动执行，所有执行和结果处理在主程序统一完成。

### 典型完整流程（伪代码）
1. 主程序初始化 ROS2 node、MTC Task、行为树工厂和黑板参数，启动service。
2. 行为树 tick，每个节点根据参数调用对应 MTC stage，将所有的stage 结果拼装成树结果，并返回给主程序，并执行init()进行初始化。
3. 节点执行，根据 MTC 结果返回 SUCCESS/FAILURE。
4. 主程序根据行为树整体返回值判断任务是否完成。

### 与 ROS2 的集成
1. 节点生命周期：在主程序中初始化 ROS2 节点，并将其指针传递给 BT 节点（通过构造函数）。
2. MTC 任务管理：每个 BT 节点只负责一个 MTC stage，主程序负责创建和管理整个 MTC Task。
3. 异步执行：如需异步执行，可在 BT 节点中返回 RUNNING 并用回调/事件驱动方式通知完成。


### 时序图
```mermaid
sequenceDiagram
    participant Client
    participant Server as MTC Server
    participant BT as Behavior Tree
    participant MTC as MTC Task
    
    Client->>Server: LoadMtcTask(XML)
    activate Server
    Server->>MTC: 创建新任务
    Server->>BT: 从XML创建行为树
    activate BT
    BT->>BT: tickOnce()
    BT->>MTC: 调用BT节点添加stages
    BT-->>Server: 返回组装和初始化结果
    deactivate BT
    Server->>MTC: init()
    MTC-->>Server: 初始化结果
    Server-->>Client: 返回加载行为树结果
    deactivate Server
    
    loop every call
        Client->>Server: ExecuteMtcTask()
        activate Server

        Server->>MTC: plan()
        MTC-->>Server: 规划结果
        Server->>MTC: execute()
        MTC-->>Server: 执行结果
        Server-->>Client: 返回执行结果
        deactivate Server
    end

```

### 前置教程

[使用 MTC 进行拾取和放置](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)

### 操作指南

1. 编译

```bash
# 1. clone 项目和下载子模块
git clone --recursive https://github.com/birdyhh/yoyo.git
# 2. 编译
colcon build
```

2. 运行  
以下针对的是moveit2自带的机械臂**moveit_resources_panda**,
可以使用[moveit2_tutorials](https://github.com/moveit/moveit2_tutorials)的launch文件启动
```bash
# 1. 启动机械臂
# 进入到moveit2_tutorials_ws目录
source install/setup.bash
ros2 launch moveit2_tutorials mtc_demo.launch.py

# 2. 启动行为树节点
# 进入到yoyo_ws目录
source install/setup.bash
ros2 launch behavior_tree_executor behavior_tree_executor_node.launch.py
```

3. 生成行为树xml文件  
利用已有的[行为树节点](./src/behavior_tree_executor/tree/mtc_bt_node.btproj)，加载到groof2，利用图形化工具生成行为树。
![](./img/groot_tree.png)

4. 调用service接口更新行为树xml字符串，[示例](./src/behavior_tree_executor/tree/example.xml)
```bash
ros2 service call /update_bt_xml bt_service_interfaces/srv/UpdateBTXml  "{xml: '{替换为行为树的xml字符串}'}"
```

5. 调用service接口执行MTC的execute函数
```bash
ros2 service call /execute_mtc_task bt_service_interfaces/srv/ExecuteMtcTask
```

6. 运行结果
![](./img/pick_and_place.gif)