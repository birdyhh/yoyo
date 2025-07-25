# yoyo

## 为什么用行为树组织 MTC stages

* **灵活性**：行为树天然支持顺序、选择、并行、条件等复杂逻辑，适合描述机器人任务流程。
* **容错与回退**：行为树的 Selector/Sequence 节点可实现自动回退和多策略尝试，类似 MTC 的 Fallbacks/Alternatives。
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
3. 行为树 XML/DSL 文件描述任务逻辑，机器人运行时加载并执行。
4. 可以通过 ROS2 action/service/topic 与 MTC 任务交互。
5. BT 节点只负责配置和添加 stage，不负责执行，这样可以灵活组合和复用 stage。
6. 主程序统一调用 task->init() 和 task->plan()，保证所有 stage 都已添加后再整体初始化和规划，避免重复初始化和资源冲突。
7. 参数通过黑板传递，如 ROS2 节点、MTC Task、目标状态等，便于解耦和扩展。
8. 行为树 tick 只做“拼装”，不做实际运动执行，所有执行和结果处理在主程序统一完成。

### 典型完整流程（伪代码）
1. 主程序初始化 ROS2 node、MTC Task、行为树工厂和黑板参数，启动service。
2. 
2. 行为树 tick，每个节点根据参数调用对应 MTC stage。
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
    BT->>MTC: 调用节点添加stages
    deactivate BT
    BT-->>Server: 返回组装结果
    Server-->>Client: 返回加载行为树结果
    deactivate Server
    
    loop every call
        Client->>Server: ExecuteMtcTask()
        activate Server
        Server->>MTC: init()
        MTC-->>Server: 初始化规划
        Server->>MTC: plan()
        MTC-->>Server: 规划结果
        Server->>MTC: execute()
        MTC-->>Server: 执行规划
        Server-->>Client: 返回执行结果
        deactivate Server
    end

```

### 操作指南

#### 调用service接口更新行为树xml字符串
```bash
ros2 service call /update_bt_xml bt_service_interfaces/srv/UpdateBTXml  "{xml: '<?xml version=\"1.0\" encoding=\"UTF-8\"?><root BTCPP_format=\"4\" main_tree_to_execute=\"main\"><BehaviorTree ID=\"main\"><Sequence><MoveToBTNode goal=\"open\" planner_type=\"0.5\" /></Sequence></BehaviorTree></root>'}"
```