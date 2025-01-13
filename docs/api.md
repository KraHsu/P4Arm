# API 文档

## V1.0.0

### Arm

#### 末端移动

`Addr` : move

`Code` `[5x左臂，6x右臂]`:
- 移动机械臂末端 ：`50/60, {"coor": [x, y, z, rx, ry, rz]}`
    - 成功：`0, {"pose": [x, y, z, rx, ry, rz]}`
    - 失败：`100, {"message": "错误原因"}` 
- 获取末端坐标：`51, {}`
    - 成功:`0, {"pose": [x, y, z, rx, ry, rz]}`
    - 失败:`100, {"message": "错误原因"}`
- 获取关节角度: `52, {}`
    - 成功：`0, {"pose": [theta1, theta2, theta3, theta4, theta5, theta6, theta7]}`
    - 失败：`100, {"message": "错误原因"}`
- 移动机械臂末端（样条曲线）: `53, {"coor": [[x, y, z, rx, ry, rz][x, y, z, rx, ry, rz], [x, y, z, rx, ry, rz]...]}`
    - 成功：`0, {"pose": [x, y, z, rx, ry, rz]}`
    - 失败：`100, {"message": "错误原因"}`
- 移动机械臂末端（笛卡尔空间圆弧）: `54, {"center": [x, y, z, rx, ry, rz],  "end": [x, y, z, rx, ry, rz], "round": float}`
    - 成功：`0, {"pose": [x, y, z, rx, ry, rz]}`
    - 失败：`100, {"message": "错误原因"}`
- 移动机械臂末端（笛卡尔直线）: `55, {"coor": [x, y, z, rx, ry, rz]}`
    - 成功：`0, {"pose": [x, y, z, rx, ry, rz]}`
    - 失败：`100, {"message": "错误原因"}`

## V2.0.0

### Arm

`Addr`：`ArmL/ArmR`

----

移动A到B：`101`
```yaml
# Payload
Actor: "Actor"          # 执行动作的主体，可选 "Actor" 或 "Wrist"
Target:                 # 目标点配置
    Frame: "World"      # 目标点参考的坐标系，可选 "World" 或 "Base"
    Position:           # 目标点的位置
        - 0.0           # x 坐标
        - 0.0           # y 坐标
        - 0.0           # z 坐标
    Euler:              # 目标点的欧拉角
        - 0.0           # rx 角度
        - 0.0           # ry 角度
        - 0.0           # rz 角度
Speed: 0                # 1-100则使用给定速度，0表示使用配置速度
# Succ 300
                        # Nothing
# Fail 400
Msg: ""                 # 错误原因
```
注意：
- `Actor`：表示机械手或夹爪抓去点，具体含义由配置文件`Actor`决定
- `Position`：单位为`米/m`
- `Euler`：角度单位使用弧度制，参数顺序为`rx->ry->rz`，外旋

----

读取位置：`102`
```yaml
# Payload
Actor: "Actor"          # 需要读取的目标，可选 "Actor" 或 "Wrist"
Frame: "World"          # 参考的坐标系，可选 "World" 或 "Base"
# Succ 300
Position:           # 目标点的位置
    - 0.0           # x 坐标
    - 0.0           # y 坐标
    - 0.0           # z 坐标
Euler:              # 目标点的欧拉角
    - 0.0           # rx 角度
    - 0.0           # ry 角度
    - 0.0           # rz 角度
# Fail 400
Msg: ""             # 错误原因
```
注意：
- `Actor`：表示机械手或夹爪抓去点，具体含义由配置文件`Actor`决定
- `Position`：单位为`米/m`
- `Euler`：角度单位使用弧度制，参数顺序为`rx->ry->rz`，外旋

----

配置：`111`
```yaml
Key: "Speed" # 需要修改的配置项
Value: 0.0   # 需要修改的值
```
可修改的项见 Config
----

返回值：`300/400`

```yaml
Code: 300/400 # 300 代表成功，400 代表失败
Res:          # 反馈信息/错误原因
```

### Config

配置文件，放在可执行文件同级目录下

或放在项目根目录下，编译时会自动复制

```yaml
# 不可修改，仅通过配置文件生效
Actor:
    ArmL: ""        # "Hand" | "Grip"
    ArmR: ""
    OffsetL:        # 抓取点相对于手腕坐标系的偏移
        Position:   # 单位 m
            - 0     # x
            - 0     # y
            - 0     # z
        Euler:      # 弧度 rx->ry->rz 外旋
            - 0     # rx
            - 0     # ry
            - 0     # rz   
    OffsetR:
        ...

Address:
    ArmL: ""    # "192.168.1.18:8080"
    ArmR: ""    # "192.168.2.18:8080"
    # 仅配置 Actor 为 Hand 时启用
    HandL: ""   # "192.168.12.210:6000" | "485"
    HandR: ""   # "192.168.11.210:6000" | "485"
# 可修改
Speed: 1-100    # 机械臂移动速度
```

### Visual