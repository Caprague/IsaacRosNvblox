# 修改 Docker 用户权限以实现 SHM 双向通信 — 影响分析（最终版）

## 背景

### 当前问题
容器内 ROS2 进程以 root (UID=0) 身份创建 SHM 段（权限 644），宿主机 nvidia 用户 (UID=1000) 只有读权限，FastDDS DataSharing 需要双向写入 → 数据投递静默失败。当前采用"主机侧 UDP-only profile"作为 workaround。

### 最终结论

**Dockerfile 和脚本不需要任何修改。** 现有机制已正确实现 UID/GID 对齐 + SHM 通信条件。

问题的真正根因是：**通过 VSCode Dev Containers 插件进入容器时，默认以 root 用户执行，绕过了 entrypoint 的 gosu 降权流程**，导致在 VSCode 终端中启动的 ROS2 进程以 UID=0 运行，创建的 SHM 段宿主机无法写入。

---

## 1. 现有机制验证

### UID/GID 对齐已正确

| | UID | GID | 用户名 | 组名 |
|--|-----|-----|--------|------|
| 容器 (通过 run_dev.sh 进入) | 1000 | 1001 | admin | admin |
| 宿主机 | 1000 | 1001 | nvidia | docker |

- UID=1000 一致，GID=1001 一致
- 用户名和组名差异 **完全不影响** — 内核权限检查只看数值
- `--ipc=host` 已配置（run_dev.sh 第288行）— 容器与宿主机共享 IPC 命名空间（含 `/dev/shm`）
- **不需要** 额外 `-v /dev/shm:/dev/shm`

### SHM 通信条件全部满足

| 条件 | 状态 |
|------|------|
| `--ipc=host` | 已有 (run_dev.sh:288) |
| 容器 ROS2 进程 UID = 宿主机 UID | 通过 run_dev.sh 进入时已满足 |
| GID 一致 | 已满足 |
| GN (group name) 一致 | **不需要** |
| `-v /dev/shm:/dev/shm` | **不需要** |

---

## 2. 真正的问题根因

### workspace-entrypoint.sh 的降权流程

```
docker run ... --entrypoint workspace-entrypoint.sh
  → PID 1 以 root 启动
  → groupadd/useradd 创建 UID=1000 用户
  → exec gosu admin /bin/bash   ← bash 以 UID=1000 运行
      → ros2 launch ...          ← 也以 UID=1000 运行 ✓
```

通过 `run_dev.sh` 脚本进入容器时，一切正常。

### VSCode Dev Containers 的进入方式

```
VSCode → docker exec -it <container> /bin/bash
  → 直接以容器默认用户 (root) 启动 shell
  → 不经过 workspace-entrypoint.sh
  → ros2 launch ...              ← 以 UID=0 运行 ✗
      → /dev/shm/fastrtps* owner = root:root (644)
      → 宿主机 nvidia(1000) 无写权限 → SHM 投递失败
```

**这就是为什么 `ros2 topic list` 能看到 topic（发现走 UDP 多播正常），但 `ros2 topic echo` 收不到数据（数据投递走 SHM 失败）。**

---

## 3. 解决方案

### 方案 A：VSCode devcontainer.json 配置（推荐）

```jsonc
// .devcontainer/devcontainer.json
{
  "remoteUser": "admin"
}
```

这样 VSCode 所有终端和扩展都以 admin (UID=1000) 运行。

### 方案 B：VSCode settings.json

```jsonc
{
  "dev.containers.dockerPath": "docker",
  "dev.containers.executeInWorkerRecreate": false
}
```

并在 attach 时手动指定用户。

### 方案 C：容器内 root 自动切换用户

在容器的 `/root/.bashrc` 末尾追加：

```bash
# 如果以 root 登录且 admin 用户存在，自动切换
if [ "$(id -u)" = "0" ] && id admin &>/dev/null; then
    exec gosu admin bash
fi
```

这样即使 VSCode 以 root 进入，也会自动降权到 admin。

### 方案 D：清理残留 + 移除 UDP workaround

确认问题解决后：

```bash
# 宿主机清理旧 SHM 残留
sudo rm -f /dev/shm/fastrtps*

# 宿主机移除 UDP-only 限制（~/.bashrc 中注释掉）
# export FASTRTPS_DEFAULT_PROFILES_FILE=...  ← 注释或删除

# 容器内确认无 profile 限制
echo $FASTRTPS_DEFAULT_PROFILES_FILE  # 应为空
echo $FASTDDS_DEFAULT_PROFILES_FILE   # 应为空
```

---

## 4. 验证步骤

```bash
# 1. 从 VSCode 终端确认用户身份
id
# 应显示 uid=1000(admin) 而非 uid=0(root)

# 2. 清理旧 SHM 残留
sudo rm -f /dev/shm/fastrtps*

# 3. 启动 ROS2 节点
ros2 launch <your_launch_file>

# 4. 检查新 SHM 文件 owner
ls -la /dev/shm/fastrtps*
# 应显示 admin:admin (1000:1001)，而非 root:root

# 5. 宿主机端（确保未设置 UDP-only profile）
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 topic echo /your/topic --once
# 应能收到数据
```

---

## 5. 总结

| 问题 | 根因 | 解决方案 | 需要修改 Dockerfile？ |
|------|------|---------|---------------------|
| SHM 权限不匹配 | VSCode 以 root 进入容器 | 配置 `remoteUser: admin` 或 root bashrc 自动切换 | **不需要** |
| 跨容器-宿主机通信失败 | 同上导致 SHM 段 owner 为 root | 同上 | **不需要** |
| 之前的 UDP workaround | 为绕过 SHM 权限问题 | 解决根因后可移除 | **不需要** |

### 教训

- `run_dev.sh` 的 entrypoint 机制设计正确：通过 `HOST_USER_UID`/`HOST_USER_GID` + `gosu` 实现运行时 UID 对齐
- 但只有通过 `run_dev.sh` 或 `docker exec -u admin` 进入容器时才走这个流程
- VSCode / Portainer / 其他工具直接 `docker exec` 时默认 root，绕过了整个降权机制
- **任何从非 entrypoint 路径进入容器的方式，都需要显式指定用户**
