#!/usr/bin/env python3
"""移动 Gazebo 方阵方块

用法:
    # 列出世界里的模型（真实名字，不是猜的）
    python3 scripts/move_block.py --list

    # 把方块移到指定位置
    python3 scripts/move_block.py block_0 0.34 0.10

    # 随机重排所有方块（--seed 可复现）
    python3 scripts/move_block.py --random --seed 1

⚠️⚠️⚠️⚠️ **四个坑，都是实测出来的（2026-09-27，gz sim 8.15.0 / Harmonic）：**

1. **`/world/<w>/set_pose` 在这个 build 里是空转的。** 它接受合法的
   `gz.msgs.Pose`、无条件回 `data: true`，然后**什么都不做**。实测矩阵：

   | 调用 | 应答 | 位姿是否变化 |
   |---|---|---|
   | `set_pose`（`name` 指定，合法请求） | `data: true` | **否** |
   | `set_pose`（改用 `id: 29`） | `data: true` | **否** |
   | `set_pose`（`name` + `id` 都给） | `data: true` | **否** |
   | `set_pose_vector` | `data: true` | **否** |
   | 对 `table` 这种大模型 | `data: true` | **否** |
   | 显式 `control {pause: false}` 之后再调 | `data: true` | **否** |
   | **模型根本不存在** | `data: true` | 否（本该失败） |
   | `/world/<w>/create`（对照） | `data: true` | 见坑 3 |
   | `/world/<w>/remove`（对照） | `data: true` | **是，真的删掉了** |

   服务自报的请求类型确实是 `gz.msgs.Pose`（`gz service -i -s ...`），
   非法 `--req` 会被 protobuf 拒掉、不存在的服务会超时 —— 所以**不是我的请求写错了**，
   是这个服务本身没接上。**结论：这个脚本原先的 `✓/✗` 是假判据**，
   它对着不存在的模型一样打 ✓（旧版正是如此，且方块名还是错的 `block_r0_c0`）。
   现在改成 **`remove` + `create`**，并且**读完位姿再判定成功**。

2. **本脚本会让 `block_grid_world_meta.json` 与场景不一致。** 标注器
   （`scripts/auto_label_grid.py`）读的是那份 meta 里的方块坐标；在这里挪了方块却
   不更新 meta，**之后产出的标签就是错的**。所以要**训练用的**随机布局，请走
   `python3 scripts/generate_grid_world.py --seed N`（它一次把世界与 meta 一起重生成，
   天然自洽），而不是本脚本。本脚本只适合**当场手工摆一摆**看看检测效果。

3. **`create` 有两个独立的陷阱，我各踩了一次。**

   (a) **SDF 必须包在 `<sdf>` 根元素里。** 裸 `<model>…</model>` 会被 sdformat 拒掉，
   服务端日志写得很直白：

   ```
   [Err] [UserCommands.cc:927] Error Code 40: Msg: XML does not seem to be an
   SDFormat or an URDF string.
   [Err] [UserCommands.cc:927] Error Code 30: Msg: Unable to read SDF string: …
   ```

   四种形态实测（判据是 `gz model -m <名字> -p` 能否读回，不是服务应答）：

   | 送的 SDF | 应答 | 模型真的建出来 |
   |---|---|---|
   | 裸 `<model name="t">…</model>` | `true` | **否** |
   | **`<sdf version="1.10"><model …></model></sdf>`** | `true` | **是** ✅ |
   | 裸 `<model>` 最简（只有 `<link/>`） | `true` | 否 |
   | 裸 `<model>` 带真实换行 | `false` | 否（proto text 解析就挂了） |

   版本号从世界文件头里读（现在是 `1.11`），不写死。

   (b) **`create` 的应答不能当作成功判据 —— 它连失败都回 `data: true`。**
   上面三行「应答 true 但模型不存在」就是证据。**这一条差点毁掉我给它加的预检**：
   我本来加了「先在高处建个探针、建得出来才动真的」来堵 `remove`+`create` 的
   真空期，可那版探针也是拿 `data: true` 判的 —— 于是探针**假通过**，
   正式那块照旧被删掉、没建回来，方块又一次从世界里消失。
   **判据一律用 `read_pose()` 读回**，`create_model()` / `remove_model()` 都自带重试。

4. **做判据的那个 `gz model` 自己也会假失败（约 10%）。** 连续 30 次读同一个方块：

   ```
   第 15/18/25 次失败：rc=0 用时 5.23s
   Service call to [/gazebo/worlds] timed out
   Command failed when trying to get the world name of the running simulation.
   失败 3/30；耗时 min=0.67s max=5.24s 均值=1.19s
   ```

   `gz model` 每次都要先做一次世界发现，那一步会偶发超时，**而退出码仍是 0**。
   后果不是报错，是**报错报错对象**：`--random` 那次有两块被写成
   「✗ 世界里没有这个模型」，可它俩好端端地在世界里没动过。
   所以 `read_pose()` 分「确定没有」与「读了个寂寞」两种，前者立即返回、
   后者重试（3 次全失败的概率 ~0.1%）；`list_models()` 同样重试 ——
   那里假失败会**静默漏掉方块**，连错都不报。
"""

import subprocess
import sys
import re
import time
import random
import argparse
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
WORLDS_DIR = PROJECT_ROOT / "ros2_ws" / "src" / "yolo_bringup" / "worlds"

# 桌面 0.8×0.6，中心 (0.5, 0)；方块边长 0.04，桌面顶端 0.765 ⇒ 中心 0.785。
# 与 generate_grid_world.py 的常量一致（那才是唯一真源，这里只是默认值）。
TABLE_W, TABLE_H, TABLE_X, TABLE_Y = 0.8, 0.6, 0.5, 0.0
BLOCK_Z = 0.785


def gz(*args, timeout=5):
    """跑一条 gz 命令，返回 (退出码, stdout+stderr)。"""
    try:
        r = subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, "<timeout>"
    except FileNotFoundError:
        return 127, "<找不到 gz 命令>"


def list_models(tries=4):
    """世界里真实的模型名。**不再硬编码** —— 旧版写死 `block_r0_c0`，
    而 SDF 里的真名是 `block_0..8`，于是它列出的名字一个都不存在。

    自带重试：`gz model` 有偶发假失败（见 `pose_once`），而这里若假失败会
    **静默漏掉方块** —— 更糟的是会漏成空表，让脚本报「这个世界里没有 block_* 模型」
    然后退出。实测就撞上过一次：世界活着、9 块齐全，`list_models()` 却回了空表。
    所以**只有试满次数才认空**。
    """
    for _ in range(tries):
        _, out = gz("model", "--list")
        names = [m.group(1) for m in re.finditer(r"^\s*-\s+(\S+)\s*$", out, re.M)]
        if names:
            return names
        time.sleep(0.3)
    return []


def world_alive(world, tries=3):
    """世界在跑吗（有 set_pose 服务就说明在跑）。试满次数才认「没在跑」。"""
    for _ in range(tries):
        _, out = gz("service", "-l")
        if f"/world/{world}/set_pose" in out:
            return True
        time.sleep(0.3)
    return False


def pose_once(name):
    """一次读取尝试 → (位姿 或 None, 这次尝试**是否可信**)。

    ⚠️ 不可信的情形：`gz model` 每次都要先向 `/gazebo/worlds` 做一次世界发现，
    而**那一步约 10% 的调用会超时**。实测（连续 30 次读同一个方块）：

    ```
    第 15/18/25 次失败：rc=0 用时 5.23s
    Service call to [/gazebo/worlds] timed out
    Command failed when trying to get the world name of the running simulation.
    失败 3/30；耗时 min=0.67s max=5.24s 均值=1.19s
    ```

    注意 **`rc` 仍是 0** —— 退出码在这里一样不能信。所以判据只能是
    「有没有读到 `[x y z]` 那行」，而「读不到」还得再分成
    **确定没有这个模型**与**读了个寂寞**。

    ⚠️ 分法必须是**正面识别「没有」**，不能靠「没看见某个错误字样」来推断。
    我第一版写的是 `"timed out" not in out`，等于把「任何我没预料到的失败」
    都归类成「模型不存在」—— 于是 `list_models()` 一旦碰到没见过的失败，
    就平静地回报一张空表，脚本继而打印「这个世界里没有 block_* 模型」并退出。
    实测撞上过一次：世界活着、9 块齐全，却报了空。
    现在只有读到 `No model named <X> was found`（实测原文）才算「确定没有」，
    **其余一律算不确定、交给重试**。
    """
    rc, out = gz("model", "-m", name, "-p")
    for line in out.splitlines():
        m = re.match(r"^\s*\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]\s*$", line)
        if m:
            return tuple(float(g) for g in m.groups()), True
    return None, f"No model named <{name}> was found" in out


def read_pose(name, tries=3):
    """读模型位姿 → (x, y, z)，读不到返回 None。

    `gz model -m X -p` 读出的位置是**可信的**：实测它精确读出 `create` 时给的
    位置（建在 (0.2,0.2,1.2) 就读出 0.200000 0.200000 1.200000）。
    但**调用本身会假失败**（上面 `pose_once` 记的 10% 超时），所以这里重试：
    单次假失败 ~10%，三次全失败 ~0.1%。
    """
    for _ in range(tries):
        pose, conclusive = pose_once(name)
        if pose is not None:
            return pose
        if conclusive:
            return None        # 应答正常却没有 —— 这才叫「没有这个模型」
        time.sleep(0.3)
    return None


def extract_model_sdf(sdf_path, name, strip_pose=True):
    """从世界 SDF 里抠出某个 `<model name="X">…</model>` 的原文（含颜色等）。

    靠**缩进**配平：模型都在 4 空格缩进，第一个同为 4 空格缩进的 `</model>`
    就是它的收尾。抠出来是为了 remove 之后按原样 create 回去 ——
    否则方块会丢掉材质颜色（检测器就是靠颜色/形状学的）。
    """
    lines = Path(sdf_path).read_text().splitlines()
    start = None
    for i, line in enumerate(lines):
        if re.match(r'^\s*<model\s+name="%s"\s*>\s*$' % re.escape(name), line):
            start = i
            indent = len(line) - len(line.lstrip())
            break
    if start is None:
        return None
    for j in range(start + 1, len(lines)):
        line = lines[j]
        if line.strip() == "</model>" and (len(line) - len(line.lstrip())) == indent:
            body = lines[start:j + 1]
            if strip_pose:
                # 去掉自带的 <pose>：位置由服务的 `pose:` 字段给，
                # 两处都给容易打架（哪边生效不好推理）。
                body = [l for l in body if not re.match(r"^\s*<pose>", l)]
            return " ".join(" ".join(body).split())
    return None


def read_model_pose(sdf_path, name):
    """读模型在世界文件里记的**模型级** pose → (x, y, z)，读不到返回 None。

    取的是**第一个** `<pose>`。这依赖「模型级 `<pose>` 排在任何 `<link>` 之前」——
    本 world 文件确实如此（`block_0`: 191 行 `<model>` / 193 行 `<pose>` /
    194 行 `<link>`），`--restore` 靠它把方块建回原位。
    """
    body = extract_model_sdf(sdf_path, name, strip_pose=False)
    if body is None:
        return None
    m = re.search(r"<pose>([^<]*)</pose>", body)
    if not m:
        return None
    vals = m.group(1).split()
    if len(vals) < 3:
        return None
    return tuple(float(v) for v in vals[:3])


def call_service(world, service, reqtype, req, timeout=5000):
    return gz("service", "-s", f"/world/{world}/{service}",
              "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
              "--timeout", str(timeout), "--req", req, timeout=timeout / 1000.0 + 5)


def list_sdf_models(sdf_path):
    """世界文件里**定义**了哪些模型（含当前不在世界里的）。顺序即文件顺序。"""
    return re.findall(r'<model\s+name="([^"]+)"',
                      Path(sdf_path).read_text())


def read_sdf_version(sdf_path):
    """从世界文件头读 `<sdf version="X">`。写死版本号迟早与世界文件脱节。"""
    m = re.search(r'<sdf\s+version="([^"]+)"', Path(sdf_path).read_text())
    return m.group(1) if m else "1.10"


def wrap_sdf(body, version):
    """把模型片段包进 `<sdf>` 根元素。

    ⚠️ **这一步不能省。** 裸 `<model>…</model>` 会被 sdformat 拒掉
    （服务端：`Error Code 30: Unable to read SDF string`）。
    """
    return f'<sdf version="{version}">{body}</sdf>'


def remove_model(world, name, tries=6):
    """删模型，**按读回判定**（删完应读不到）。

    轮询里用单次尝试 `pose_once`，只在「应答正常且读不到」时才认定删掉了 ——
    否则一次读取超时会被误当成「已经删掉」。
    """
    call_service(world, "remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL')
    for _ in range(tries):
        pose, conclusive = pose_once(name)
        if pose is None and conclusive:
            return True, "已删除"
        time.sleep(0.2)
    if read_pose(name) is None:      # 收尾带重试，免得把读取超时当成删成功
        return True, "已删除"
    return False, "remove 之后模型还在"


def create_model(world, sdf, name, x, y, z, tries=8):
    """按 `sdf` 建模型，位置由服务的 `pose:` 给。返回 (ok, 说明)。

    ⚠️ **成功与否只看 `read_pose(name)` 能否读回，不看服务应答** ——
    实测失败的 create 一样回 `data: true`。
    ⚠️ 模型名取自 **SDF 内部**的 `<model name=...>`，不是这里的参数 ——
    想换个名字建，得先改 SDF 文本。`name` 只用于读回校验。
    ⚠️ SDF 里满是双引号（`name="link"`），塞进 proto text 的字符串字段**必须转义**，
    否则 protobuf 在第一个内层引号处就报错
    （实测：`Error parsing text-format gz.msgs.EntityFactory`）。
    """
    sdf_escaped = sdf.replace("\\", "\\\\").replace('"', '\\"')
    req = (f'sdf: "{sdf_escaped}", '
           f'pose: {{position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}}}')
    rc, out = call_service(world, "create", "gz.msgs.EntityFactory", req)
    for _ in range(tries):
        got, _ = pose_once(name)
        if got is not None:
            return True, got
        time.sleep(0.2)
    got = read_pose(name)            # 收尾带重试，免得把读取超时当成 create 失败
    if got is not None:
        return True, got
    return False, (f"服务应答={'data: true' in out}，但世界里读不到 {name} —— "
                   f"create 大概被 sdformat 拒了，看服务端日志的 "
                   f"`UserCommands.cc:927`。原始应答：{out.strip()[:60]}")


def move(world, name, x, y, z, sdf_cache):
    """把模型挪到 (x,y,z)。**成功与否由读回位姿判定**，不看服务应答。

    返回 (ok, detail)。

    ⚠️ **remove + create 中间有真空期**：create 一旦失败，方块就**从世界里没了**。
    这不是假想 —— 第一版就发生了（SDF 没转义 → create 被拒 → `block_0` 消失，
    而如果照旧版那样只看服务应答，还会报 ✓）。所以正式动手之前先做一道**预检**：
    用同一份 SDF 在**高处**（z=3.0，避开桌面）建一个临时探针，建得出来才动真的。
    预检失败的代价是「什么都没做」，不是「丢了一个方块」。

    ⚠️ 预检**必须靠读回判定**：第一版探针是拿服务应答判的，而失败的 create 也回
    `data: true`，于是探针**假通过**、方块照样丢。判据已全部换成 `read_pose()`。
    """
    if read_pose(name) is None:
        return False, "世界里没有这个模型"

    sdf = sdf_cache.get(name)
    if sdf is None:
        return False, "在世界 SDF 里找不到它的定义，无法重建"

    probe = f"__preflight_{name}"
    probe_sdf = sdf.replace(f'name="{name}"', f'name="{probe}"', 1)
    ok, detail = create_model(world, probe_sdf, probe, 0.0, 0.0, 3.0)
    if not ok:
        return False, f"预检失败，**没有动** {name}：{detail}"
    ok, detail = remove_model(world, probe)
    if not ok:
        return False, (f"预检的探针删不掉（{detail}）——世界里会多一个 {probe}，"
                       f"先手工清掉再重试")

    ok, detail = remove_model(world, name)
    if not ok:
        return False, detail

    ok, got = create_model(world, sdf, name, x, y, z)
    if not ok:
        return False, (f"{name} 已从世界里消失：{got}"
                       f"\n     恢复：重启仿真 `ros2 launch yolo_bringup "
                       f"grid_world.launch.py`，或重生成世界 "
                       f"`python3 scripts/generate_grid_world.py`")
    if max(abs(got[0] - x), abs(got[1] - y), abs(got[2] - z)) > 1e-3:
        return False, f"读回位姿不符：期望 ({x:.3f}, {y:.3f}, {z:.3f})，实际 {got}"
    return True, f"({got[0]:.3f}, {got[1]:.3f}, {got[2]:.3f})"


def main():
    ap = argparse.ArgumentParser(
        description="移动 Gazebo 方阵方块（remove + create，因为 set_pose 在此 build 下空转）",
        epilog="⚠️ 挪动不会更新 block_grid_world_meta.json，标注/训练前请改用 "
               "generate_grid_world.py --seed N。")
    ap.add_argument("model", nargs="?", help="模型名（用 --list 看真名）")
    ap.add_argument("x", nargs="?", type=float, help="X 坐标")
    ap.add_argument("y", nargs="?", type=float, help="Y 坐标")
    ap.add_argument("--z", type=float, default=BLOCK_Z, help=f"Z 高度（默认 {BLOCK_Z}）")
    ap.add_argument("--world", default="block_grid_world", help="世界名")
    ap.add_argument("--list", action="store_true", help="列出世界里的模型")
    ap.add_argument("--random", action="store_true", help="随机重排所有方块")
    ap.add_argument("--seed", type=int, default=None, help="--random 的种子（可复现）")
    ap.add_argument("--restore", metavar="模型名",
                    help="把世界 SDF 里有、但世界里没有的模型按记的原位建回来"
                         "（本脚本 create 失败后的恢复路径）")
    args = ap.parse_args()

    if not world_alive(args.world):
        print(f"✋ 世界 {args.world} 没在跑。先起：\n"
              f"   ros2 launch yolo_bringup grid_world.launch.py headless:=true",
              file=sys.stderr)
        return 2

    models = list_models()
    # ⚠️ **必须排序**，不能直接用 `list_models()` 的顺序。那是世界内部的顺序，
    # 而方块每被 remove+create 一次就跑到列表末尾 —— 于是同一个 `--seed`
    # 只会复现「抽签序列」，复现不了「谁拿到哪一签」。实测过：
    # 两遍 `--random --seed 1` 抽出的 9 组坐标一模一样，但对应到方块名全变了。
    blocks = sorted((m for m in models if m.startswith("block_")),
                    key=lambda m: (int(m.rsplit("_", 1)[1])
                                   if m.rsplit("_", 1)[-1].isdigit() else -1, m))

    if args.list:
        print(f"世界 {args.world} 里的模型（{len(models)} 个）:")
        for m in models:
            mark = " ← 方块" if m in blocks else ""
            pose = read_pose(m)
            pos = f"  ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})" if pose else ""
            print(f"  {m}{pos}{mark}")
        print(f"\n示例: python3 scripts/move_block.py {(blocks or models)[0]} 0.34 0.10")
        return 0

    sdf_path = WORLDS_DIR / f"{args.world}.sdf"
    # 缓存里存的是**可直接发出去**的 SDF：已经包好 `<sdf>` 根元素
    # （裸 `<model>` 会被 sdformat 拒），版本号从世界文件头读，不写死。
    #
    # 取「世界里跑着的」∪「世界文件里定义了的」——后者是 `--restore` 要用的：
    # 从世界里消失的方块，恰恰不在前者的名单里。
    sdf_version = read_sdf_version(sdf_path)
    names = list(dict.fromkeys(models + list_sdf_models(sdf_path)))
    sdf_cache = {}
    for m in names:
        body = extract_model_sdf(sdf_path, m)
        sdf_cache[m] = wrap_sdf(body, sdf_version) if body else None

    if args.restore:
        name = args.restore
        if sdf_cache.get(name) is None:
            print(f"✋ 世界文件里没有 {name} 的定义，建不回来", file=sys.stderr)
            return 2
        if read_pose(name) is not None:
            print(f"{name} 已经在世界里了，无需恢复")
            return 0
        pose = read_model_pose(sdf_path, name)
        if pose is None:
            print(f"✋ {name} 在世界文件里没记 pose，请改用 "
                  f"`{name} <x> <y> [z]` 指定位置", file=sys.stderr)
            return 2
        ok, detail = create_model(args.world, sdf_cache[name], name, *pose)
        print(f"恢复 {name} → {pose} {'✓ ' + str(detail) if ok else '✗ ' + detail}")
        return 0 if ok else 1

    if args.random:
        if not blocks:
            print("✋ 这个世界里没有 block_* 模型", file=sys.stderr)
            return 2
        rng = random.Random(args.seed)
        print(f"随机重排 {len(blocks)} 个方块"
              f"（seed={args.seed}）{'' if args.seed is not None else ' —— 加 --seed 可复现'}")
        ok_n = 0
        for b in blocks:
            x = rng.uniform(TABLE_X - TABLE_W / 2 + 0.06, TABLE_X + TABLE_W / 2 - 0.06)
            y = rng.uniform(TABLE_Y - TABLE_H / 2 + 0.06, TABLE_Y + TABLE_H / 2 - 0.06)
            ok, detail = move(args.world, b, x, y, args.z, sdf_cache)
            ok_n += ok
            print(f"  {b} → ({x:.2f}, {y:.2f}) {'✓ ' + detail if ok else '✗ ' + detail}")
        print(f"完成 {ok_n}/{len(blocks)}")
        print("⚠️ meta.json 已与场景不一致：要出训练数据请用 "
              "generate_grid_world.py --seed N 重生成世界+meta。")
        return 0 if ok_n == len(blocks) else 1

    if args.model and args.x is not None and args.y is not None:
        ok, detail = move(args.world, args.model, args.x, args.y, args.z, sdf_cache)
        print(f"{args.model} → ({args.x}, {args.y}, {args.z}) {'✓ ' + detail if ok else '✗ ' + detail}")
        return 0 if ok else 1

    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
