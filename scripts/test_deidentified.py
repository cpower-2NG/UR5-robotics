#!/usr/bin/env python3
"""
脱敏自检 —— 这个仓库是要**公开**的，别把身份信息再写回去
==========================================================
本仓从内部仓裁剪而来，历史里原本带着板子 IP、硬件序列号、machine-id、MAC 和
本机用户名。那些已经用 `git filter-repo` 清过一遍 —— 但**清历史不等于以后不会再
写进去**：下一次贴一段板端日志、抄一行 `hostname -I`，脏东西就又回来了，而且是
以「看起来很正常的文档」的形式回来。

所以这个脚本的作用是**当次拦住**，不是事后考古。它扫两处：

  1. **全部被跟踪的文本文件**（当前检出）
  2. **全部提交信息**（`git log --all --format=%B`）

第 2 条不是多余的：第一遍脱敏时 `--replace-text` 只改 blob、**不改提交信息**，
于是文件内容是干净的、而 `git log -p` 里仍然能翻出一堆 IP —— 这条路径真的漏过。

⚠️ 它**扫不了图片里的像素**。图上烧的文字（状态栏、叠加标注）filter-repo 改不了，
只能人眼看 —— 本项目已经因此漏过一次（示意图状态栏印着相机序列号前缀）。
图片相关的约定见 docs/ 里的说明。

── 怎么处理命中 ─────────────────────────────────────────────────────
· **真泄漏**：删掉/换成占位符。历史里的还得再跑一次 filter-repo。
· **本来就该在那儿的**（如 Docker 默认网桥 `172.17.0.1`）：加进下面的白名单，
  **并写清理由** —— 白名单没有理由，下一个人就只能靠猜。

── 额外的两条反向检查 ───────────────────────────────────────────────
· `config/realsense_camera.yaml` 必须**仍是占位符**。这条防的是「有人图省事把
  自己相机的真序列号填进仓库里」——那正是这份模板要避免的事。
· 本机可选的黑名单 `~/.config/ur5-vision-grasp/denylist.txt`（**不入库**，一行
  一个字面量）：把真实序列号之类放在那里，就能在本机扫到它们，而**不必把它们
  写进这个公开脚本**。这是关键 —— 自检脚本自己不能成为泄漏源。

用法:
    python3 scripts/test_deidentified.py
退出码 0 = 干净，1 = 有命中（逐条打印）。
"""

import os
import re
import subprocess
import sys

sys.dont_write_bytecode = True  # 防 .pyc 假绿（本仓约定）

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 本脚本自己要跳过：它满篇都是这些模式的**字面量**（正则、白名单），扫自己必然
# 满屏假阳性。跳过意味着「不能把真东西藏在这个文件里」—— 由 denylist 兜住。
SELF = os.path.relpath(os.path.abspath(__file__), PROJECT_ROOT)

DENYLIST_PATH = os.path.join(
    os.path.expanduser('~'), '.config', 'ur5-vision-grasp', 'denylist.txt')

# ── 白名单：每条都必须有理由 ────────────────────────────────────────────
ALLOWED_IPV4 = {
    '172.17.0.1':   'Docker 默认网桥 docker0 —— 与实验环境无关的通用默认值',
    '192.168.56.1': 'URCap 安装页内建默认值 / VirtualBox host-only 默认，非本机地址',
}
ALLOWED_EMAILS = {
    'user@example.com':              '占位符',
    'openclaw@2026.3.13':            '版本号，不是邮箱（正则误伤）',
    'Image@gz.msgs.Image':           'ROS/Gazebo 话题类型写法，不是邮箱',
    'JointState@gz.msgs.Model':      'ROS/Gazebo 话题类型写法，不是邮箱',
    'serial-getty@ttyGS0.service':   'systemd 单元名，不是邮箱',
    'serial-getty@ttyTCU0.service':  'systemd 单元名，不是邮箱',
    'git@github.com':                'git 的 SSH 远端写法（git@主机:路径），不是邮箱',
    # 下面两个是提交信息里的 AI 协作署名，**不是**使用者身份或位置信息，
    # 本脚本管的就是那两类，所以放行。要不要在公开历史里保留这个署名是另一件事
    # （属可读性/署名偏好），需要的话另行决定，别在这里悄悄抹掉。
    'noreply@anthropic.com':         'AI 协作署名（Co-Authored-By），非身份/位置信息',
    'copilot@github.com':            'AI 协作署名（Co-Authored-By），非身份/位置信息',
}
ALLOWED_MACS = {
    '00:00:5e:00:01:12': 'VRRP 标准虚拟 MAC（RFC 5798），不是任何一块网卡',
}
# md5/sha 校验和 —— 记的是**权重文件/二进制**的指纹，不是设备身份。
ALLOWED_HEX32 = {
    '20dbfb977616baeb67f27a6eed1582f3',
    '4916bd5e48f5d98557b1d1caad61a85f',
    '87fb70359c644ec6ab83faf309b2538e',
    'a926d959b5aa9f9e2435c2deb9666803',
    'e7fec53f2c948bff0facaf8d54102f02',
}

PATTERNS = [
    # 私网 / 链路本地 IPv4。不含 0.0.0.0/127.0.0.1/255.255.255.0（非私网段）。
    ('私网 IPv4', re.compile(
        r'\b(?:10\.\d{1,3}|172\.(?:1[6-9]|2\d|3[01])|192\.168|169\.254)'
        r'\.\d{1,3}\.\d{1,3}\b')),
    ('邮箱', re.compile(r'[A-Za-z0-9._%+-]+@[A-Za-z0-9._-]+(?:\.[A-Za-z0-9_-]+)+')),
    ('MAC 地址', re.compile(r'\b(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}\b')),
    # 用户名路径。**两种写法都算** —— 第一遍脱敏只写了 /home/<user> 这条正斜杠的，
    # 于是 Windows 路径 \home\<user> 原样留了下来（那条规则就是这么漏的）。
    # `\Users\` 同理：Windows 的 profile 名也是人名/学号。
    # 占位符 `<user>` 天然不会命中（`<` 不在字符类里），所以不用额外排除。
    ('用户名路径', re.compile(
        r'[/\\]home[/\\][A-Za-z0-9_.-]+'
        r'|[/\\]Users[/\\][A-Za-z0-9_.-]+')),
    # 完整公钥（base64 够长才算）。**故意不匹配截断写法**：文档里 `ssh-ed25519
    # AAAA…` 是在叙述「两行都是干净的公钥」这件事，不是真的贴了钥匙 ——
    # 一开始按 `AAAA` 匹配，把这类叙述全报成了命中。
    ('公钥/私钥材料', re.compile(
        r'BEGIN [A-Z ]*PRIVATE KEY'
        r'|ssh-(?:rsa|ed25519|dss) AAAA[0-9A-Za-z+/]{40,}')),
    # 13 位数字 ≈ Jetson 模组序列号的形状。两侧不许挨着数字、点**或十六进制
    # 字母**：`\d` 挡不住 SHA，实测 `e89a79d41b8a7af50084334d10d5187152815176`
    # 里的 `5187152815176` 被当成了序列号（前一个字符是 `d`，不是数字）。
    # 也不许挨着点，否则 `917.6205444335938` 这种浮点内参的尾巴会中招。
    ('13 位数字（疑似序列号）',
     re.compile(r'(?<![0-9A-Fa-f.])[0-9]{13}(?![0-9A-Fa-f])')),
    ('32 位 hex（疑似 machine-id / 秘钥指纹）',
     re.compile(r'(?<![0-9a-fA-F])[0-9a-fA-F]{32}(?![0-9a-fA-F])')),
]


def _git(*args):
    out = subprocess.run(['git', '-C', PROJECT_ROOT, *args],
                         capture_output=True, text=True, check=False)
    if out.returncode != 0:
        raise SystemExit(f'git {" ".join(args)} 失败: {out.stderr.strip()}')
    return out.stdout


def tracked_text_files():
    """被跟踪的文件里，能当文本读的那些（跳过二进制，见 `_looks_binary`）。"""
    names = [n for n in _git('ls-files', '-z').split('\0') if n]
    for rel in names:
        ab = os.path.join(PROJECT_ROOT, rel)
        if not os.path.isfile(ab):
            continue
        try:
            with open(ab, 'rb') as fh:
                head = fh.read(8192)
        except OSError:
            continue
        if _looks_binary(head):
            continue
        try:
            with open(ab, 'r', encoding='utf-8', errors='replace') as fh:
                yield rel, fh.read()
        except OSError:
            continue


def _looks_binary(head):
    """含 NUL 就当二进制。

    ⚠️ 必须跳过二进制，否则**压缩字节会撞出假阳性**：实测某张 PNG 的字节流里
    恰好有一段长得像邮箱，`git grep` 直接报 `Binary file ... matches`。
    扫图片像素是**人眼**的活（见文件头）。
    """
    return b'\0' in head


def _allowlisted(kind, hit):
    table = {
        '私网 IPv4': ALLOWED_IPV4,
        '邮箱': ALLOWED_EMAILS,
        'MAC 地址': ALLOWED_MACS,
        '32 位 hex（疑似 machine-id / 秘钥指纹）': ALLOWED_HEX32,
    }.get(kind, {})
    return hit in table


def scan_text(label, text, hits):
    """把 `text` 按行过一遍所有模式，命中记进 `hits`。"""
    for lineno, line in enumerate(text.splitlines(), 1):
        for kind, rx in PATTERNS:
            for m in rx.finditer(line):
                hit = m.group(0)
                if _allowlisted(kind, hit):
                    continue
                hits.append((label, lineno, kind, hit))


def main():
    print('=' * 64)
    print('  脱敏自检 —— 公开仓不得含身份/位置信息')
    print('=' * 64)

    hits = []

    # ---- 1. 工作树的文本文件 ----
    n_files = 0
    for rel, body in tracked_text_files():
        if rel == SELF:
            continue
        n_files += 1
        scan_text(rel, body, hits)
    print(f'[1/4] 扫了 {n_files} 个被跟踪的文本文件')

    # ---- 2. 提交信息 ----
    # 只列**当前分支可达**的提交（`--all` 会把已删分支一起拉进来，那些不发布）。
    log = _git('log', '--all', '--format=%H%x00%B%x00%x00')
    n_commits = _git('rev-list', '--all', '--count').strip()
    scan_text('(提交信息)', log, hits)
    print(f'[2/4] 扫了 {n_commits} 条提交的信息')

    # ---- 3. 反向检查：模板里不许出现真序列号 ----
    print('[3/4] 反向检查：config/realsense_camera.yaml 必须仍是占位符')
    tmpl = os.path.join(PROJECT_ROOT, 'ros2_ws', 'src', 'yolo_bringup',
                        'config', 'realsense_camera.yaml')
    if not os.path.isfile(tmpl):
        hits.append(('ros2_ws/src/yolo_bringup/config/realsense_camera.yaml',
                     0, '文件缺失', '模板不存在 —— 相机序列号就没有约定的落点了'))
    else:
        with open(tmpl, encoding='utf-8') as fh:
            val = ''
            for line in fh:
                s = line.strip()
                if s.startswith('serial_no:'):
                    val = s.split(':', 1)[1].strip().strip('"').strip("'")
                    break
        if val and not (val.startswith('<') and val.endswith('>')):
            hits.append(('ros2_ws/src/yolo_bringup/config/realsense_camera.yaml',
                         0, '真序列号写进了模板',
                         f'serial_no={val!r} —— 序列号属设备身份，'
                         f'应放 ~/.config/ur5-vision-grasp/ 而不是仓库'))
        else:
            print(f'       serial_no={val!r} —— 是占位符，OK')

    # ---- 4. 本机可选黑名单（真实序列号放这里，不进仓库）----
    print('[4/4] 本机黑名单（可选，不入库）')
    if not os.path.isfile(DENYLIST_PATH):
        print(f'       没有 {DENYLIST_PATH} —— 跳过。'
              f'（想让它也在本机被扫到，就把真序列号一行一个放进去）')
    else:
        with open(DENYLIST_PATH, encoding='utf-8') as fh:
            terms = [t.strip() for t in fh
                     if t.strip() and not t.startswith('#')]
        for rel, body in tracked_text_files():
            for t in terms:
                if t in body:
                    hits.append((rel, 0, '本机黑名单命中', t))
        if terms:
            scan_text('(提交信息)·黑名单', log, hits)
        print(f'       黑名单 {len(terms)} 条，已一并扫过')

    # ---- 结论 ----
    print('\n' + '=' * 64)
    if not hits:
        print('干净：没有命中任何脱敏模式。')
        print('（提醒：这**不包含**图片像素 —— 新加的截图仍需人眼过一遍。）')
        return 0
    print(f'FAILED: {len(hits)} 处命中\n')
    for label, lineno, kind, hit in hits:
        where = f'{label}:{lineno}' if lineno else label
        print(f'  ✗ [{kind}] {where}\n      {hit}')
    print('\n真泄漏就改掉；本来就该在那儿的，加进本脚本白名单并写明理由。')
    return 1


if __name__ == '__main__':
    sys.exit(main())
