#!/usr/bin/env python3
"""sim_bridge 协议自检（不开 ROS、不开 socket）：process_line 的三条契约。

2026-11-14 审计补洞的钉子：
1. **任何一行都得到回包** —— 畸形 JSON 时旧实现 ``record(req, ...)`` 抛
   NameError 打死客户端线程且无回包（record 只捕 OSError）；
2. dispatch 抛错 ⇒ 错误回包 + 进 req/rep 日志（req 已解析）；
3. 正常请求 ⇒ dispatch 结果原样回 + 记日志。
缺 rclpy（import sim_bridge 要 ROS 环境）时整体 SKIP —— 明确报出，不装跑过。
"""
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

PASS = FAIL = SKIP = 0
FAILS = []


def check(name, ok, detail=""):
    global PASS, FAIL
    if ok:
        PASS += 1
        print("  PASS  %s" % name)
    else:
        FAIL += 1
        FAILS.append(name)
        print("  FAIL  %s   ← %s" % (name, detail))


try:
    import sim_bridge
except ImportError as e:
    print("SKIP 全部：sim_bridge 需要 ROS 环境（%s）" % e)
    print("0 PASS  0 FAIL  1 SKIP")
    sys.exit(0)


class FakeNode:
    """只实现 process_line 用到的两个面：dispatch / record。"""

    def __init__(self, boom=False):
        self.boom = boom
        self.recorded = []

    def dispatch(self, req):
        if self.boom:
            raise RuntimeError("dispatch 内部炸（测试注入）")
        return {"ok": True, "echo": req}

    def record(self, req, rep):
        self.recorded.append((req, rep))


def main():
    print("=" * 68)
    node = FakeNode()
    req, rep = sim_bridge.process_line(node, '{"op": "ping"}')
    check("正常请求：回 dispatch 结果 + req 已解析",
          req == {"op": "ping"} and rep.get("ok") is True, (req, rep))
    check("正常请求：进 record", len(node.recorded) == 1, node.recorded)

    node2 = FakeNode(boom=True)
    req, rep = sim_bridge.process_line(node2, '{"op": "move"}')
    check("dispatch 抛错 ⇒ 错误回包（带异常类型与文案）",
          req is not None and rep.get("ok") is False
          and "RuntimeError" in str(rep.get("error"))
          and "测试注入" in str(rep.get("error")), (req, rep))
    check("dispatch 抛错 ⇒ 仍进 record（req/rep 都在）",
          len(node2.recorded) == 1, node2.recorded)

    node3 = FakeNode()
    req, rep = sim_bridge.process_line(node3, "{不是 JSON")
    check("畸形 JSON ⇒ 也回错误包（不炸线程、客户端看得见失败）",
          req is None and rep.get("ok") is False, (req, rep))
    check("畸形 JSON ⇒ 不进 record（没解析出来的请求不占日志）",
          node3.recorded == [], node3.recorded)

    print("\n%d PASS  %d FAIL" % (PASS, FAIL))
    if FAILS:
        print("失败项：")
        for f in FAILS:
            print("  - %s" % f)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
