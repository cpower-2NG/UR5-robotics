# RESTRUCTURE_PLAN —— 已移出公开仓（留桩）

> **层级**：流水 · 这里原来放本仓的**过程台账**（126 轮逐轮证据，17860 行）。
> **2026-10-08 按「公开仓只留恒久 / 意图 / 状态三层」的决定移出仓库**，
> 原内容**一字未改**，现在在仓库外：

```
~/robotics/notes/ur5-vision-grasp/RESTRUCTURE_PLAN.md
```

**为什么留这个桩**：全仓有 17 份文档与脚本注释在引用它（形如
`见 RESTRUCTURE_PLAN.md 第五十轮`）。桩让那些引用**仍然可达** —— 顺着桩走一步
就到正文，不需要改那 17 处，也不会出现断链。

**怎么检索它**（不要通读 —— 它是一份只增不减的流水）：

```
grep -n "第五十轮" ~/robotics/notes/ur5-vision-grasp/RESTRUCTURE_PLAN.md
grep -n "空夹\|外参" ~/robotics/notes/ur5-vision-grasp/RESTRUCTURE_PLAN.md | tail -30
```

> ⚠️ **台账是过程记录**：里面的缺陷编号、参数值、结论都对应**当时那一刻**。
> **引用前回代码核对**。「这台系统现在什么样」一律看
> [当前状态.md](当前状态.md)。

> ⚠️ 它**不再随本仓分发**。那份外部文件已单独纳入版本控制
> （`~/robotics/notes/ur5-vision-grasp/` 是一个独立的 git 仓）；本仓的历史里
> 仍保留 2026-10-08 之前的全部台账正文，可用 `git show <commit>:docs/RESTRUCTURE_PLAN.md` 取回。
