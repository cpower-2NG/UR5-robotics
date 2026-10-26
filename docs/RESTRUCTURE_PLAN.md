# RESTRUCTURE_PLAN.md（**桩** —— 正身已移出仓库）

> **层级**：流水 · 本文件只是**指路桩**，不是台账。
> **移动 + 留桩，禁止复制**（[`../AGENTS.md`](../AGENTS.md) §3，2026-10-26 v2 迁移落桩）。

## 正身在哪

**`~/robotics/notes/ur5-vision-grasp/RESTRUCTURE_PLAN.md`**（独立 git 仓，151 轮起持续追加，1–150 轮正身也在）。

查历史**只按轮 grep，禁止整读**：

```bash
grep -n "第一百四十九轮" ~/robotics/notes/ur5-vision-grasp/RESTRUCTURE_PLAN.md
```

## 本仓历史里的旧副本

移出前的旧副本全文（**94 轮，停在 2026-10-05**，之后的内容只在仓外正身）仍可从 git 历史取：

```bash
git log --oneline -- docs/RESTRUCTURE_PLAN.md | head -3   # 找移出前最后一个提交
git show <commit>:docs/RESTRUCTURE_PLAN.md
```

## 给引用方的提示

全仓「见台账第 N 轮」类引用：**N ≤ 94** 时本仓旧副本与仓外正身都能查（内容一致）；
**N > 94** 只在仓外正身。别在本仓再新建指向台账正文的复述 —— 指路即可。
