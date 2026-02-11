# Agent Docs Index

## Purpose
- 供 AI 代理在本仓库执行任务时读取的最小治理文档。
- 仅描述执行约束、验证要求与 workflow 索引。

## Global Rules
- 保持最小必要改动，优先增量实现，避免重构核心链路。
- 涉及地图渲染时，禁止使用会清空全图对象的方式替代局部清理。
- 新增功能默认关闭，避免改变既有用户行为。
- 变更完成后必须执行并通过：`./gradlew lint`、`./gradlew testDebugUnitTest`。

## Workflows
- `workflow/260211-dynamic-trajectory-color.md`：动态轨迹颜色（基于 Wi-Fi RSSI）叠加层实现流程。
