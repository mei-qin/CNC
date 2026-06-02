# CNC 项目记忆

## 审计工作流 (v2.0)
- **触发词**: "开始审计" / "audit" → 加载 `red-team-audit` 技能
- **核心原则**: 每次审计必须基于当前 Git 状态重新生成弹药包，禁止复用历史 `.audit_payload.txt`
- **弹药包生成**: `python gen_audit_prompt.py -b "业务需求" -q` → `.audit_payload.txt`
- **审查标准**: `REVIEW_PROMPT.md` (**5维清单**) + `CLAUDE.md` (架构红线)
- **5 维度**: ①实时性 ②运动学 ③内存安全 ④状态机死锁 ⑤**业务需求一致性**
- **Skill 注册**: 项目级 Skill 依赖 `.workbuddy/skills/` 目录自动发现（可能在会话启动时扫描）；若未自动加载，手动按 SKILL.md 流程执行

## 项目约定
- 开发环境 Windows，运行环境 Ubuntu Preempt-RT（禁止在 Windows 上编译运行）
- AI 代码输出必须包含 AI-Tags: `@Context`/`@Safe`/`@Danger`/`@Thread-Safety`
- 红军审计报告格式：PASS/CONDITIONAL PASS/REJECT + F/H/W 级别缺陷细分

## 目录结构
- `.workbuddy/skills/red-team-audit/SKILL.md` — 审计技能定义 (v2.0)
- `gen_audit_prompt.py` — 审计弹药包生成器 v2.0
- `REVIEW_PROMPT.md` — **5 维度**审计标准
- `CLAUDE.md` — 架构红线 + 编程规范
