# SEI Artifact Amendments

冻结纪律（FREEZE_MANIFEST.json declaration）：追加允许；冻结文件只经
amendment 条目修改。本文件为 amendment 登记处，冻结件 byte 不变。

## Amendment 1（2026-09-30，术语映射声明——不改动任何冻结文件）

**事项**：artifact 说明文字与 replay 脚本文案中存在历史措辞
"natural-defect closure" / "natural-defect repair/revert"
（README.md :25/:93/:98、replay/replay_f1.py :3/:138、run_all.py :18）。

**声明**：以上均为早期内部工作措辞。正式论文术语统一为：

| 历史措辞 (冻结件内保留) | 正式术语 (论文/引用时使用) |
|---|---|
| natural-defect closure | causal closure of a naturally occurring diagnostic-witness propagation inconsistency |
| natural-defect repair/revert | minimal source intervention / relation restored / revert-based causal confirmation |
| repair validation | intervention validation |

**性质**：F-1 是 naturally occurring 的见证传播不一致（非注入、非人造），
其干预是 minimal source intervention；"defect/repair" 字样不用于论文 claim。
冻结文件（含上述措辞出现的 README、replay 脚本文案字符串）byte 不变；
本 amendment 为唯一权威术语映射，论文与外部材料引用 artifact 时以本表为准。

（B3 正式批先例：措辞类问题以 amendment 收口、冻结件 byte 不变，
见 b3_formal_20260928 处理记录。）
