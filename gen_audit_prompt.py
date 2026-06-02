# -*- coding: utf-8 -*-
"""
CNC Red Team Auditor — 审计弹药包生成器 v2.0

用法:
  # 交互模式（提示输入业务需求）
  python gen_audit_prompt.py

  # 非交互模式（Skill 自动化调用）
  python gen_audit_prompt.py -b "升级为7段S曲线加减速"

  # 指定输出文件
  python gen_audit_prompt.py -b "修复短板限幅" -o custom_payload.txt

  # 仅提取 diff 不读完整文件（快速预览）
  python gen_audit_prompt.py --diff-only

输出:
  .audit_payload.txt — 包含 diff + 完整源码上下文，供红军审计使用
"""
import os
import sys
import argparse
import subprocess


def run_cmd(cmd, cwd=None):
    """安全执行 shell 命令并返回 stdout."""
    try:
        return subprocess.run(
            cmd, capture_output=True, text=True,
            encoding='utf-8', errors='ignore', cwd=cwd
        ).stdout.strip()
    except Exception:
        return ""


def is_source_file(fname):
    """仅收集 C 源码和头文件."""
    return fname.endswith('.c') or fname.endswith('.h')


def collect_modified_files(cwd=None):
    """从 git 收集所有修改/新增的源文件."""
    raw = (
        run_cmd(['git', 'diff', '--cached', '--name-only'], cwd) + "\n" +
        run_cmd(['git', 'diff', '--name-only'], cwd) + "\n" +
        run_cmd(['git', 'ls-files', '--others', '--exclude-standard'], cwd) + "\n"
    )
    all_files = set(f.strip() for f in raw.split('\n') if f.strip())
    # 始终包含关键全局文件（即使未被修改）
    mandatory = {'inc/axis_cfg.h', 'inc/global_def.h'}
    return sorted(all_files | {f for f in mandatory if os.path.exists(
        os.path.join(cwd, f) if cwd else f)})


def build_diff_block(cwd=None):
    """构建 diff 文本块（暂存区 + 工作区）."""
    blocks = []
    staged = run_cmd(['git', 'diff', '--cached'], cwd)
    if staged:
        blocks.append("--- 暂存区 (Staged) 修改 ---\n" + staged)
    unstaged = run_cmd(['git', 'diff'], cwd)
    if unstaged:
        blocks.append("--- 工作区 (Unstaged) 修改 ---\n" + unstaged)
    return "\n\n".join(blocks)


def build_context_block(files, cwd=None):
    """读取所有修改文件的完整源码."""
    blocks = []
    for file in files:
        if not is_source_file(file):
            continue
        filepath = os.path.join(cwd, file) if cwd else file
        if not os.path.exists(filepath):
            continue
        try:
            with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
                content = f.read()
        except Exception:
            content = f"[ERROR: 无法读取文件 {filepath}]"
        blocks.append(
            f"\n{'='*40}\n"
            f"--- 完整文件上下文: {file} ---\n"
            f"{'='*40}\n"
            f"{content}\n"
        )
    return "\n".join(blocks)


def main():
    parser = argparse.ArgumentParser(
        description="CNC 审计弹药包生成器 — 提取 git diff + 源文件上下文"
    )
    parser.add_argument('-b', '--business-req', default=None,
                        help='业务需求描述（非交互模式）')
    parser.add_argument('-o', '--output', default='.audit_payload.txt',
                        help='输出文件路径 (默认: .audit_payload.txt)')
    parser.add_argument('--diff-only', action='store_true',
                        help='仅输出 diff，不读取完整源文件')
    parser.add_argument('--cwd', default=None,
                        help='Git 仓库根目录 (默认: 当前目录)')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='静默模式，不打印提示信息')
    args = parser.parse_args()

    cwd = args.cwd

    # 验证在 git 仓库中
    if not run_cmd(['git', 'rev-parse', '--git-dir'], cwd):
        print("❌ 错误：当前目录不是 Git 仓库，无法提取 diff。", file=sys.stderr)
        sys.exit(1)

    # 获取业务需求
    if args.business_req is not None:
        feature_goal = args.business_req
    else:
        try:
            feature_goal = input("👉 请简述本次修改期望实现的功能（按回车跳过）：\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            feature_goal = ""

    # 收集 diff
    diff = build_diff_block(cwd)
    if not diff:
        print("未检测到 Git 修改（工作区和暂存区均为空），退出。")
        sys.exit(0)

    if not args.quiet:
        staged_count = len(run_cmd(['git', 'diff', '--cached', '--name-only'], cwd).split('\n')) if run_cmd(['git', 'diff', '--cached', '--name-only'], cwd) else 0
        print(f"📋 检测到暂存区 {staged_count} 个文件变更")

    # 收集修改文件列表
    files = collect_modified_files(cwd)
    src_files = [f for f in files if is_source_file(f)]
    if not args.quiet:
        print(f"📄 涉及 {len(src_files)} 个源文件: {', '.join(src_files[:5])}"
              f"{'...' if len(src_files) > 5 else ''}")

    # 构建上下文
    context = ""
    if not args.diff_only:
        context = build_context_block(files, cwd)
        if not args.quiet:
            print(f"📖 已读取 {sum(1 for f in files if is_source_file(f))} 个源文件完整内容")

    # 组装弹药包
    payload = f"""请你对我提交的以下代码修改进行极限安全审计。

【核心目标】：本次修改的业务需求是："{feature_goal if feature_goal else '通用安全审计'}"。

【被修改的核心差异对比 (Git Diff)】:
{diff}

【涉及的完整文件源码 (上帝视角上下文)】:
{context}
"""
    output_path = os.path.join(cwd, args.output) if cwd else args.output
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(payload)

    if not args.quiet:
        size_kb = os.path.getsize(output_path) / 1024
        print(f"✅ 审计弹药包已生成 → {output_path} ({size_kb:.1f} KB)")


if __name__ == "__main__":
    main()
