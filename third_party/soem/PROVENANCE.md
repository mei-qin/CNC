# SOEM 源码来源 (vendored)

- 上游: https://github.com/OpenEtherCATsociety/SOEM
- 提交: `2f73eaa` (fix(win32): detect CreateThread failure and pass correct
  handle to SetThreadPriority), 工作树干净、零本地补丁, 2026-11-24 入库
- 许可: GPLv3 / 商业双授权, 全文见本目录 `LICENSE.md`
  (本仓库为科研/教学用途; 商用需向上游购买商业授权)

## 入库范围 (linux 构建最小子集)

```
src/*.c                  EtherCAT 主站核心 (9 文件, 与上游 soem 库目标一致)
include/soem/            静态头文件 + 生成文件 ec_options.h (见下)
osal/osal.h              OS 抽象层接口
osal/linux/              linux OSAL 实现 (osal.c + osal_defs.h)
oshw/linux/              网卡驱动层 (nicdrv + oshw)
LICENSE.md  README.md    上游原样
```

裁剪项 (与本仓库 Linux 构建无关, 需要时从上游取):
`.git/` `build/` `samples/` `contrib/` `scripts/` `cmake/` `CMakeLists.txt`
`CMakePresets.json` `osal/{rtk,win32}/` `oshw/{rtk,win32}/`

## ec_options.h 说明

上游该文件是 CMake 模板 (`include/soem/ec_options.h.in`) 生成的构建配置。
此处**原样固化**联调现场 `/home/meiqin/SOEM/build` 生成的那份 (默认值,
含 `EC_PRIMARY_MAC "01:01:01:01:01:01"`), 使本仓库 `make` 不依赖 cmake。
如需改配置: 从上游 cmake 重新生成后覆盖本文件, 并在此记录差异。

## 构建方式

`Makefile` 直接源内编译本子集 (与 `third_party/open62541` 同先例),
不再依赖 `SOEM_PATH` 指向外部预编译 `libsoem`。环境变量 `SOEM_PATH`
仍可覆盖 (如临时指回外部源码树对比验证)。

## 更新策略

升级上游时: 重新拷贝上述范围 + 重新生成 ec_options.h, 更新本文件提交号,
并在 WSL2 重跑 `scripts/check_p1_limits.py` 与实机冒烟后再合入。
