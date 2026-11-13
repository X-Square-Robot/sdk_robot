# 机器人描述文件下载

[English](README.md) | [中文](README_CN.md)

Git 仓库按 `urdf/<机型>/` 保存各机型资源的版本、校验信息及包元数据。
URDF/STL 文件通过独立的 Release 附件分发，不进入 Git 历史。

## 机型与资源状态

| 机型 | 目录 | 当前资源 |
| --- | --- | --- |
| Quanta X1 | `quanta_x1/` | 预留，资源版本与压缩包待确认 |
| Quanta X2 | `quanta_x2/` | 预留，资源版本与压缩包待确认 |
| Quanta X3 | `quanta_x3/` | v00.05.08，13 个 URDF 变体、323 个 STL 网格 |

每个已发布机型都有独立的 `resource.json`，其中的 `release_asset`、`sha256`
和 `archive_root` 指定要下载的 ZIP、校验和及解压后的外层目录。
按机型选择资源，不能交叉使用；只有 README 的预留目录尚未登记资源。
不同机型可以使用不同的资源版本，SDK Release 的 `SHA256SUMS` 覆盖全部发布附件。

## 下载和使用 Quanta X3

从对应版本的 [SDK GitHub Release](https://github.com/X-Square-Robot/sdk_robot/releases)
下载 `quanta_x3-urdf-v00.05.08.zip` 和 `SHA256SUMS`。
资源包包含 13 个 URDF 变体和 323 个 STL 网格，请根据机器人的硬件版本、手臂和末端执行器选择。

把下载的文件放在 SDK 发布包或 GitHub 仓库根目录，在该目录执行：

```bash
sha256sum --ignore-missing -c SHA256SUMS
urdf_temp_dir="$(mktemp -d)"
unzip -q quanta_x3-urdf-v00.05.08.zip -d "$urdf_temp_dir"
mkdir -p urdf/quanta_x3
cp -a "$urdf_temp_dir/ex002-robot-v00.05.08/." urdf/quanta_x3/
rm -rf "$urdf_temp_dir"
```

ZIP 是原始 `ex002-robot-v00.05.08.zip` 的公开下载名称，包内外层目录和文件内容不变。
压缩包 SHA-256：

```text
068b1051e6297487271801bbc6cdf636e137c1417a8cfcd2dcd6a7ceec306d4e
```

URDF 与 `mesh/` 目录应一起保存，并让加载器以所选 URDF 所在目录解析 `mesh/...` 路径。
`quanta_x3/resource.json` 记录压缩包名称、上游下载地址、版本及校验和；
`package_info.json` 保留上游的各硬件变体信息。
