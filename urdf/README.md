# Robot description downloads

[English](README.md) | [中文](README_CN.md)

Git stores each model's resource version, checksums and package metadata under
`urdf/<model>/`. The URDF/STL files are distributed as a separate Release asset,
not stored in Git history.

## Models and resource availability

| Model | Directory | Current resource |
| --- | --- | --- |
| Quanta X1 | `quanta_x1/` | Reserved; resource version and archive pending confirmation |
| Quanta X2 | `quanta_x2/` | Reserved; resource version and archive pending confirmation |
| Quanta X3 | `quanta_x3/` | v00.05.08, 13 URDF variants and 323 STL meshes |

Each published model has its own `resource.json`. Its `release_asset`, `sha256`
and `archive_root` identify the download, checksum and extracted top directory.
Choose the resource for your model; do not substitute another model's files.
A README-only directory is a placeholder, not a published resource.
Model resource versions are independent. The SDK Release's `SHA256SUMS` covers
all published attachments.

## Download and use Quanta X3

Download `quanta_x3-urdf-v00.05.08.zip` and `SHA256SUMS` from the matching
[SDK GitHub Release](https://github.com/X-Square-Robot/sdk_robot/releases).
This archive contains 13 URDF variants and 323 STL meshes. Select the variant
matching the robot's hardware revision, arms and end effectors.

From the SDK package or GitHub checkout root, with the downloaded files there:

```bash
sha256sum --ignore-missing -c SHA256SUMS
urdf_temp_dir="$(mktemp -d)"
unzip -q quanta_x3-urdf-v00.05.08.zip -d "$urdf_temp_dir"
mkdir -p urdf/quanta_x3
cp -a "$urdf_temp_dir/ex002-robot-v00.05.08/." urdf/quanta_x3/
rm -rf "$urdf_temp_dir"
```

The ZIP is the original `ex002-robot-v00.05.08.zip`, renamed for public download;
its outer directory and contents are unchanged. Its SHA-256 is:

```text
068b1051e6297487271801bbc6cdf636e137c1417a8cfcd2dcd6a7ceec306d4e
```

Keep each URDF with its `mesh/` directory. Configure the loader to resolve
`mesh/...` paths relative to the selected URDF. `quanta_x3/resource.json` records
the archive filename, upstream download URL, version and expected checksum;
`package_info.json` preserves upstream variant metadata.
