"""Mapping, localization and streamed map upload/download example."""

import argparse
import io
import tarfile
from pathlib import Path

import x2robot_sdk
from x2robot.sdk import (
    DownloadRequest,
    FileTransferMeta,
    NavigationMode,
    NavigationModeParam,
    SaveMapParam,
    StartLocalizationParam,
    UploadRequest,
)

from x2robot.geometry_msgs import Pose, Point, Quaternion


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def map_archive(map_name, source=None):
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode="w:gz") as archive:
        source_path = Path(source) if source else None
        if source_path is not None and source_path.is_dir():
            for child in source_path.rglob("*"):
                if child.is_file():
                    archive.add(str(child), arcname=f"{map_name}/{child.relative_to(source_path)}")
        else:
            for name, content in (
                (f"{map_name}/map.yaml", b"image: map.pgm\nresolution: 0.05\n"),
                (f"{map_name}/map.pgm", b"P5\n1 1\n255\n\x00"),
            ):
                item = tarfile.TarInfo(name)
                item.size = len(content)
                archive.addfile(item, io.BytesIO(content))
    return output.getvalue()


def _upload(robot, map_name: str, source=None) -> None:
    archive = map_archive(map_name, source)
    require(
        robot.navigation.load_map(
            [
                UploadRequest(
                    meta=FileTransferMeta(
                        identifier=f"{map_name}.tar.gz",
                        total_size=len(archive),
                        extra={"map_name": map_name},
                    )
                ),
                UploadRequest(chunk=archive),
            ],
            timeout=15.0,
        ),
        "Navigation.LoadMap",
    )
    print(f"uploaded {map_name} ({len(archive)} bytes)")


def _export(robot, map_name: str, output: str | None) -> None:
    stream = robot.navigation.export_map(
        DownloadRequest(identifier=f"{map_name}.tar.gz", extra={"map_name": map_name}),
        timeout=15.0,
    )
    chunks: list[bytes] = []
    try:
        for result in stream:
            response = require(result, "Navigation.ExportMap")
            if response.chunk is not None:
                chunks.append(bytes(response.chunk))
    finally:
        close_stream(stream)
    payload = b"".join(chunks)
    if not payload:
        raise RuntimeError("Navigation.ExportMap returned no bytes")
    if output:
        path = Path(output)
        path.mkdir(parents=True, exist_ok=True)
        with tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz") as archive:
            archive.extractall(path)
        print(f"extracted {map_name} into {path} ({len(payload)} bytes)")
    else:
        print(f"downloaded {map_name} ({len(payload)} bytes)")


def _actions(robot, args) -> None:
    map_name = args.map_name
    if args.action == "load":
        _upload(robot, map_name, args.map_path)
    elif args.action == "export":
        _export(robot, map_name, args.output)
    elif args.action == "list":
        print(require(robot.navigation.get_map_list(timeout=3.0), "Navigation.GetMapList").map_list)
    elif args.action == "mapping":
        require(robot.navigation.start_mapping(timeout=5.0), "Navigation.StartMapping")
        require(robot.navigation.stop_mapping(SaveMapParam(map_name=map_name), timeout=5.0), "Navigation.StopMapping")
        print(f"saved map {map_name}")
    elif args.action == "localization":
        pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(w=1.0))
        require(robot.navigation.start_localization(
            StartLocalizationParam(map_name=map_name, use_init_pose=True, init_pose=pose),
            timeout=5.0,
        ), "Navigation.StartLocalization")
        print(require(robot.navigation.get_navigation_status(timeout=3.0), "Navigation.GetNavigationStatus"))
        require(robot.navigation.cancel_navigation(timeout=3.0), "Navigation.CancelNavigation")
        require(robot.navigation.stop_localization(timeout=3.0), "Navigation.StopLocalization")


def _arguments(parser) -> None:
    parser.add_argument("--map-name", default="sdk_example2_map")
    parser.add_argument("--output", help="archive path for the export action")


def _verify_safe_reads(robot) -> None:
    """Verify navigation reads without issuing a state-changing command.

    Quanta X3 now exposes the navigation write RPCs.  The aggregate verifier
    still limits its automatic check to map-list and status reads so running
    ``verify`` cannot start mapping/localization or alter a map on a real robot.
    The explicit ``mapping``, ``localization``, ``load`` and ``export`` actions
    remain available for an operator-controlled check.
    """
    maps = require(robot.navigation.get_map_list(timeout=3.0), "Navigation.GetMapList")
    status = require(
        robot.navigation.get_navigation_status(timeout=3.0),
        "Navigation.GetNavigationStatus",
    )
    print(f"maps={maps.map_list if maps else []}")
    print(f"navigation_status={getattr(status, 'json', status)}")


def verify(robot, _model: str = "auto") -> None:
    if _model == "quanta_x3":
        _verify_safe_reads(robot)
        return

    map_name = "sdk_example2_map"
    require(robot.navigation.start_mapping(timeout=5.0), "Navigation.StartMapping")
    require(
        robot.navigation.stop_mapping(SaveMapParam(map_name=map_name), timeout=5.0),
        "Navigation.StopMapping",
    )
    require(
        robot.navigation.set_navigation_mode(
            NavigationModeParam(mode=NavigationMode.BUILT_IN_NAVIGATION), timeout=3.0
        ),
        "Navigation.SetNavigationMode",
    )
    pose = Pose(position=Point(x=0.1, y=0.0, z=0.0), orientation=Quaternion(w=1.0))
    require(
        robot.navigation.start_localization(
            StartLocalizationParam(map_name=map_name, use_init_pose=True, init_pose=pose),
            timeout=5.0,
        ),
        "Navigation.StartLocalization",
    )
    require(robot.navigation.get_navigation_status(timeout=3.0), "Navigation.GetNavigationStatus")
    require(robot.navigation.cancel_navigation(timeout=3.0), "Navigation.CancelNavigation")
    require(robot.navigation.stop_localization(timeout=3.0), "Navigation.StopLocalization")

    archive = map_archive(map_name)
    upload = [
        UploadRequest(
            meta=FileTransferMeta(
                identifier=f"{map_name}.tar.gz",
                total_size=len(archive),
                extra={"map_name": map_name},
            )
        ),
        UploadRequest(chunk=archive),
    ]
    require(robot.navigation.load_map(upload, timeout=15.0), "Navigation.LoadMap")

    stream = robot.navigation.export_map(
        DownloadRequest(identifier=f"{map_name}.tar.gz", extra={"map_name": map_name}),
        timeout=15.0,
    )
    chunks: list[bytes] = []
    try:
        for result in stream:
            response = require(result, "Navigation.ExportMap")
            if response.chunk is not None:
                chunks.append(bytes(response.chunk))
    finally:
        close_stream(stream)
    if not b"".join(chunks):
        raise AssertionError("Navigation.ExportMap returned no bytes")

    maps = require(robot.navigation.get_map_list(timeout=3.0), "Navigation.GetMapList")
    if map_name not in maps.map_list:
        raise AssertionError(f"uploaded map {map_name!r} is absent from map list")
    print(f"maps={maps.map_list}, downloaded={sum(map(len, chunks))} bytes")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", choices=("load", "export", "list"),
                        help="v1-compatible subcommand")
    parser.add_argument("command_args", nargs="*", help="load: MAP_NAME MAP_PATH; export: MAP_NAME TARGET_PATH")
    parser.add_argument("--server", default="127.0.0.1:55051")
    parser.add_argument("--action", choices=("verify", "load", "export", "list", "mapping", "localization"), default="list")
    parser.add_argument("--map-name", default="sdk_example2_map")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    if args.command:
        args.action = args.command
        if args.command == "load":
            if len(args.command_args) != 2:
                parser.error("load requires MAP_NAME MAP_PATH")
            args.map_name, args.map_path = args.command_args
        elif args.command == "export":
            if len(args.command_args) != 2:
                parser.error("export requires MAP_NAME TARGET_PATH")
            args.map_name, args.output = args.command_args
        else:
            if args.command_args:
                parser.error("list takes no positional arguments")
    else:
        args.map_path = None
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot, getattr(robot, "model_name", "auto"))
        else:
            _actions(robot, args)
        print("PASS: navigation")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
