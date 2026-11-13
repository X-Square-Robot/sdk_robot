"""SDK2 generic upload/download example. Does not enter robot control mode."""
import argparse
import json
import uuid

import x2robot_sdk as sdk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", required=True, help="SDK server host:port")
    parser.add_argument("--timeout", type=float, default=600, help="total transfer timeout in seconds")
    actions = parser.add_subparsers(dest="action", required=True)
    upload = actions.add_parser("upload")
    upload.add_argument("source")
    upload.add_argument("--root-id", default="user_data")
    upload.add_argument("--relative-path", required=True)
    upload.add_argument("--request-id", help="reuse this ID when retrying an uncertain upload")
    download = actions.add_parser("download")
    download.add_argument("file_id", help="file ID returned by upload or recording")
    download.add_argument("destination", help="new local file; existing paths are preserved")
    args = parser.parse_args()
    connected = sdk.connect(args.address)
    if not connected.is_success:
        raise SystemExit(json.dumps(connected.error, ensure_ascii=False))
    with connected.data as robot:
        if args.action == "upload":
            request_id = args.request_id or uuid.uuid4().hex
            print(f"request_id={request_id}", flush=True)
            result = robot.files.upload_file(args.source, root_id=args.root_id,
                relative_path=args.relative_path, request_id=request_id, timeout=args.timeout)
            if result.is_success:
                print(json.dumps({"file_id": result.data.file_id, "size": result.data.total_size,
                    "md5": result.data.md5, "request_id": result.data.request_id}))
        else:
            result = robot.files.download_file(args.file_id, args.destination, timeout=args.timeout)
            if result.is_success:
                print(result.data)
        if not result.is_success:
            raise SystemExit(json.dumps(result.error, ensure_ascii=False))


if __name__ == "__main__":
    main()
