"""
Small SSH upload test for LIMO.

Uploads a local file (default: ../fleet_config.yaml) to a remote LIMO directory.
"""
import argparse
import os
import posixpath
import sys

import paramiko


def resolve_local_path(path_arg: str) -> str:
    if os.path.isabs(path_arg):
        return path_arg
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(script_dir, path_arg))


def main() -> int:
    parser = argparse.ArgumentParser(description="SSH upload test for LIMO")
    parser.add_argument("--host", default="192.168.137.175", help="LIMO IP")
    parser.add_argument("--port", type=int, default=22, help="SSH port")
    parser.add_argument("--username", default="agilex", help="SSH username")
    parser.add_argument("--password", default="agx", help="SSH password")
    parser.add_argument(
        "--local-file",
        default="../fleet_config.yaml",
        help="Local file to upload (absolute or relative to this script)",
    )
    parser.add_argument(
        "--remote-dir",
        default="/home/agilex/agilex_ws/src/limo_nav_huy_test/limo_nav_huy_test",
        help="Remote directory on LIMO",
    )
    parser.add_argument(
        "--remote-name",
        default="fleet_config_test.yaml",
        help="Remote filename",
    )
    parser.add_argument("--timeout", type=int, default=12, help="SSH timeout in seconds")
    args = parser.parse_args()

    local_file = resolve_local_path(args.local_file)
    if not os.path.exists(local_file):
        print(f"[ERR] Local file not found: {local_file}")
        return 1

    remote_dir = args.remote_dir.rstrip("/")
    remote_file = posixpath.join(remote_dir, args.remote_name)

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        print(f"[INFO] Connecting to {args.username}@{args.host}:{args.port} ...")
        ssh.connect(
            hostname=args.host,
            port=args.port,
            username=args.username,
            password=args.password,
            timeout=args.timeout,
        )

        ssh.exec_command(f"mkdir -p '{remote_dir}'")
        sftp = ssh.open_sftp()
        try:
            sftp.put(local_file, remote_file)
            print(f"[OK] Uploaded:\n  {local_file}\n  -> {remote_file}")
        finally:
            sftp.close()

        _, stdout, _ = ssh.exec_command(f"ls -l '{remote_file}'")
        print("[INFO] Remote file:")
        print(stdout.read().decode("utf-8", errors="replace").strip())
        return 0

    except Exception as exc:
        print(f"[ERR] Upload failed: {exc}")
        return 1
    finally:
        ssh.close()


if __name__ == "__main__":
    sys.exit(main())
