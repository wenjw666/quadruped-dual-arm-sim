#!/usr/bin/env python3
"""Password-prompting SSH/SFTP helper for the C1200 board."""
from __future__ import annotations

import argparse
import getpass
import os
import posixpath
import sys
from pathlib import Path

import paramiko


def connect(args):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    password = os.environ.get("C1200_PASSWORD") or getpass.getpass(
        f"Password for {args.user}@{args.host}: "
    )
    client.connect(args.host, port=args.port, username=args.user, password=password,
                   timeout=args.timeout, banner_timeout=args.timeout,
                   auth_timeout=args.timeout)
    return client


def run(args):
    client = connect(args)
    try:
        _, stdout, stderr = client.exec_command(args.command, timeout=args.timeout)
        out = stdout.read().decode("utf-8", "replace")
        err = stderr.read().decode("utf-8", "replace")
        if out:
            print(out, end="")
        if err:
            print(err, end="", file=sys.stderr)
        return stdout.channel.recv_exit_status()
    finally:
        client.close()


def put_one(sftp, local: Path, remote: str):
    if local.is_dir():
        try:
            sftp.mkdir(remote)
        except OSError:
            pass
        for child in local.iterdir():
            put_one(sftp, child, posixpath.join(remote, child.name))
        return
    parent = posixpath.dirname(remote)
    if parent:
        current = ""
        for part in parent.strip("/").split("/"):
            current += "/" + part
            try:
                sftp.mkdir(current)
            except OSError:
                pass
    sftp.put(str(local), remote)


def put(args):
    client = connect(args)
    try:
        sftp = client.open_sftp()
        put_one(sftp, Path(args.local).resolve(), args.remote)
        sftp.close()
        print(f"uploaded {args.local} -> {args.user}@{args.host}:{args.remote}")
        return 0
    finally:
        client.close()


def put_many(args):
    client = connect(args)
    try:
        sftp = client.open_sftp()
        for pair in args.pairs:
            local, remote = pair.split("=", 1)
            put_one(sftp, Path(local).resolve(), remote)
            print(f"uploaded {local} -> {args.user}@{args.host}:{remote}")
        sftp.close()
        return 0
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.1.100")
    parser.add_argument("--user", default="bst")
    parser.add_argument("--port", type=int, default=22)
    parser.add_argument("--timeout", type=float, default=20.0)
    sub = parser.add_subparsers(dest="op", required=True)
    p = sub.add_parser("run")
    p.add_argument("command")
    p.set_defaults(func=run)
    p = sub.add_parser("put")
    p.add_argument("local")
    p.add_argument("remote")
    p.set_defaults(func=put)
    p = sub.add_parser("put-many")
    p.add_argument("pairs", nargs="+", metavar="LOCAL=REMOTE")
    p.set_defaults(func=put_many)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
