#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Runs inside the bwrap sandbox: python -I -c <this source> <unix socket> <port> -- <argv...>.

The sandbox has a private network namespace, so the SDK's http://127.0.0.1:<port> lands here and
is pumped, byte for byte, to the relay's Unix socket bound into the sandbox. The command runs as a
forked child; this process exits with its status. Stdlib only: it is passed as source, not path.
"""

import os
import signal
import socket
import sys
import threading


def _pump(a, b):
    try:
        while True:
            data = a.recv(1 << 16)
            if not data:
                break
            b.sendall(data)
    except OSError:
        pass
    finally:
        try:
            b.shutdown(socket.SHUT_WR)
        except OSError:
            pass


def _serve(conn, sock_path):
    up = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        up.connect(sock_path)
    except OSError:
        conn.close()
        up.close()
        return
    back = threading.Thread(target=_pump, args=(up, conn), daemon=True)
    back.start()
    _pump(conn, up)
    back.join()
    conn.close()
    up.close()


def _accept(lsock, sock_path):
    while True:
        try:
            c, _ = lsock.accept()
        except OSError:
            continue
        threading.Thread(target=_serve, args=(c, sock_path), daemon=True).start()


def main(argv):
    sock_path, port = argv[1], int(argv[2])
    cmd = argv[argv.index("--") + 1:]
    lsock = socket.create_server(("127.0.0.1", port), backlog=64)
    pid = os.fork()
    if pid == 0:
        lsock.close()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        try:
            os.execv(cmd[0], cmd)
        finally:
            os._exit(127)
    # Interrupts are for the command (the sender signals the whole process group).
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    threading.Thread(target=_accept, args=(lsock, sock_path), daemon=True).start()
    while True:
        try:
            _, status = os.waitpid(pid, 0)
            break
        except InterruptedError:
            continue
    code = os.waitstatus_to_exitcode(status)
    os._exit(code if code >= 0 else 128 - code)


if __name__ == "__main__":
    main(sys.argv)
