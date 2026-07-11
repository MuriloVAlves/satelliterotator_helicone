#!/usr/bin/env python3
import socket, argparse

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--listen-host", default="127.0.0.1")
    p.add_argument("--listen-port", type=int, default=7777)
    args = p.parse_args()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.listen_host, args.listen_port))
    srv.listen(1)
    print(f"Dummy GS-232 logger on {args.listen_host}:{args.listen_port}")

    while True:
        print("Waiting for connection...")
        conn, addr = srv.accept()
        print("GPredict connected from", addr)
        with conn:
            while True:
                data = conn.recv(256)
                if not data:
                    break
                cmd = data.decode("ascii", errors="ignore").strip()
                print("→ SatDump:", repr(cmd))
                # Minimal echo responses so GS-232 stays happy
                if cmd.upper().startswith("AZ"):
                    resp = cmd.upper()  # echo back same
                elif cmd.upper().startswith("EL"):
                    resp = cmd.upper()
                else:
                    resp = cmd.upper()
                print("← Dummy:", repr(resp))
                conn.sendall((resp + "\r").encode("ascii"))
        print("GPredict disconnected")

if __name__ == "__main__":
    print("Script started")
    main()
