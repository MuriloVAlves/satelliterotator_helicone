#!/usr/bin/env python3
import socket, argparse
from time import time, sleep

TIMEOUT = 1
MAX_RETRIES = 100

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--listen-host", default="127.0.0.1")
    p.add_argument("--listen-port", type=int, default=4533)
    args = p.parse_args()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.listen_host, args.listen_port))
    srv.listen(1)
    srv.settimeout(TIMEOUT)
    print(f"Dummy GS-232 logger on {args.listen_host}:{args.listen_port}")
    last_az = "0"
    last_el = "0"
    try:
        timeout_counter = 0
        # fake values
        az = "179.9"
        el = "53.8"
        while True:
            try:
                conn, addr = srv.accept()
                timeout_counter = 0
                print(f"GPredict connected from {addr[0]}:{addr[1]}")
                with conn:
                    while True:
                        data = conn.recv(256)
                        if not data:
                            break
                        cmd = data.decode("ascii", errors="ignore").strip()
                        print(time(),"→ GPredict:", repr(cmd))
                        sleep(0.5)

                        if cmd == 'p':
                            resp = f"{float(last_az):.16f} {float(last_el):.16f}"
                        elif cmd[0] == 'P':
                            _,az,el = cmd.split(' ')
                            resp = f"P {az} {el}"
                            last_az = az
                            last_el = el
                        elif cmd[0].upper() == 'S':
                            resp = "S0"
                        elif cmd[0].upper() == 'Q':
                            # no reply on Q, just break out
                            print("← Dummy: <closing>")
                            break
                        else:
                            # for AZ/EL set commands and others we just acknowledge
                            resp = cmd.upper()

                        print(time(),"← Dummy:", repr(resp))
                        conn.sendall((resp + "\r\n").encode("ascii"))

                print("GPredict disconnected")
            except TimeoutError:
                timeout_counter += 1
                print(f"\rTry: {timeout_counter} - No connection received within {TIMEOUT} seconds. Retrying...",end="")
                if timeout_counter >= MAX_RETRIES:
                    print("Exiting program...")
                    exit(1)
    except KeyboardInterrupt:
        print("\nStopping the server manually...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    main()
