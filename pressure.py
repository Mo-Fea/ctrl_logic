import argparse
import re
import socket
import time

from lib2 import tools


PRESSURE_PATTERN = re.compile(rb"pressure:(-?\d+),temp:(-?\d+)")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Continuously print latest pressure from chassis TCP feedback."
    )
    parser.add_argument(
        "--tcp-ip",
        default=tools.TCP_IP,
        help="Chassis controller TCP server IP.",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=tools.TCP_PORT,
        help="Chassis controller TCP server port.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=70.0,
        help="Control frame send frequency used to keep the TCP session alive.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Print interval in seconds.",
    )
    parser.add_argument(
        "--raw-recv-only",
        action="store_true",
        help="Only receive TCP feedback and do not start frame_thread sending.",
    )
    return parser.parse_args()


def run_raw_recv_only(args):
    with socket.create_connection((args.tcp_ip, args.tcp_port), timeout=5.0) as sock:
        sock.settimeout(None)
        buf = b""
        while True:
            data = sock.recv(1024)
            if not data:
                print("socket closed")
                break

            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                match = PRESSURE_PATTERN.fullmatch(line)
                if match is None:
                    print(f"other data: {line!r}")
                    continue

                pressure = int(match.group(1))
                temperature = int(match.group(2))
                print(f"pressure={pressure}, temp={temperature}")


def main():
    args = parse_args()
    if args.hz <= 0.0:
        raise ValueError(f"hz must be > 0, got {args.hz}")
    if args.interval <= 0.0:
        raise ValueError(f"interval must be > 0, got {args.interval}")

    if args.raw_recv_only:
        run_raw_recv_only(args)
        return

    sock = tools.connect(tcp_ip=args.tcp_ip, tcp_port=args.tcp_port)
    sender = tools.frame_thread(
        sock=sock,
        hz=args.hz,
        tcp_ip=args.tcp_ip,
        tcp_port=args.tcp_port,
    )
    sender.start()

    try:
        while True:
            status = sender.get_pressure_status()
            pressure = status.get("pressure")
            temperature = status.get("temperature")
            update_time = status.get("update_time")
            last_rx_error = status.get("last_rx_error")

            if pressure is None:
                state = sender.get_state()
                rx_byte_count = status.get("rx_byte_count", 0)
                last_rx_line = status.get("last_rx_line")
                last_unmatched_rx_line = status.get("last_unmatched_rx_line")
                if last_rx_error:
                    print(f"waiting pressure... last_rx_error={last_rx_error}")
                elif last_unmatched_rx_line is not None:
                    print(
                        "waiting pressure... "
                        f"rx_bytes={rx_byte_count}, "
                        f"last_unmatched={last_unmatched_rx_line!r}"
                    )
                elif last_rx_line is not None:
                    print(
                        "waiting pressure... "
                        f"rx_bytes={rx_byte_count}, "
                        f"last_line={last_rx_line!r}"
                    )
                else:
                    print(
                        "waiting pressure... "
                        f"send_ok={state.get('last_send_ok')}, "
                        f"seq={state.get('seq')}, "
                        f"rx_bytes={rx_byte_count}"
                    )
            else:
                age_sec = time.time() - float(update_time)
                print(
                    f"pressure={pressure}, "
                    f"temp={temperature}, "
                    f"age={age_sec:.3f}s"
                )

            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop(send_stop=True)
        tools.socket_close(sender.sock)


if __name__ == "__main__":
    main()
