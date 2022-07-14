#!/usr/bin/env python3
# See argparse description
import argparse
import socket
import sys
import time

parser = argparse.ArgumentParser(
    description="Creates a TCP server and send any client whom connects the full contents of the specified binary file"
)
parser.add_argument("file", help="name of file to playback")
parser.add_argument("--port", default=10001, type=int, help="Port to bind to")
parser.add_argument("--ip", default="127.0.0.1", help="IP address to bind to")
parser.add_argument(
    "--batch_size",
    default=0,
    type=int,
    help="Number of samples to send at a time (in bytes), 0 for all available",
)
parser.add_argument(
    "--rate", default=0, type=float, help="Number of times to send a batch per second"
)
parser.add_argument(
    "--advance_on_arrow",
    default=False,
    type=bool,
    help="Advance to next batch with right arrow, back a sample with left arrow",
)
args = parser.parse_args()

TCP_IP = args.ip
TCP_PORT = args.port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

FILE = args.file
data = bytes(open(FILE, "rb").read())

if args.batch_size != 0:
    data = [
        data[i : i + args.batch_size]
        for i in range(0, len(data) - args.batch_size, args.batch_size)
    ]

wait_time = 0
if args.rate != 0:
    wait_time = 1.0 / args.rate

conn, addr = s.accept()
try:
    if args.advance_on_arrow == False:
        if args.batch_size == 0:
            while True:
                conn.send(data)
                print("sent samples")
                conn.close()
                print("Closed connection")
        elif args.batch_size != 0:
            while True:
                print("New connection")
                for sample in data:
                    conn.send(sample)
                    print("sent samples")
                    time.sleep(wait_time)
                conn.close()
                print("Closed connection")
    else:
        left = "\x1b[C"
        right = "\x1b[D"

        def get_next_sample_idx(i):
            key = input("Press Right or Left Arrow")
            while key != left and key != right:
                key = input("Press Right or Left Arrow")
            if key == right:
                print("   sent next sample")
                i = i - 1
                if i < 0:
                    return len(data) + i
                if i > len(data) - 1:
                    return i - len(data)
                else:
                    return i
            else:
                print("   sent previous sample")
                i = i + 1
                if i < 0:
                    return len(data) + i
                if i > len(data) - 1:
                    return i - len(data)
                else:
                    return i

        print(len(data))
        while True:
            print("New connection")
            i = 0
            while i < len(data):
                print("index: ", i)
                conn.send(data[i])
                print("sent samples")
                i = get_next_sample_idx(i)
            conn.close()
            print("Closed connection")

except KeyboardInterrupt:
    print("closing port")
    conn.close()
