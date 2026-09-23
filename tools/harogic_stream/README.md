# SAN-60 spectrum stream

This standalone path transports processed spectrum traces rather than raw IQ data:

```text
SAN-60 USB -> sender PC 192.168.10.5
           -> TX 192.168.10.31 / 192.168.20.31
           -> semantic-box link
           -> RX 192.168.20.32 / 192.168.10.32
           -> receiver PC 192.168.10.10 -> Tkinter spectrum display
```

Defaults are intentionally scoped to the test network:

- sender connects only to TX `192.168.10.31:5000`;
- TX accepts only `192.168.10.5`, then connects from `192.168.20.31` to
  RX `192.168.20.32:5000`;
- RX accepts only `192.168.20.31`, then connects from `192.168.10.32` to
  receiver `192.168.10.10:5000`;
- receiver accepts only source `192.168.10.32`;
- sweep range is 2.4--2.5 GHz with 100 kHz RBW at 2 FPS.

Receiver:

```bash
python3 spectrum_receiver.py
```

TX relay:

```bash
python3 tcp_relay.py --name TX \
  --listen-host 192.168.10.31 --allow-source 192.168.10.5 \
  --source-host 192.168.20.31 --target-host 192.168.20.32
```

RX relay:

```bash
python3 tcp_relay.py --name RX \
  --listen-host 192.168.20.32 --allow-source 192.168.20.31 \
  --source-host 192.168.10.32 --target-host 192.168.10.10
```

Sender:

```bash
chmod +x build_capture.sh
./build_capture.sh
python3 harogic_sender.py
```

To show the same frames on the sender PC as well, start a local receiver on port
5001 and add `--local-preview-port 5001` to the sender command.

Frequency parameters can be changed without editing code, for example:

```bash
python3 harogic_sender.py --start-hz 1e9 --stop-hz 2e9 --rbw-hz 500e3 --fps 1
```

Only one process may own the USB analyzer at a time. Close SAStudio before starting
the sender, and stop the sender before opening SAStudio again.

The native C++ collector is intentional: the vendor Python `ctypes` example is not
ABI-stable with the currently installed SAN/SDK combination, while the vendor C++
API completes continuous sweeps reliably.
