# Motherboard USB CDC protocol

USB CDC uses ASCII lines terminated by `\n`. The receive interrupt only copies
bytes to a ring buffer; commands are parsed and executed from the main loop.

## Commands

```text
AIR,1
AIR,0
AIROUT,1.250
DROP,0.080
STREAM,START,20
STREAM,STOP
PING
```

- `AIR` explicitly enables or disables the ITV air valve.
- `AIROUT` selects direct PC control and accepts `0.000...5.000 V`.
- `DROP` sets the burst detection threshold (`0.020...0.500 V`).
- `STREAM,START` enables telemetry; period is clamped to `20...1000 ms`.
- `PING` refreshes the remote-control watchdog.

The application repeats `AIROUT` at least every 200 ms. If no remote-control
command arrives for 750 ms, the controller sets `AIROUT` to zero and closes the
air valves.

## Telemetry

```text
TEL,0.842,1.500,1.250,1,1
```

Fields:

1. `ITV_PRESSURE`, volts;
2. `REG_ITV`, volts;
3. `AIROUT`, volts;
4. `bigairon`, `0/1`;
5. pressure sensor validity, `0/1`.

The pressure input is considered valid from `0.35 V` to `2.05 V`. The nominal
4–20 mA range on a 100-ohm shunt is `0.40...2.00 V`.
