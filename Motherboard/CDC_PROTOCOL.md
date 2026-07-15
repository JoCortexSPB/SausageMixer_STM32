# Motherboard USB CDC protocol

USB CDC uses ASCII lines terminated by `\n`. Commands are accepted both from
MixerMonitor and from the physical keyboard through the same state-machine entry
point, so the two control surfaces have identical semantics.

## Machine commands

```text
START
STOP
RESET
LEFT
AIR,1
AIR,0
PAUSE,1
PAUSE,0
LENGTH,10.000
COUNTERS,RESET
```

- `START` starts or resumes the current state-machine stage.
- `STOP` is a safe stop: air, carriage and both frequency-converter directions
  are disabled. The program and current scenario stage remain available.
- `RESET` first performs the same safe stop, then returns the scenario to the
  homing stage. Production counters are retained.
- `LEFT` toggles manual reverse only while the automatic scenario is not running.
- `PAUSE` enables/disables the intermediate stop by produced length.
- `LENGTH` configures that stop in the range `0.1...1000 m`.
- `COUNTERS,RESET` is accepted only while the state machine is not running.

## Service / burst-test commands

```text
AIROUT,1.250
DROP,0.080
STREAM,START,50
STREAM,STOP
PING
```

- `AIROUT` selects direct PC control and accepts `0.000...10.000 V` at the ITV
  input. The AD5328 produces `0...5 V`; the external op-amp has gain x2.
- `DROP` sets the controller-side burst threshold (`0.020...0.500 V`).
- `STREAM,START` enables telemetry; period is clamped to `20...1000 ms`.
- Direct `AIROUT` control has a 750 ms watchdog.

## Production pressure control

`REG_ITV` is the working-pressure setting. Its `0...5 V` range maps to
`0...2 bar`; for the 0...10 bar, 4...20 mA sensor on a 100-ohm shunt this is
`ITV_PRESSURE = 0.400...0.720 V`. `PRESSURE_READY` becomes true after the
measured value stays within 0.008 V below the target for 300 ms and is released
with 0.016 V hysteresis. CYCLE actuators do not start until this condition is met.

The pressure sensor validity range is `0.35...2.05 V`; nominal range is
`0.40...2.00 V`. Critical-pressure warning is a MixerMonitor setting and does
not automatically stop the controller.

## Telemetry

```text
TEL,pressure_v,target_v,reg_itv_v,reg_carousel_v,reg_carriage_v,airout_v,carriage_speed_v,air_on,valid,ready,state,stage,meter_m,cycle_count,product_count,pause_enabled,pause_length_m,sensors,carriage_on,vf_forward,vf_reverse,cycle_active
```

All voltages and lengths have three decimal places. Boolean values are `0/1`.

- state: `0=IDLE`, `1=RUNNING`, `2=STOPPED`, `10=ERROR`;
- stage: `0=NOINIT`, `1=CYCLE`, `2=REWARD`, `3=SEEKHOME`;
- sensors bit mask: bit 0 HOME, bit 1 CNT, bit 2 NEAR, bit 3 FAR;
- `cycle_active` starts with the first START in CYCLE and clears after SEEKHOME.
