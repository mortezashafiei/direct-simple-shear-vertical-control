# How to control the vertical axis of a DSS device

A practical guide, from what I learned automating the vertical system of a direct simple shear (DSS) device with a small microcontroller board (MCU). It applies to any DSS (or oedometer, or direct shear) device with a motor-driven loading piston, a load cell and a displacement sensor.

**Contents**

1. [What you are controlling](#1-what-you-are-controlling)
2. [Hardware: what limits you](#2-hardware-what-limits-you)
3. [Split the work between the microcontroller and the PC](#3-split-the-work-between-the-microcontroller-and-the-pc)
4. [Constant height (constant volume)](#4-constant-height-constant-volume)
5. [Holding a vertical load](#5-holding-a-vertical-load)
6. [Constant normal stiffness (CNS)](#6-constant-normal-stiffness-cns)
7. [Putting a load step on](#7-putting-a-load-step-on)
8. [Safety layers](#8-safety-layers)
9. [Commissioning checklist](#9-commissioning-checklist)
10. [Pitfalls](#10-pitfalls)

---

## 1. What you are controlling

The vertical axis is a chain:

```
DC motor → gearbox → worm drive → loading piston → frame and load cell → top cap → specimen
```

During a test it must keep one of these boundary conditions:

| Phase | Boundary condition | Kept | Free |
|---|---|---|---|
| Consolidation | Constant vertical stress (each load step) | σv | height |
| Shear | Constant height (constant volume; ASTM D6528, D8296) | height | σv; its change is taken as the pore pressure Δu |
| Shear | Constant normal stress | σv | height |
| Shear | Constant normal stiffness (CNS) | K = Δσv/Δh | both |

**Why it is hard.** The loading system is stiff. A loaded clay specimen in series with the frame typically has a stiffness of several hundred kPa per mm (illustrative order). One micrometre of piston travel is then about 1 kPa. So the controller has to work at micrometre level with a motor and gears that do not like to move by micrometres. Gear play and static friction mean that when the motor breaks free, it always moves a little more than asked. That minimum jump (the gears' **release**) grows with the load.

## 2. Hardware: what limits you

| Item | What to look for |
|---|---|
| Height sensor | An LVDT with a signal conditioner. Its noise, not its resolution on paper, sets the smallest band you can hold. |
| Analog-to-digital converter | A 12-bit microcontroller ADC was not enough. A 16-bit board helped. A 24-bit board, read every 20 ms, allowed a 0.3 mV band. |
| Load cell + data acquisition | The reading rate matters more than resolution. About 1 reading/s is too slow to stop a move in time; about 9 scans/s lets the program cut a move as the load changes. |
| Motor feedback | A tachometer or encoder on the motor. With it, a step is "n turns" instead of "t seconds at p % power", so its size no longer depends on friction and load. |
| Motor driver | PWM with a known direction convention (on many rigs a minus command unloads). Keep the motor supply switchable (for example, unplug it when flashing firmware). |
| E-STOP | Within reach of the operator, cutting motor power at the rig. |

**Rule of thumb for the band.** Measure the RMS noise of the height reading with the motor still. The band must be at least about 3 × that noise, or the loop chases noise. Then check what one push changes, both in height and in load. If the smallest possible push already exceeds the band, the band is too tight for the gears.

## 3. Split the work between the microcontroller and the PC

| Microcontroller (fast, simple, safe) | PC program (slow, smart, recorded) |
|---|---|
| Reads the height every 20 ms | Reads the load cells and the other sensors |
| Pushes the motor back into the band (AUTO) | Decides targets, setpoints and step sizes |
| Runs the motor at a set power (MANUAL) | Learns the specimen stiffness, the load per motor turn and the power needed per direction |
| Makes counted steps of n turns | Judges every move on the load |
| Own guards: sensor at its rail, runaway motor, lost link | Records, plots and reports; asks before anything irreversible |

Keep the microcontroller's loop small and testable. Every decision that needs the load, the test plan or history belongs on the PC.

## 4. Constant height (constant volume)

The microcontroller holds the height reading `h` at the setpoint `sp` captured at the start of the shear:

```python
every 20 ms:
    h = read_height()                       # median of a few conversions
    if now < rest_until:   continue         # let the specimen and the frame settle
    err = h - sp
    if abs(err) <= band:   continue         # e.g. band = 0.3 mV
    turns = min(turns_to_close(err), TKMAX) # never more than one capped push
    push(direction_of(err), turns)
    rest_until = now + 4.5 s
```

- **Cap every push.** Choose TKMAX = 0.9 × (allowed load change per push, e.g. 1 kPa) / k_pt. Here k_pt (kPa per motor turn) is measured by the PC and sent to the microcontroller. Allow 2 kPa while the reading is behind by more than 1.5 bands.
- **Judge every correction on the load.** The PC compares σv before and after each push. Past about 2 kPa it warns; a stop is an option the operator can tick.
- **Rest after a push.** The frame, the load cell and the specimen need a few seconds before the next reading means anything.
- **Expect the load to change.** In a contractive soil σv falls at constant height, and the controller mostly unloads. The change Δu = σ′vc − σ′v is the result of the test, not an error.

## 5. Holding a vertical load

The controlling sensor is the load cell. Three ways to move the motor, from simplest to best:

**a) Timed moves on the load.** The microcontroller runs in MANUAL and the PC sets the power and stops each move. Correct when |T − σv| > ¾ band. Close a large gap along a ramp, and never move back within 10 s. With a fast load reading, cut each move as soon as the load has changed by the allowed amount. This works, but each move's size depends on friction, so it can be coarse at high load.

**b) Fine setpoint steps.** The microcontroller holds the height (AUTO), and the PC moves the setpoint:

```python
dsigma = 0.5 * (T - sigma_v)                # never more than one band
dsp    = s * dsigma / (1.5 * k)             # k = c * sigma_v, s = the sign for "more load"
```

Here k (the specimen stiffness) and s are learned while the step is loaded. Each move is small and gentle, because the height loop does the pushing.

**c) Governed steps (best, needs motor feedback).** The PC asks for n motor turns; the microcontroller ramps the motor gently, catches it at its first turning, and stops after n counted turns:

```python
every reading (about 10 per second):
    sigma = median(last 5 readings)                 # 0.5 s
    e  = T - sigma
    e1 = e - (d sigma/dt) * 1 s                     # the error expected one second ahead
    if abs(e1) < q / 2:  continue                   # q = about 0.5 kPa per step
    if direction(e1) != last_direction and not (past_by_one_band_for_30_s or past_by_two_bands):
        continue                                    # don't turn around on noise
    n = clip((abs(e1) + q/2) / k_pt, 0.12, 0.60)    # turns
    n = min(n, abs(e) / k_pt)                       # never sized to go past the target
    step(direction(e1), n)
    learn k_pt from the load change this step caused
```

Learn k_pt continuously. It changes with the load and with the direction.

## 6. Constant normal stiffness (CNS)

The target follows the height change from the start of the shear (Δh in mm, + = dilation):

```python
T = max(2 kPa, sigma_vc + K * median(dh over 2 s))
```

Then hold T exactly as in section 5. K = ∞ is constant height and K = 0 is constant normal stress, which is a useful check of the code.

## 7. Putting a load step on

- **Ramp the target** instead of jumping to it: T(t) = σ0 + (Tk − σ0)·min(1, t/60 s).
- **Start gently.** The first moves at low power (for example 25 %), raised by +5 % only when a move brings no answer, and never above a set maximum power.
- **Never go past the target.** Stop by T + max(1 kPa, 0.5 % of T) at the latest; the last few kPa are better done by small steps.
- **Learn each direction separately.** Loading and unloading need different power, and both change with the load.
- **End a step automatically when the specimen is done.** For example, when εv changes less than 0.01 % in 30 min.

## 8. Safety layers

1. **E-STOP** at the rig. The operator stays within reach whenever the motor can move.
2. **Sensor limits.** Each sensor has a limit below its rated capacity, which stops the test when passed.
3. **Load guard.** It watches the vertical load whenever no control is changing it on purpose, and stops the motor when the load moves past a limit. During the shear it only warns.
4. **Microcontroller guards.** A height reading at its rail, or a motor that runs without the reading changing, stops the motor by itself. A watchdog on the link from the PC is worth adding too.
5. **Ask before irreversible actions**, and never let the motor start at a power left over from before.

## 9. Commissioning checklist

1. Measure the noise of the height reading with the motor still, and choose the band (section 2).
2. Measure k_pt (kPa per turn) and the release in both directions at a few loads.
3. Try the control rules on a simulation first. [`examples/vertical_control_sim.py`](../examples/vertical_control_sim.py) is a starting point: change the stiffness, the release and the noise to match your rig.
4. Run a dummy specimen (or a stiff spring) at low load, then at the working load.
5. Check each move in the log: the load change per move, how often the controller moves, and that it never goes past the target.
6. Only then run a real specimen.

## 10. Pitfalls

- **A band tighter than the noise.** The motor hunts back and forth forever.
- **Steps sized by time.** The same "0.5 s at 40 %" moves very differently at 100 and 800 kPa.
- **A load reading too slow to stop a move.** The load overshoots before the controller knows.
- **Turning around on noise.** Use medians, and require the load to be clearly past the target before reversing.
- **A big jump at the start of automatic control.** Capture the setpoint from the current reading and ramp from there.
- **Treating the gears as ideal.** The release when the motor breaks free is the true lower limit of every correction.

---

*Morteza Shafiei (Mortezashafiei). References: see the [README](../README.md#references).*
