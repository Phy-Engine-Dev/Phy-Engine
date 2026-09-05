# Analog components and import boundaries

These additions are native MNA devices. They do not call an LLM, a remote
simulator, or a formula evaluator in place of the circuit solver.

| C ABI code | Model | Properties, in order | Pins, in order |
|---|---|---|---|
| 24 | `clamped_op_amp` | gain, minimum output V, maximum output V | +, −, out, reference |
| 25 | `voltage_meter` | input resistance Ω | +, − |
| 26 | `analog_schmitt` | low threshold V, high threshold V, inverted 0/1, low output V, high output V, slew V/s | input, out, reference |
| 27 | `triangle_gen` with load | high V, low V, frequency Hz, phase rad, rising duty fraction, series Ω | +, − |
| 28 | `relay_current_spdt` | L H, R Ω, pull-in A, drop-out A, on Ω, off Ω, operate delay s, release delay s, initial engaged 0/1 | NC, COM, NO, coil+, coil− |
| 29 | `fuse_latched` | rated A, trip A, on Ω, off Ω, enabled 0/1, initially blown 0/1 | +, − |

Existing codes 17 (unlimited amplifier), 23 (four-property triangle), and 233
(digital Schmitt) retain their property stream and defaults. Triangle code 23
still defaults to 50% duty and zero internal resistance. Negative phases now
wrap correctly. A time-domain source has zero *small-signal* AC excitation;
its series resistance remains in the AC system.

The new five-pin relay does not replace legacy code 18. Its coil is a real
series R-L branch, integrated with backward Euler in transient analysis.
Magnitude of solved coil current controls pull-in/drop-out hysteresis; delays
advance at actual solve times, never by Newton iteration count. Contacts have
explicit finite on/off resistances. DC ignores mechanical delays; AC holds the
preceding DC contact state. No bounce, arcing or thermal failure is claimed.
Branch currents are coil+→coil−, COM→NC and COM→NO. Read-only attributes 9/10
are engagement and real coil current; use branch phasors for AC current.

The fuse is an explicit instantaneous-overcurrent engineering model, not a
thermal I²t model. The rated-current field is retained separately from the trip
threshold. A trip latches the real finite-resistance branch open; removing the
fault does not heal it. The `Reset` attribute explicitly represents replacement
or reset, and a remaining fault retrips on the next solve. Small-signal AC does
not trip from phasor magnitude. Read-only attributes 7/8/9 return blown state,
real branch current and the converged current that caused the last trip.

## Convergence and irreversible state

Device residual checks accept a const model and must not latch irreversible
state. Only after every constitutive check passes does the solver call the
optional `commit_converged_state_define(tag, model&)` hook. It returns false
when an event changed state and another MNA solve is required. All such hooks
observe the same solved electrical state. The mixed-signal driver additionally
waits for the digital-drive fixed point before committing events. This prevents
a fuse from blowing from a PN/BJT limiter trial or an intermediate logic drive.

The new `fuse_convergence_order` regression checks a real BJT feedback circuit
with the fuse first and last in the model list, on both sides of its trip
threshold. It also checks an analog-input NOT gate whose initial trial drive
would overload a fuse but whose settled drive is zero. These cases previously
exposed model-order-dependent and premature irreversible trips. C++ embedders
must rebuild against the updated model interface; the existing C creation
streams and function signatures remain compatible.

## Digital levels and mixed-signal analysis

The existing named-scalar ABI accepts `Ll` and `Hl` for INPUT, OUTPUT, NOT, TFF,
T_BAR_FF and JKFF. INPUT keeps its original digital attribute 0; levels occupy
1/2. OUTPUT likewise keeps its legacy read-only digital observation at index 0
and adds analog input thresholds at indices 1/2. It never injects an analog
drive; pure-digital observations and its existing transition-settling timing
are unchanged. The other four models use indices 0/1. Values must be finite,
with zero and negative levels allowed. Legacy creation streams/default 0/5 V
are unchanged. Other basic gates already expose the same names. Gate output
drivers remain ideal voltage sources: saved maximum-current ratings are not silently implemented
as physical limits, and must be disclosed or rejected by an adapter.

`circuit_run_mixed_dc(circuit, type)` supports type 0 (OP) and 1 (DC).
The existing bounded transient/trace ABI also settles digital and analog
models at each actual step. Logic events establish ideal drives, duplicate
visits to the same output are merged, MNA branch indices are refreshed, and
the analog solution is checked against the resulting drive state. L/C history
updates only once per real time step. Conflicting drivers or a fixed point
that fails to settle within 64 iterations return failure, not a successful
partial answer. This is a numerical convergence bound, not an agent budget.
Mixed AC is not supported. On failure discard the handle or rebuild from the
input specification; in-memory trial state is not transactionally rolled back.

## PN-junction convergence

The PN model now tests actual solved-voltage conduction against its stamped
linearization, preventing a tiny-Is junction from falsely converging while
its internal voltage limiter is still advancing. Its overflow-safe exponential
continuation has a consistent derivative. The four-junction bridge forwards
the checks of all internal junctions. These are numerical fixes, not changes
to diode parameters to fit a target voltage or archived result.

The amplifier is a memoryless finite-gain, hard-clamped behavioral model with
zero input current and ideal output impedance. Its converged solution must
satisfy the actual clipped equation, not merely small successive Newton steps.
Small-signal AC uses the converged DC slope, zero in saturation. There is no
invented bandwidth, slew rate, output-current limit, supply current or thermal
model. These omissions must be disclosed by a file-format adapter.

The meter inserts a finite shunt conductance, **not a zero-volt source**. Scalar
attributes 16/17 return signed currents into +/−; 18/19 return their AC imaginary
parts. Its dial ranges and archived RMS/average readings are not emulated by
this passive input probe. Resistance must be specified with a provenance.

Schmitt state is based on the previous solved step, never on the ordering of
Newton trial iterates. Its explicit native slew is in V/s; zero means no rate
limit. It is not assumed to share the units of an undocumented application
property. Equal high/low output levels are exactly constant regardless of
hysteresis and slew. Its reference is an explicit third native pin.

## BJT numerical initialization

The existing quasi-static Ebers–Moll equations and parameters are unchanged.
When both junction voltages are effectively zero, the first Newton
linearization uses a thermal-voltage/saturation-current-derived critical BE
voltage. The same numerical initialization is available at the start of a
transient step after returning to zero bias. It does **not** assign stored node
voltages or pretend a physical initial condition; subsequent iterations and
the convergence check require the unlimited device equation at solved pins.

This follows the role of the initial junction guess in
[ngspice's BJT load implementation](https://github.com/ngspice/ngspice/blob/master/src/spicelib/devices/bjt/bjtload.c),
but is an independent implementation. It prevents an all-zero exponential
Jacobian from sending a feedback amplifier's first trial millions of volts
past its rails. Native tests include both BJT polarities, several input
magnitudes, and repeated zero/nonzero transitions, with independent terminal
KCL and exponential-law checks.

## PhysicsLab archives

The official SDK describes serialized fields and pin roles, not every detail
of the closed-source application's solver. Historical v2.0.6 archives and the
current SDK also differ (notably Schmitt field names). The C ABI above is an
explicit native model contract; a renderer knowing a model's pin count does
not imply numerical simulation compatibility.

An adapter must preserve the original model ID, properties, statistics,
positions and wires separately from simulation assumptions. The original
`Operational Amplifier` pins are negative, positive, output; native pin order
must therefore be remapped and the hidden application reference represented
explicitly. Unconnected inputs must not be confused with actually wired
floating nodes. Original meter statistics are evidence, not fresh results;
consistent archived V/I can justify an explicitly inferred loading assumption
but not a claim of independent calibration or a universal dial-impedance table.

## Native regression

`test/0033.analog_components/analog_components.cpp` is picked up by the existing
test CMake glob. It checks amplifier rails/finite gain, incompatible ideal
drivers returning failure, real meter loading/current, triangle duty/phase/
loaded voltage, Schmitt hysteresis/slew/polarity, and generic nonlinear feedback.
It contains no saved community solution or application-specific design recipe.

Additional repository tests cover five-pin relay DC/AC/transient contacts,
digital level attributes, INPUT/NOT-driven relay step responses, invalid
parameters, conflicting drivers and non-settling mixed feedback. The transient
oracle is the discrete RL recurrence, checked at every completed native step.
