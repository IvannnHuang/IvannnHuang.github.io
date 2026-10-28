/* Portfolio content. Every number here comes from the project's report or README. */

const CATEGORIES = [
  {
    id: "robotics", short: "Robotics", name: "Robotics & Autonomy", color: "var(--c-robotics)",
    blurb: "Estimation, planning and control on real robots — from particle filters to closed-loop steppers."
  },
  {
    id: "perf", short: "Performance", name: "Performance Engineering", color: "var(--c-perf)",
    blurb: "Profiling-driven optimization of dense, sparse and ML workloads on multicore CPUs."
  },
  {
    id: "embedded", short: "Embedded", name: "Embedded Systems", color: "var(--c-embedded)",
    blurb: "Real-time firmware on the RP2350: interrupts, DMA, dual-core scheduling and fixed-point math."
  },
  {
    id: "fpga", short: "FPGA & HLS", name: "FPGA & Hardware Acceleration", color: "var(--c-fpga)",
    blurb: "Custom datapaths in programmable logic and high-level synthesis for compute-heavy kernels."
  },
  {
    id: "silicon", short: "Silicon", name: "Silicon & Architecture", color: "var(--c-silicon)",
    blurb: "Transistor-level layout, RTL processors and ASIC accelerators taken through physical design."
  }
];

const PROJECTS = [
  /* ---------------- SILICON & ARCHITECTURE ---------------- */
  {
    id: "tcam", repo: "https://github.com/IvannnHuang/VLSI-TCAM", cat: "silicon", demo: "tcam",
    title: "32×32 Ternary CAM with Priority Encoder",
    course: "VLSI Design · 180 nm CMOS",
    purpose: "Single-cycle associative lookup — the memory behind router tables and TLBs — that searches 32 ternary words at once and returns the highest-priority match.",
    how: "Full-custom 2-SRAM + XOR pull-down cells on precharged matchlines, feeding a 32→5 priority encoder built from chained 8→3 encoders.",
    metrics: [["69 ps", "search → matchline"], ["241 ps", "search → encoder out"], ["97×54 µm", "array layout"]],
    tags: ["Cadence Virtuoso", "Verilog-A", "Layout", "DRC/LVS"],
    steps: [
      "<b>Cell:</b> two SRAM bits encode 0 / 1 / X; an XOR pull-down network discharges the matchline on any mismatch.",
      "<b>Row:</b> 32 cells share a precharged domino matchline, so all rows compare in parallel in one evaluate phase.",
      "<b>Encoder:</b> chained 8→3 encoders pick the highest-index matching row out of 32.",
      "<b>Layout:</b> 2.98×1.625 µm cell (M2/M4 vertical, M3 horizontal) tiled into a 97×54 µm array; DRC and LVS clean.",
      "<b>Sign-off:</b> post-extraction simulation — 1.39 pJ per search, 2.16 pJ average write."
    ],
    figs: [["tcam.webp", "32×32 TCAM array layout"], ["tcam-wave.webp", "Write / search waveform with matchline and encoder outputs"]],
    hint: "precharge → broadcast key → mismatches discharge"
  },
  {
    id: "riscv", repo: "https://github.com/IvannnHuang/PipelineProc", cat: "silicon", demo: "pipeline",
    title: "Pipelined Multicore RISC-V Processor",
    course: "ECE 4750 · Computer Architecture", team: "with Antti Meriluoto",
    purpose: "A complete RV32 system in RTL — from a single pipelined core up to a four-core machine sharing a banked cache.",
    how: "5-stage in-order pipeline with full bypassing, load-use stalls and branch squashing; direct-mapped I/D caches; four cores linked to a 4-bank data cache over a ring network.",
    metrics: [["5-stage", "bypassed pipeline"], ["4 cores", "ring-network memory"], ["−54%", "cycles on parallel sort"]],
    tags: ["SystemVerilog", "PyMTL3", "RISC-V"],
    steps: [
      "<b>Core:</b> IF/ID/EX/MEM/WB with forwarding from MEM and WB, stall logic for load-use hazards and squash on taken branches, plus a single-cycle multiplier.",
      "<b>Memory:</b> direct-mapped instruction and data caches with ready/valid interfaces and a latency-modelled memory.",
      "<b>Multicore:</b> four cores, private I-caches and a shared 4-bank D-cache connected by a ring network with shortest-path routing and fixed-priority arbitration.",
      "<b>Verification:</b> directed and random unit tests per block, then full-system tests on vvadd, cmult, mfilt, bsearch and a multithreaded sort."
    ],
    figs: [["riscv.webp", "Single-threaded cycle counts across evaluation programs"]],
    hint: "watch forwarding, stalls & squashes"
  },
  {
    id: "tinyflow", cat: "silicon", demo: "tinyflow",
    title: "TinyFlow: Our Own RTL-to-GDS Toolflow",
    course: "ECE 6745 · Complex Digital ASIC Design", team: "Group 15",
    purpose: "Take logic from Verilog to manufacturable layout using an ASIC flow we wrote ourselves, then tape out custom blocks with it.",
    how: "Custom standard cells, a Python tech-mapper and static timing analyzer, simulated-annealing placement minimizing wirelength, and a maze router with rip-up-and-retry.",
    metrics: [["8-step", "RTL → GDS flow"], ["SA", "placement on HPWL"], ["DRC/LVS", "clean sign-off"]],
    tags: ["Python", "Verilog", "KLayout", "Verilator"],
    steps: [
      "<b>Standard cells:</b> designed and verified INV, NAND2, NOR2, AOI21 and more (layout, SPICE, Verilog views).",
      "<b>Synthesis:</b> parse gate-level Verilog, tech-map onto the cell library and run static timing analysis.",
      "<b>Place & route:</b> random initial placement refined by simulated annealing on half-perimeter wirelength; multi-net maze routing on two metal layers with rip-up and retry.",
      "<b>Tapeout:</b> 8-in/8-out blocks — a 4-bit Sklansky parallel-prefix adder, a 3-bit ALU and multiply/divide variants — through RTL sim, gate-level sim, DRC and LVS."
    ],
    figs: [],
    note: "Course code is private under the course's academic-integrity policy, so only results are shown.",
    hint: "annealing placement → maze routing"
  },
  {
    id: "sw", cat: "silicon", demo: "smithwaterman",
    title: "Smith–Waterman DNA Alignment Accelerator",
    course: "ECE 6745 · Complex Digital ASIC Design", team: "with Yaqi Gao",
    purpose: "Speed up DNA sequence alignment — a core bioinformatics kernel — with a custom accelerator attached to a RISC-V processor.",
    how: "A processing-element array sweeps the dynamic-programming matrix along anti-diagonals inside a fixed band, computing match / insert / delete maxima in parallel.",
    metrics: [["3.52×", "speedup vs. processor"], ["−69.7%", "energy"], ["10 ns", "timing met post-PnR"]],
    tags: ["Verilog", "PyMTL", "ASIC flow", "RISC-V"],
    steps: [
      "<b>Algorithm:</b> banded local alignment — only cells near the diagonal are computed, which bounds the hardware to a fixed number of lanes.",
      "<b>Datapath:</b> per-lane score registers, max-of-three scoring logic and a best-score tracker; CSR-configured from the processor.",
      "<b>Physical design:</b> 117,812 µm² standalone (~6k cells), timing met at 10 ns; the critical path runs through the scoring comparison logic.",
      "<b>System result:</b> 215,436 → 61,302 cycles end-to-end for +58% area and +6% power, cutting energy from 44.4 µJ to 13.4 µJ."
    ],
    figs: [["sw-layout.webp", "Post-PnR layout — red: processor, yellow: accelerator"]],
    note: "Course code is private under the course's academic-integrity policy, so only results are shown.",
    hint: "anti-diagonal wavefront → traceback"
  },

  /* ---------------- FPGA & ACCELERATION ---------------- */
  {
    id: "lorenz", repo: "https://github.com/IvannnHuang/MEng_ZYBO", cat: "fpga", demo: "lorenz",
    title: "Lorenz Attractor Solver on Zybo Z7",
    course: "M.Eng Project · Xilinx Zynq-7010",
    purpose: "Port Cornell's DE1-SoC FPGA labs to the Xilinx Zybo Z7. The first lab is a hardware ODE solver that draws chaos live over HDMI.",
    how: "A 27-bit fixed-point Euler integrator packaged as custom AXI IP; the ARM core reads x/y/z over AXI-Lite and plots three projections over HDMI.",
    metrics: [["27-bit", "fixed-point solver"], ["AXI IP", "PL ↔ ARM PS"], ["HDMI", "3-view plotter"]],
    tags: ["Verilog", "Vivado", "Vitis", "Zynq"],
    steps: [
      "<b>Solver IP:</b> x, y, z, σ, ρ, β and dt as signed 27-bit fixed-point registers; one Euler step per tick in the programmable logic.",
      "<b>Integration:</b> wrapped as AXI4-Lite IP inside the Digilent HDMI reference design; the PS sets initial conditions and parameters and reads state back.",
      "<b>Display:</b> the ARM plots xy / yz / xz projections as connected lines, with hardware speed control from the terminal.",
      "<b>Lab guide:</b> wrote a step-by-step documentation site (clean build and full debugging history) so future students can recreate the lab."
    ],
    figs: [["zybo-lorenz.webp", "Running on the Zybo Z7-10 — three projections over HDMI", true]],
    hint: "two trajectories, Δx₀ = 0.001 — the butterfly effect"
  },
  {
    id: "mandel", repo: "https://github.com/IvannnHuang/MEng_ZYBO", cat: "fpga", demo: "mandelbrot", status: "in progress",
    title: "1080p Mandelbrot Accelerator",
    course: "M.Eng Project · Xilinx Zynq-7010",
    purpose: "Render the Mandelbrot set at 1920×1080 in FPGA fabric with live pan and zoom — the second ported lab.",
    how: "Q4.23 fixed-point iterator running one iteration per clock with only three multipliers, verified bit-exact against a Python model, driven by ARM control code.",
    metrics: [["687/687", "bit-exact test vectors"], ["3 mults", "per iteration"], ["1 iter/clk", "pipeline"]],
    tags: ["Verilog", "Python", "Vitis", "USB host"],
    steps: [
      "<b>Iterator:</b> zr' = zr² − zi² + cr, zi' = 2·zr·zi + ci; squares are saved between cycles so only three multipliers are needed, and saturating products keep the escape test correct.",
      "<b>Verification:</b> a bit-exact Python model of the RTL generated 687 vectors (in-set points, corners, zooms, random points); mutation tests confirm the bench catches bugs.",
      "<b>Planning:</b> ~3.2 G iterations per 1080p frame → frame time scales as 3.2 s / N pipelines, sized against the Z7's DSP48 budget.",
      "<b>Interaction:</b> PL driver plus PS control code, with USB-mouse pan and click-to-zoom on the board's USB host port."
    ],
    figs: [],
    hint: "click to zoom · tiles = parallel solver lanes"
  },
  {
    id: "hls", cat: "fpga", demo: "vla",
    title: "laySmolVLA: Robot-Policy Transformer on FPGA",
    course: "High-Level Synthesis · Alveo U280", team: "4-person team",
    purpose: "Accelerate the Action Expert of SmolVLA — the transformer that turns vision-language features into 50-step robot action chunks.",
    how: "Wrote self/cross-attention (RoPE, GQA), RMSNorm and gated MLP as Allo kernels, then used operational-intensity analysis to guide kernel fusion, pipelining and int8 quantization.",
    metrics: [["2×", "fusion + pipelining"], ["5×", "with int8 (csynth)"], ["U280", "on-board bitstream"]],
    tags: ["Allo", "Vitis HLS", "PyTorch", "NumPy"],
    steps: [
      "<b>Model:</b> flow-matching transformer that refines noisy action tokens into a 50-step trajectory; self-attention and cross-attention layers alternate.",
      "<b>Correctness:</b> every Allo kernel was checked against deterministic NumPy references — first per kernel, then composed layers.",
      "<b>Bottlenecks:</b> a Roofline / operational-intensity study showed the large GEMMs dominate.",
      "<b>Optimizations:</b> fused the gate + up MLP projections (≈2× on the MLP alone), inlined to cut buffers, restructured reductions for better initiation intervals, then explored int8 linear layers."
    ],
    figs: [["hls-roofline.webp", "U280 Roofline with measured kernel intensities"], ["hls-speedup.webp", "Csynth speedup: baseline → optimized → int8"]],
    hint: "flow-matching denoises an action chunk"
  },

  /* ---------------- EMBEDDED ---------------- */
  {
    id: "galton", repo: "https://github.com/IvannnHuang/uController", cat: "embedded", demo: "galton",
    title: "Digital Galton Board",
    course: "ECE 5730 · RP2350 (Pico 2)", team: "with Amber Li & Zarif Karim",
    purpose: "The central limit theorem, live: thousands of balls bouncing through pegs on a VGA screen, each collision making a sound.",
    how: "Fixed-point ball-peg physics split across both Cortex-M33 cores, DMA-driven audio and eight measured optimizations to hit the 60 fps deadline.",
    metrics: [["16,900", "balls at 60 fps"], ["2 cores", "+ DMA audio"], ["8", "measured optimizations"]],
    tags: ["C", "RP2350", "VGA", "DMA"],
    steps: [
      "<b>Physics:</b> balls reflect off round pegs along the collision normal with tunable bounciness and gravity, all in fixed point.",
      "<b>Parallelism:</b> ball updates are split across both cores, synchronized once per frame.",
      "<b>Memory & code:</b> shrank ball structs from 24 to 12 bytes, used nearest-peg lookup instead of checking every peg, used precomputed sprites, and moved hot loops into on-chip SRAM.",
      "<b>Lab extras:</b> rotary-encoder UI, histogram reset when parameters change, and an LED that lights on a missed frame deadline."
    ],
    figs: [["galton.webp", "The final board on VGA — 16,900 balls", true], ["galton-opt.webp", "Max balls at 60 fps after each optimization"]],
    hint: "a binomial histogram becomes a Gaussian"
  },
  {
    id: "bird", repo: "https://github.com/IvannnHuang/uController", cat: "embedded", demo: "dds",
    title: "Birdsong Synthesizer",
    course: "ECE 5730 · RP2350 (Pico 2)", team: "with Amber Li & Zarif Karim",
    purpose: "Synthesize bird calls in real time, then test them against the Merlin Bird ID app.",
    how: "A 50 kHz timer ISR runs direct digital synthesis (phase accumulator + sine table) out to an SPI DAC; protothreads handle the keypad, slider and modes.",
    metrics: [["50 kHz", "DDS sample rate"], ["SPI DAC", "audio output"], ["3 modes", "play · record · compose"]],
    tags: ["C", "Protothreads", "SPI", "ADC"],
    steps: [
      "<b>DDS:</b> each sample adds a frequency-proportional increment to a 32-bit phase accumulator; its top bits index a sine table.",
      "<b>Audio path:</b> samples go over SPI to a DAC and out a 3.5 mm jack, with amplitude ramps to avoid clicks.",
      "<b>Inputs:</b> slider potentiometer via ADC sets pitch; a keypad debounced by an FSM records sound bites.",
      "<b>Application:</b> play, record and compose modes run as cooperative protothreads alongside the ISR."
    ],
    figs: [["bird.webp", "Merlin Bird ID and a spectrogram of the synthesized output"]],
    hint: "click to hear a synthesized chirp ♪"
  },
  {
    id: "heli", repo: "https://github.com/IvannnHuang/uController", cat: "embedded", demo: "pid", status: "in progress",
    title: "PID-Controlled 1-D Helicopter",
    course: "ECE 5730 · RP2350 (Pico 2)", team: "with Amber Li & Zarif Karim",
    purpose: "Hold a propeller-driven arm at any angle, including the unstable region above horizontal, using feedback control.",
    how: "1 kHz ISR: read the MPU6050 IMU → low-pass the accelerometer → complementary filter → PID in fixed point → motor PWM.",
    metrics: [["1 kHz", "control loop"], ["IMU fusion", "complementary filter"], ["PID", "15.16 fixed point"]],
    tags: ["C", "I²C", "PWM", "Control"],
    steps: [
      "<b>Sensing:</b> accelerometer angle is accurate but noisy, gyro is smooth but drifts; a complementary filter blends them.",
      "<b>Control:</b> error = desired − measured angle feeds P, I and D terms, computed in 15.16 fixed point inside the ISR.",
      "<b>Why closed-loop:</b> thrust grows with motor speed squared and the plant turns nonlinear past 90°, so open-loop lookup tables fail."
    ],
    figs: [],
    hint: "click anywhere to set a new target angle"
  },

  /* ---------------- ROBOTICS ---------------- */
  {
    id: "fastrobots", repo: "https://github.com/IvannnHuang/FastRobot", cat: "robotics", demo: "bayes",
    title: "Fast Robots: Autonomous RC Car",
    course: "ECE 4160 · Fast Robots · 12 labs",
    purpose: "Turn a toy RC car into a fast autonomous robot, building sensing, control, estimation and planning from scratch over a semester.",
    how: "Artemis board with BLE telemetry, IMU and dual time-of-flight sensors, PID control, a Kalman filter, a flip stunt, mapping and Bayes-filter localization.",
    metrics: [["12", "labs, one robot"], ["KF", "drives a flip stunt"], ["Bayes", "grid localization"]],
    tags: ["Arduino C++", "Python", "BLE", "Jupyter"],
    steps: [
      "<b>Sensing:</b> BLE command/telemetry link; IMU complementary filtering; two VL53L1X ToF sensors.",
      "<b>Control:</b> PID on wall distance and on yaw, with linear extrapolation between slow ToF readings.",
      "<b>Estimation:</b> a Kalman filter on the drivetrain model enables a high-speed approach and flip.",
      "<b>Autonomy:</b> 360° ToF scans build a map; a grid Bayes filter localizes the robot in sim and on the real car, then drives a waypoint path."
    ],
    figs: [["fr-stunt.webp", "Flip stunt with KF distance estimate"], ["fr-bel.webp", "Bayes-filter belief vs. ground truth"]],
    hint: "belief converging on the true pose"
  },
  {
    id: "roomba", repo: "https://github.com/IvannnHuang/Ece4180_Project", cat: "robotics", demo: "pfrrt",
    title: "Roomba Localization & Navigation",
    course: "ECE 4180 · Autonomous Mobile Robots",
    purpose: "Start a robot at an unknown pose, let it work out where it is, then visit every waypoint while avoiding stay-away zones.",
    how: "700-particle filter converged by a 360° depth sweep, then an RRT roadmap + Dijkstra for shortest paths, re-planning as bump-detected walls are added to the map.",
    metrics: [["700", "particles"], ["±0.1 m", "pose accuracy"], ["RRT + Dijkstra", "path planning"]],
    tags: ["MATLAB", "iRobot Create", "Particle filter"],
    steps: [
      "<b>Localize:</b> particles are seeded at every candidate waypoint with random headings; a 360° turn with depth + odometry collapses them onto the true pose.",
      "<b>Plan:</b> stay-away points are inflated into obstacles; RRT builds a roadmap and Dijkstra selects the shortest route.",
      "<b>Execute:</b> follow the path while the particle filter keeps updating; bump sensors mark optional walls and trigger re-planning.",
      "<b>Result:</b> robust localization and navigation in simulation and on the robot. The strict run-time limit meant the final run did not reach every waypoint."
    ],
    figs: [["pf.webp", "Initial particle filter localization"], ["rrt.webp", "RRT roadmap and selected path"]],
    hint: "particles collapse → RRT grows → robot drives"
  },
  {
    id: "ros", cat: "robotics", demo: "prm",
    title: "Autonomy Stack: Robot Arm & AGV",
    course: "ECE 4770 · Robotics · ROS",
    purpose: "Integrate state estimation, planning and control into two complete robots: a pick-and-place arm and a hospital delivery vehicle.",
    how: "Arm: object pose → inverse kinematics → start and goal inserted into a PRM roadmap → collision-free joint-space path. AGV: particle filter + sampling planner + MPC.",
    metrics: [["IK → PRM", "joint-space planning"], ["MPC", "vehicle control"], ["ROS", "full pipeline"]],
    tags: ["ROS", "Python", "Kinematics", "MPC"],
    steps: [
      "<b>Estimate:</b> particle-filter detections give can and wallet poses; the code waits for them to stabilize before grasping.",
      "<b>IK:</b> every Cartesian goal becomes a joint configuration and is checked for feasibility.",
      "<b>Plan:</b> start and goal configurations are inserted into the roadmap and queried for a collision-free path.",
      "<b>Execute & AGV:</b> the controller tracks the path; the AGV fuses a particle filter, planner and MPC to fetch medicine and return in time."
    ],
    figs: [["arm-sim.webp", "Arm picking a can in the airport scenario"]],
    hint: "left: workspace · right: C-space roadmap"
  },
  {
    id: "arm", repo: "https://github.com/IvannnHuang/Robot-Arm", cat: "robotics", demo: "stepper",
    title: "Closed-Loop Stepper Robot Arm",
    course: "INFO 4320 · Hardware build",
    purpose: "Design and build a desktop robot arm from scratch: CAD, 3D-printed joints, electronics and firmware.",
    how: "3D-printed links from custom CAD, NEMA23 steppers with AS5600 magnetic encoders, and ESP32 firmware with per-joint PID correction of step error.",
    metrics: [["3 axes", "+ servo gripper"], ["AS5600", "encoder feedback"], ["ESP32", "PID firmware"]],
    tags: ["CAD", "3D printing", "ESP32", "Arduino"],
    steps: [
      "<b>Mechanical:</b> designed the base, joints and encoder mounts (STL files in the repo) and printed them.",
      "<b>Feedback:</b> open-loop steppers can silently skip steps; an AS5600 on each joint measures the real angle.",
      "<b>Firmware:</b> step rate per motor is set from the angle error with an integral term, so joints converge even under load.",
      "<b>Teleop:</b> joystick control and a servo gripper for pick-and-place."
    ],
    figs: [],
    links: [["PID control video", "https://youtube.com/shorts/Roi8RTV1Stw"], ["Joystick control video", "https://youtube.com/shorts/DsuliSb9qkk"]],
    hint: "encoder catches missed steps → PID corrects"
  },

  /* ---------------- PERFORMANCE ENGINEERING ---------------- */
  {
    id: "gemm", cat: "perf", demo: "gemm",
    title: "Cache-Optimized Matrix Multiply",
    course: "ECE 6750 · Parallel Computing", team: "with Kian Mahmoodi",
    purpose: "Make a 2000×2000 double-precision GEMM fast on one Xeon core by understanding the memory hierarchy.",
    how: "Profiled with Intel VTune, then applied loop reordering, cache tiling (block-size sweep → 48), software prefetching, a 4×4 register-blocked micro-kernel, unrolling and AVX FMA.",
    metrics: [["18.2 s → 0.7 s", "≈26× faster"], ["BS = 48", "from tile sweep"], ["−83%", "stores w/ reg. blocking"]],
    tags: ["C", "AVX2/FMA", "Intel VTune", "SLURM"],
    steps: [
      "<b>Baseline:</b> i-j-k walks B by column; VTune shows 38.5% memory-bound slots and 126 M LLC misses.",
      "<b>Locality:</b> i-k-j order streams rows (4.5×); tiling keeps blocks of A, B and C in cache (≈8×).",
      "<b>Compute:</b> a 4×4 register-blocked micro-kernel with manual prefetch reached 1.5 s; unrolling and compiler vectorization brought it to 0.7 s.",
      "<b>Reference:</b> OpenBLAS runs in ~0.07 s, a measure of how much is left."
    ],
    figs: [["gemm.webp", "Runtime by optimization step"]],
    hint: "tiles of A and B stay cache-resident"
  },
  {
    id: "bksb", cat: "perf", demo: "bksb",
    title: "Parallel Back-Substitution Solver",
    course: "ECE 6750 · Parallel Computing", team: "with Kian Mahmoodi",
    purpose: "Parallelize a triangular solver whose rows depend on each other, where the provided threaded version was slower than serial.",
    how: "Persistent threads, per-row parallel reductions with per-thread partial sums, cache-line padding, a serial cutoff and lock-free atomic synchronization, across eight versions.",
    metrics: [["0.38× → 2.69×", "vs. serial baseline"], ["8", "optimization versions"], ["lock-free", "atomic sync"]],
    tags: ["C", "pthreads", "Atomics"],
    steps: [
      "<b>Diagnose:</b> the naive version busy-waited on dependencies (0.38×); a mutex + condvar scheduler was even worse (0.27×).",
      "<b>Restructure:</b> persistent threads split each row's dot product; per-thread partial sums remove the lock (2.25×).",
      "<b>Memory:</b> padding partial sums to cache lines removes false sharing (2.51×).",
      "<b>Integrate:</b> adaptive thread activation, a serial cutoff for short rows and acquire/release atomics give the best result (0.282 s, 2.69×)."
    ],
    figs: [["bksb.webp", "Speedup sweep for n = 10k–50k, p = 1–20"]],
    hint: "each row's dot product split across threads"
  },
  {
    id: "spgemm", repo: "https://github.com/IvannnHuang/ECE6750-PA3", cat: "perf", demo: "spgemm",
    title: "Load-Balanced Sparse Matrix Multiply",
    course: "ECE 6750 · Parallel Computing", team: "with Kian Mahmoodi",
    purpose: "Parallelize Gustavson's sparse GEMM, the kernel behind sparse ML, when row costs are wildly uneven.",
    how: "Compared cyclic row assignment, OpenMP schedules and a partitioner that splits rows by predicted multiply count, then removed malloc from the critical path.",
    metrics: [["1.4× → 4.9×", "uneven matrix, p = 8"], ["≈ OpenMP", "within 5% at the end"], ["CSR", "row-wise Gustavson"]],
    tags: ["C", "pthreads", "OpenMP", "CSR"],
    steps: [
      "<b>Algorithm:</b> each output row is built by scaling and merging rows of B selected by the non-zeros of A, which fits CSR storage.",
      "<b>Naive:</b> cyclic rows work on the even matrix (5.6×) but stall on the uneven one (1.4×): a few heavy rows land on one thread.",
      "<b>Balance:</b> rows are partitioned by predicted operation count, so each thread gets equal work (4.9×).",
      "<b>Allocation:</b> per-thread accumulators are allocated once and reused, matching OpenMP dynamic scheduling within 5%."
    ],
    figs: [["spgemm.webp", "Relative speedup by implementation, p = 8"]],
    hint: "same rows: cyclic vs. op-count balanced"
  },
  {
    id: "rag", cat: "perf", demo: "rag",
    title: "RAG Pipeline Performance Study",
    course: "ECE 6765 · Team 07", team: "with Dennis Bui & Haoqin Xiu",
    purpose: "Find where time goes in a four-service retrieval-augmented generation pipeline on a 160-core Arm server, and what limits it.",
    how: "Instrumented every service boundary, characterized the server, built FP32 and integer Rooflines and ran top-down PMU analysis, then evaluated quantization, data parallelism and ANN search.",
    metrics: [["96.9%", "of time in generation"], ["23.6 GB/s", "per-socket DRAM roof"], ["3.79×", "quantized prediction"]],
    tags: ["Python", "perf / PMU", "Roofline", "Pixi"],
    steps: [
      "<b>Attribute:</b> generation takes 5.35 s/query against 140 ms embedding and 33 ms vector search, 96.9% of wall time.",
      "<b>Characterize (my part):</b> 2 × 80 Neoverse N1 cores, but one DDR4 channel per socket: reads saturate at 23.6 GB/s; remote NUMA latency costs 4.6×.",
      "<b>Diagnose (my part):</b> decode sits at 69–78% of the bandwidth Roofline, is 74–97% backend-bound and shows no gain from more cores, so it is memory-bound.",
      "<b>Optimize:</b> quantization cuts matrix bytes 3.76× for a 3.79× prediction speedup; I also owned the chunked DB + ANN search and run-to-run variation analysis."
    ],
    figs: [["rag-roofline.webp", "FP32 Roofline of the server with pipeline kernels"], ["rag-combined.webp", "Baseline vs. combined optimized configuration"]],
    hint: "generation dominates the timeline"
  }
];

const SKILLS = [
  ["Hardware design", "var(--c-silicon)", ["SystemVerilog / Verilog", "VHDL", "Microarchitecture", "PyMTL3", "Cadence Virtuoso", "KLayout", "Static timing", "Physical design"]],
  ["FPGA & HLS", "var(--c-fpga)", ["Vivado / Vitis", "Zynq AXI IP", "Quartus", "Allo HLS", "Fixed-point datapaths"]],
  ["Embedded", "var(--c-embedded)", ["C / C++", "RP2040/RP2350", "ESP32 / Artemis", "DMA · ISRs", "SPI · I²C · BLE"]],
  ["Robotics", "var(--c-robotics)", ["ROS", "Kalman / particle / Bayes filters", "RRT · PRM", "PID · MPC", "MATLAB"]],
  ["Performance", "var(--c-perf)", ["pthreads · OpenMP", "AVX intrinsics", "VTune · perf/PMU", "Roofline analysis", "Python · NumPy"]]
];
