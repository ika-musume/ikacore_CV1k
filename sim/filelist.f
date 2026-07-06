// ikacore_CV1k board simulation - source manifest (Verilator / any SV sim)
// ip_cores/HS3 is a symlink to the read-only SH-3 IP; files are READ only,
// all build output goes to sim/build (see build_sim.sh --Mdir).

// --- packages + interfaces first (dependency order) ---
ip_cores/HS3/cpu_core/cpu_bus_if.sv
ip_cores/HS3/cpu_core/cache_pkg.sv
ip_cores/HS3/cpu_core/int_pipe_pkg.sv
ip_cores/HS3/peri/peri_bus_if.sv

// --- SH-3 CPU core ---
ip_cores/HS3/cpu_core/cache_mem.sv
ip_cores/HS3/cpu_core/cache.sv
ip_cores/HS3/cpu_core/agu.sv
ip_cores/HS3/cpu_core/int_pipe_mem.sv
ip_cores/HS3/cpu_core/int_pipe.sv
ip_cores/HS3/cpu_core/ctrl_reg.sv
ip_cores/HS3/cpu_core/exc_handler.sv
ip_cores/HS3/cpu_core/cpu_core.sv

// --- SH7709S peripherals + chip top ---
ip_cores/HS3/peri/ibus_splitter.sv
ip_cores/HS3/peri/ibus_bridge.sv
ip_cores/HS3/peri/bsc.sv
ip_cores/HS3/peri/cpg_wdt.sv
ip_cores/HS3/peri/intc.sv
ip_cores/HS3/peri/ioport.sv
ip_cores/HS3/peri/rtc.sv
ip_cores/HS3/peri/tmu.sv
ip_cores/HS3/HS3.sv

// --- board: vendor memory models (patched for Verilator, see *.verilator.patch) ---
models/mt48lc2m32b2.v
models/MX29LV320E.v

// --- PCB top + testbench ---
ikacore_CV1k.sv
tb/cpu_tracer.sv
tb/tb_cv1k.sv
