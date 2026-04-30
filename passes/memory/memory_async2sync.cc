/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2026  Aditya Pawade
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  ---
 *
 *  memory_async2sync — promote asynchronous read ports on memories to synchronous
 *  by inserting an output flip-flop on each async port and re-marking the port
 *  CLK_ENABLE=1.  This is the inverse of memory_dff (which absorbs trailing FFs
 *  into a memory port), and is used when the target hardware (e.g. Gowin GW5A)
 *  has only sync-read BRAMs available and the source RTL uses async reads inside
 *  always_ff / always_comb that the FSM can absorb a 1-cycle latency for.
 *
 *  Behavior change: every async-read result that downstream consumers see is
 *  delayed by exactly one clock cycle.  For FSMs with multi-cycle states this
 *  is usually transparent; for bare combinational consumers it is observably
 *  different.  Use only when you've verified the design tolerates the delay.
 *
 *  Usage in synth flow (between memory_collect/_share and memory_libmap):
 *      memory_async2sync [-min-bits N] [-only-mem name1,name2,...]
 *
 *  Options:
 *      -min-bits N
 *          Only transform memories whose total bit count is >= N.  Useful for
 *          targeting only the large memories that benefit from BRAM mapping.
 *          Default: 1024 bits.
 *
 *      -only-mem N1,N2
 *          Comma-separated list of memory names; only these are transformed.
 *          Overrides -min-bits.
 */

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "kernel/mem.h"
#include "kernel/ff.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct MemoryAsync2SyncWorker {
	Module *module;
	int min_bits;
	pool<IdString> only_mem;

	int memories_transformed = 0;
	int ports_transformed = 0;

	MemoryAsync2SyncWorker(Module *m, int mb, pool<IdString> om)
		: module(m), min_bits(mb), only_mem(om) {}

	// Find a clock signal we can use to register an async port. Strategy:
	//   1. If any write port is clocked, take its clk + polarity.
	//   2. Otherwise, give up (return empty).  ROMs have no natural clock and
	//      are typically already small enough to stay in distributed RAM.
	bool find_clock(const Mem &mem, SigSpec &out_clk, bool &out_polarity)
	{
		for (auto &wr : mem.wr_ports) {
			if (wr.clk_enable && GetSize(wr.clk) > 0) {
				out_clk = wr.clk;
				out_polarity = wr.clk_polarity;
				return true;
			}
		}
		// fallback: any other read port that's already sync
		for (auto &rd : mem.rd_ports) {
			if (rd.clk_enable && GetSize(rd.clk) > 0) {
				out_clk = rd.clk;
				out_polarity = rd.clk_polarity;
				return true;
			}
		}
		return false;
	}

	void run()
	{
		std::vector<Mem> memories = Mem::get_all_memories(module);
		for (auto &mem : memories) {
			int total_bits = mem.width * mem.size;
			bool wanted = false;

			if (!only_mem.empty()) {
				if (only_mem.count(mem.memid)) wanted = true;
			} else {
				if (total_bits >= min_bits) wanted = true;
			}

			if (!wanted) continue;

			// Skip memories with no async read ports (already fully sync).
			bool any_async = false;
			for (auto &rd : mem.rd_ports) {
				if (!rd.clk_enable) { any_async = true; break; }
			}
			if (!any_async) continue;

			SigSpec clk;
			bool clk_polarity = true;
			if (!find_clock(mem, clk, clk_polarity)) {
				log("memory %s.%s has async read ports but no clock available; "
					"skipping.\n", log_id(module), log_id(mem.memid));
				continue;
			}

			bool changed = false;
			for (auto &rd : mem.rd_ports) {
				if (rd.clk_enable) continue;

				// Mark port as sync.  The $mem_v2 cell's own internal output
				// register absorbs the 1-cycle latency (yosys-libmap's
				// brams_map_gw5a then matches this against SDPB / SDP).
				// Consumers see the previous-cycle value of the addressed
				// location, which is acceptable for FSM-driven RTL with
				// multi-cycle states.
				rd.clk_enable = true;
				rd.clk = clk;
				rd.clk_polarity = clk_polarity;
				rd.ce_over_srst = false;

				ports_transformed++;
				changed = true;

				log("  promoted async port %d of %s.%s to sync (clk=%s pol=%d, +1cyc latency).\n",
					(int)(&rd - &mem.rd_ports[0]), log_id(module), log_id(mem.memid),
					log_signal(clk), clk_polarity ? 1 : 0);
			}

			if (changed) {
				mem.emit();
				memories_transformed++;
			}
		}
	}
};

struct MemoryAsync2SyncPass : public Pass {
	MemoryAsync2SyncPass() : Pass("memory_async2sync",
		"convert async-read memory ports to sync by inserting output DFFs") {}

	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    memory_async2sync [options] [selection]\n");
		log("\n");
		log("Promotes asynchronous read ports of selected memories to synchronous by\n");
		log("inserting an output flip-flop, so the resulting memory is BRAM-mappable on\n");
		log("targets whose block RAM is sync-read only (e.g. Gowin GW5A BSRAM).\n");
		log("\n");
		log("WARNING: this transformation introduces ONE EXTRA CYCLE OF READ LATENCY\n");
		log("on every promoted async port.  Downstream consumers see the previous-cycle\n");
		log("value of the addressed location.  Run only on designs where the FSMs have\n");
		log("multi-cycle states large enough to absorb that latency.\n");
		log("\n");
		log("Options:\n");
		log("\n");
		log("  -min-bits N\n");
		log("    Only transform memories whose total bit count (width * depth) is >= N.\n");
		log("    Default: 1024.\n");
		log("\n");
		log("  -only-mem name1,name2,...\n");
		log("    Comma-separated list of memory names to transform.  Overrides -min-bits.\n");
		log("\n");
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		log_header(design, "Executing MEMORY_ASYNC2SYNC pass.\n");

		int min_bits = 1024;
		pool<IdString> only_mem;
		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-min-bits" && argidx + 1 < args.size()) {
				min_bits = atoi(args[++argidx].c_str());
				continue;
			}
			if (args[argidx] == "-only-mem" && argidx + 1 < args.size()) {
				std::string list = args[++argidx];
				size_t pos = 0;
				while (pos < list.size()) {
					size_t comma = list.find(',', pos);
					std::string name = list.substr(pos, comma - pos);
					if (!name.empty()) {
						if (name[0] != '\\' && name[0] != '$')
							name = "\\" + name;
						only_mem.insert(IdString(name));
					}
					if (comma == std::string::npos) break;
					pos = comma + 1;
				}
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		int total_mems = 0, total_ports = 0;
		for (auto mod : design->selected_modules()) {
			MemoryAsync2SyncWorker worker(mod, min_bits, only_mem);
			worker.run();
			total_mems += worker.memories_transformed;
			total_ports += worker.ports_transformed;
		}

		log("memory_async2sync transformed %d memories (%d ports total).\n",
			total_mems, total_ports);
	}
} MemoryAsync2SyncPass;

PRIVATE_NAMESPACE_END
