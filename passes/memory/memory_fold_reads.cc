/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  memory_fold_reads: collapse multiple read ports of a memory into a single
 *  shared read port by muxing the address based on the per-port EN signals.
 *
 *  ASSUMES the original ports are MUTUALLY EXCLUSIVE — only one EN is high
 *  per cycle.  This is true for FSM-driven designs where each port is gated
 *  by a state-decode signal.  This pass does NOT prove the assumption (use
 *  memory_share -sat for proven-safe folding) — it trusts the user's design.
 *
 *  When two ENs would simultaneously be high, behavior is undefined (one of
 *  the addresses wins; the consumer of the loser gets stale or wrong data).
 *
 *  Why this exists: yosys's memory_libmap allocates one BRAM cell per sync
 *  read port.  Multi-port memories blow the BSRAM budget on FPGAs with
 *  small block-RAM counts (e.g. Gowin GW5A-25 with 56 BSRAMs).  Folding to
 *  one port lets a multi-megabyte sector_buffer fit in 1 BSRAM.
 */

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "kernel/mem.h"
#include <queue>

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct MemoryFoldReadsWorker {
	Module *module;
	int min_bits;
	int min_ports;
	pool<IdString> only_mem;

	// SigMap to canonicalise signals before lookup.
	SigMap sigmap;
	// Map: each driven SigBit -> set of cells that consume it.
	dict<SigBit, pool<Cell *>> bit_to_consumers;

	int memories_folded = 0;
	int ports_eliminated = 0;

	MemoryFoldReadsWorker(Module *m, int mb, int mp, pool<IdString> om)
		: module(m), min_bits(mb), min_ports(mp), only_mem(om), sigmap(m) {
		build_consumer_index();
	}

	void build_consumer_index() {
		for (auto cell : module->cells()) {
			for (auto &conn : cell->connections()) {
				if (!cell->input(conn.first)) continue;
				for (auto &bit : sigmap(conn.second))
					bit_to_consumers[bit].insert(cell);
			}
		}
	}

	// Try to find a "select" signal for an async-read port by walking from
	// port.data into the consuming logic and finding a $mux whose A or B
	// input is port.data and whose S input is a single-bit signal that
	// (heuristically) decides whether this port's read result is observed.
	SigSpec extract_active_signal(SigSpec data) {
		if (data.empty()) return SigSpec();
		// BFS through consumer cells starting at data[0], looking for either
		//   - a $mux/$pmux gated by a single-bit S (use S as active signal); or
		//   - a $dffe/$dff with EN (use EN).
		// Walk through pass-through cells ($shiftx, $shift, simple bitops) so
		// reads that go through byte-extract logic before hitting a state
		// mux can still be folded.
		SigBit b0 = sigmap(data[0]);
		if (!bit_to_consumers.count(b0)) return SigSpec();
		pool<SigBit> visited;
		std::queue<SigBit> q;
		q.push(b0);
		visited.insert(b0);
		const int max_depth = 6;
		int depth = 0;
		while (!q.empty() && depth < max_depth) {
			int sz = (int)q.size();
			for (int i = 0; i < sz; i++) {
				SigBit cur = q.front(); q.pop();
				if (!bit_to_consumers.count(cur)) continue;
				for (auto cell : bit_to_consumers.at(cur)) {
					// $mux: data on A or B → S is the active signal.
					if (cell->type == ID($mux)) {
						SigSpec s = cell->getPort(ID::S);
						if (GetSize(s) != 1) continue;
						SigSpec a = cell->getPort(ID::A);
						SigSpec by = cell->getPort(ID::B);
						bool data_in_a = false, data_in_b = false;
						for (auto &x : sigmap(a)) if (x == cur) { data_in_a = true; break; }
						for (auto &x : sigmap(by)) if (x == cur) { data_in_b = true; break; }
						if (!data_in_a && !data_in_b) continue;
						if (data_in_b) return SigSpec(s);
						else return module->Not(NEW_ID, SigSpec(s));
					}
					// $pmux: Y = case S of B[0..W-1]; else A.  If data feeds B[k*W..(k+1)*W-1]
					// for some k, that "case" is selected when S[k]=1 → active = S[k].
					if (cell->type == ID($pmux)) {
						SigSpec s = cell->getPort(ID::S);
						SigSpec a = cell->getPort(ID::A);
						SigSpec by = cell->getPort(ID::B);
						int W = GetSize(a);
						int N = GetSize(s);
						if (N < 1 || GetSize(by) != W * N) continue;
						// Find which B-slot contains our `cur` bit.
						int found_k = -1;
						SigSpec by_mapped = sigmap(by);
						for (int k = 0; k < N; k++) {
							for (int b = 0; b < W; b++) {
								if (by_mapped[k * W + b] == cur) { found_k = k; break; }
							}
							if (found_k >= 0) break;
						}
						if (found_k >= 0) {
							return SigSpec(s[found_k]);
						}
						// Maybe data feeds A (default case).  Active = !|S (none of cases match).
						SigSpec a_mapped = sigmap(a);
						bool in_a = false;
						for (auto &x : a_mapped) if (x == cur) { in_a = true; break; }
						if (in_a) {
							SigSpec or_s = module->ReduceOr(NEW_ID, s);
							return module->Not(NEW_ID, or_s);
						}
						continue;
					}
					// FF with EN: that EN gates capture of this data.
					if (cell->type.in(ID($dff), ID($dffe), ID($adff), ID($adffe), ID($sdff), ID($sdffe))) {
						if (cell->hasPort(ID::EN)) {
							SigSpec en = cell->getPort(ID::EN);
							if (GetSize(en) == 1) return SigSpec(en);
						}
					}
					// Pass-through: chase Y output forward.
					if (cell->type.in(ID($shiftx), ID($shift), ID($not), ID($pos), ID($neg),
					                   ID($logic_not), ID($and), ID($or), ID($xor), ID($xnor),
					                   ID($reduce_or), ID($reduce_and), ID($reduce_xor),
					                   ID($add), ID($sub), ID($eq), ID($ne), ID($lt), ID($le),
					                   ID($gt), ID($ge), ID($bmux), ID($demux))) {
						if (cell->hasPort(ID::Y)) {
							for (auto &b : sigmap(cell->getPort(ID::Y))) {
								if (visited.insert(b).second) q.push(b);
							}
						}
					}
				}
			}
			depth++;
		}
		return SigSpec();
	}

	void run() {
		auto mems = Mem::get_selected_memories(module);
		for (auto &mem : mems) {
			if (!only_mem.empty()) {
				if (!only_mem.count(mem.memid)) continue;
			} else {
				int total_bits = mem.width * mem.size;
				if (total_bits < min_bits) continue;
				int active_rd = 0;
				for (auto &rd : mem.rd_ports) if (!rd.removed) active_rd++;
				if (active_rd < min_ports) continue;
			}
			fold(mem);
		}
	}

	void fold(Mem &mem) {
		// Group rd_ports by (clk_enable, clk_polarity, clk, wide_log2,
		// arst, srst, ce_over_srst).  Within a group all ports share the same
		// timing/reset behaviour and can be folded into one.
		struct Key {
			bool clk_enable;
			bool clk_polarity;
			SigBit clk;
			int wide_log2;
			SigBit arst, srst;
			bool ce_over_srst;
			bool operator<(const Key &o) const {
				return std::tie(clk_enable, clk_polarity, clk, wide_log2, arst, srst, ce_over_srst) <
					std::tie(o.clk_enable, o.clk_polarity, o.clk, o.wide_log2, o.arst, o.srst, o.ce_over_srst);
			}
		};
		std::map<Key, std::vector<int>> groups;
		for (size_t i = 0; i < mem.rd_ports.size(); i++) {
			auto &rd = mem.rd_ports[i];
			if (rd.removed) continue;
			Key k;
			k.clk_enable = rd.clk_enable;
			k.clk_polarity = rd.clk_polarity;
			k.clk = rd.clk_enable ? rd.clk[0] : SigBit(State::Sx);
			k.wide_log2 = rd.wide_log2;
			k.arst = rd.arst[0];
			k.srst = rd.srst[0];
			k.ce_over_srst = rd.ce_over_srst;
			groups[k].push_back((int)i);
		}

		bool changed = false;
		for (auto &kv : groups) {
			auto &idxs = kv.second;
			if ((int)idxs.size() < 2) continue;  // nothing to fold

			// Pick port[0] as the survivor.  Fold ports[1..] into it.
			auto &surv = mem.rd_ports[idxs[0]];
			int abits = GetSize(surv.addr);
			int data_width = GetSize(surv.data);

			// Build a chain of conditional address selects:
			//   addr_chain = en_n ? addr_n : (...)
			// We need a per-port "active" signal to decide which address wins
			// each cycle.  For ASYNC reads, port.en is required to be S1, so
			// it can't double as the active signal — we synthesise an active
			// signal by checking if any of the consumer's data bits are
			// observed.  For now, fall back to the simplest scheme:
			//   - Default address = surv.addr.
			//   - Each non-survivor's address overrides if its own consumers
			//     gate observation; we use the OR of the original ENs as the
			//     "select" signal regardless of clk_enable.
			//
			// Note: original port.en values are saved before we overwrite them.
			std::vector<SigSpec> orig_en;
			for (size_t j = 0; j < idxs.size(); j++)
				orig_en.push_back(mem.rd_ports[idxs[j]].en);

			// Save original data wires so we can rewire them post-fold.
			std::vector<SigSpec> orig_data;
			for (size_t j = 0; j < idxs.size(); j++)
				orig_data.push_back(mem.rd_ports[idxs[j]].data);

			SigSpec addr_chain = surv.addr;
			int folded_into_surv = 0;
			for (size_t j = 1; j < idxs.size(); j++) {
				auto &other = mem.rd_ports[idxs[j]];
				SigSpec other_addr = other.addr;
				if (GetSize(other_addr) < abits) other_addr.extend_u0(abits, false);
				if (GetSize(other_addr) > abits)
					other_addr = other_addr.extract(0, abits);

				// Determine select signal.
				SigSpec sel;
				if (orig_en[j] == State::S1) {
					// Async port — extract from consumer $mux.
					sel = extract_active_signal(orig_data[j]);
					if (sel.empty()) {
						log("memory_fold_reads: cannot find active signal for port %zu of %s.%s — skipping.\n",
							idxs[j], log_id(module), log_id(mem.memid));
						continue;
					}
				} else {
					sel = orig_en[j];
				}
				addr_chain = module->Mux(NEW_ID, addr_chain, other_addr, sel);
				folded_into_surv++;
			}

			if (folded_into_surv == 0) continue;  // nothing folded for this group
			surv.addr = addr_chain;
			// Leave surv.en untouched (S1 for async, original for sync).

			// Connect each non-survivor's data to the survivor's data so that
			// consumers see the (now-shared) read result.  Then mark removed.
			for (size_t j = 1; j < idxs.size(); j++) {
				auto &other = mem.rd_ports[idxs[j]];
				if (other.removed) continue;
				int other_dw = GetSize(orig_data[j]);
				if (other_dw != data_width) {
					log_warning("memory_fold_reads: skipping data-width mismatch port %zu of %s.%s (%d vs %d).\n",
						idxs[j], log_id(module), log_id(mem.memid), other_dw, data_width);
					continue;
				}
				module->connect(orig_data[j], surv.data);
				other.removed = true;
				ports_eliminated++;
			}
			changed = true;
		}

		if (changed) {
			mem.emit();
			memories_folded++;
			log("memory_fold_reads: folded read ports of %s.%s (%d ports remain).\n",
				log_id(module), log_id(mem.memid),
				(int)std::count_if(mem.rd_ports.begin(), mem.rd_ports.end(),
					[](const MemRd &p) { return !p.removed; }));
		}
	}
};

struct MemoryFoldReadsPass : public Pass {
	MemoryFoldReadsPass() : Pass("memory_fold_reads", "fold mutually-exclusive read ports into one") {}

	void help() override {
		log("\n");
		log("    memory_fold_reads [options] [selection]\n");
		log("\n");
		log("Collapse multiple read ports of a memory into a single shared read port,\n");
		log("muxing the address by EN signal.  ASSUMES mutual exclusion across ports.\n");
		log("\n");
		log("    -min-bits N         only fold memories with width*depth >= N (default 4096)\n");
		log("    -min-ports N        only fold memories with >= N read ports (default 2)\n");
		log("    -only-mem name1,... allowlist\n");
		log("\n");
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override {
		log_header(design, "Executing MEMORY_FOLD_READS pass.\n");

		int min_bits = 4096;
		int min_ports = 2;
		pool<IdString> only_mem;
		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-min-bits" && argidx + 1 < args.size()) {
				min_bits = atoi(args[++argidx].c_str()); continue;
			}
			if (args[argidx] == "-min-ports" && argidx + 1 < args.size()) {
				min_ports = atoi(args[++argidx].c_str()); continue;
			}
			if (args[argidx] == "-only-mem" && argidx + 1 < args.size()) {
				std::string list = args[++argidx];
				size_t pos = 0, prev = 0;
				while ((pos = list.find(',', prev)) != std::string::npos) {
					std::string name = list.substr(prev, pos - prev);
					if (!name.empty()) {
						if (name[0] != '\\' && name[0] != '$')
							name = "\\" + name;
						only_mem.insert(name);
					}
					prev = pos + 1;
				}
				std::string name = list.substr(prev);
				if (!name.empty()) {
					if (name[0] != '\\' && name[0] != '$')
						name = "\\" + name;
					only_mem.insert(name);
				}
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		int total_mems = 0, total_eliminated = 0;
		for (auto mod : design->selected_modules()) {
			MemoryFoldReadsWorker w(mod, min_bits, min_ports, only_mem);
			w.run();
			total_mems += w.memories_folded;
			total_eliminated += w.ports_eliminated;
		}
		log("memory_fold_reads: folded %d memories (%d redundant ports eliminated).\n",
			total_mems, total_eliminated);
	}
} MemoryFoldReadsPass;

PRIVATE_NAMESPACE_END
