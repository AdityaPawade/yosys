/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  memory_widen_mixed: pack a multi-port memory into a wider underlying storage
 *  to reduce the number of BRAM cells required after libmap.
 *
 *  For a memory M of width W and depth D with mixed-width ports (some narrow,
 *  some wide), this pass picks a target widening factor 2^K and rewrites M:
 *
 *    - Storage becomes width = W << K, depth = D >> K (same total bits).
 *    - Each read port at wide_log2 = lo (where lo < K) is rewritten to read
 *      the wider word with addr = old_addr >> K, then byte-position muxed by
 *      old_addr[K-1:0] to recover the narrow data.
 *    - Each write port with wide_log2 < K becomes a byte-enabled write to
 *      the wider word: address shifted down, data placed at byte position,
 *      wr_en mask gated to the appropriate byte lanes.
 *    - Read/write ports already at wide_log2 >= K need no muxing — they
 *      naturally span the wider word.
 *
 *  After this pass, memory_libmap sees a wider, shallower memory with
 *  fewer effective port count per BRAM cell — typically 1 BSRAM per memory
 *  even when the source had many narrow read ports at distinct addresses.
 *
 *  This addresses the gw5a fitting bottleneck: yosys's libmap allocates
 *  one BSRAM per sync read port, so designs with many narrow async reads
 *  on a single memory exceed the chip's BSRAM count.  Pre-widening the
 *  memory packs N byte reads into 1 word read on a wider BSRAM, with
 *  the same data semantically selected via byte-mux.
 *
 *  Cost: each narrow read adds one $bmux cell (~K LUTs).  Each narrow
 *  write adds byte-enable gating (~K LUTs).  These are typically less
 *  than the savings in BRAM count and the LUT-mapped FF storage that
 *  the memory would otherwise occupy.
 */

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "kernel/mem.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct MemoryWidenMixedWorker {
	Module *module;
	int min_bits;
	int target_log2;
	pool<IdString> only_mem;

	int memories_widened = 0;
	int rd_ports_muxed = 0;
	int wr_ports_byteen = 0;

	MemoryWidenMixedWorker(Module *m, int mb, int tl, pool<IdString> om)
		: module(m), min_bits(mb), target_log2(tl), only_mem(om) {}

	// 2026-05-12 (Codex thread 019e1b34): allow hierarchical-suffix match.
	// `-only-mem fdr_reader_inst.sector_buffer` now matches a memid like
	// `\top_p15b.u_risc.sd_drv_inst.fdr_reader_inst.sector_buffer` because the
	// pattern is preceded by `.` in the haystack. Exact match still works.
	// The `.` separator requirement prevents `foo_sector_buffer` from
	// accidentally matching pattern `sector_buffer`.
	bool memid_matches_only_mem(IdString memid)
	{
		std::string hay = memid.str();
		for (auto pat_id : only_mem) {
			std::string pat = pat_id.str();
			if (hay == pat)
				return true;
			if (!pat.empty() && pat[0] == '\\')
				pat = pat.substr(1);
			if (pat.empty())
				continue;
			if (hay.size() > pat.size() &&
			    hay.compare(hay.size() - pat.size(), pat.size(), pat) == 0 &&
			    hay[hay.size() - pat.size() - 1] == '.')
				return true;
		}
		return false;
	}

	void run() {
		auto mems = Mem::get_selected_memories(module);
		for (auto &mem : mems) {
			if (!only_mem.empty()) {
				if (!memid_matches_only_mem(mem.memid)) continue;
			} else {
				int total_bits = mem.width * mem.size;
				if (total_bits < min_bits) continue;
			}
			widen(mem);
		}
	}

	void widen(Mem &mem) {
		int K = target_log2;

		// Sanity: depth must be divisible by 2^K.
		if (mem.size & ((1 << K) - 1)) {
			log("memory_widen_mixed: skipping memory %s.%s (depth %d not multiple of %d).\n",
				log_id(module), log_id(mem.memid), mem.size, 1 << K);
			return;
		}

		// Sanity: at least one port must be NARROW (wide_log2 < K).  If all
		// ports are already at >= K, nothing to gain.
		bool any_narrow = false;
		for (auto &p : mem.rd_ports)
			if (p.wide_log2 < K) any_narrow = true;
		for (auto &p : mem.wr_ports)
			if (p.wide_log2 < K) any_narrow = true;
		if (!any_narrow) {
			log("memory_widen_mixed: skipping memory %s.%s (all ports already at wide_log2 >= %d).\n",
				log_id(module), log_id(mem.memid), K);
			return;
		}

		int new_width = mem.width << K;
		int new_depth = mem.size >> K;

		// Note: ports with wide_log2 > K are handled by simply reducing the port's
		// wide_log2 by K (it now spans 2^(L-K) wide words instead of 2^L base words).

		log("memory_widen_mixed: widening %s.%s from %d×%d to %d×%d (factor 2^%d).\n",
			log_id(module), log_id(mem.memid), mem.width, mem.size, new_width, new_depth, K);

		// --- read ports ---
		for (size_t i = 0; i < mem.rd_ports.size(); i++) {
			auto &rd = mem.rd_ports[i];
			if (rd.removed) continue;

			int L = rd.wide_log2;
			SigSpec narrow_data = rd.data;
			int narrow_width = mem.width << L;

			if (L >= K) {
				// Port spans >= 1 wide word; just reduce wide_log2 by K and
				// shift address.  No byte-mux needed.
				int new_wide_log2 = L - K;
				rd.addr = rd.addr.extract_end(K);
				rd.wide_log2 = new_wide_log2;
				// data/init/arst/srst sizes are unchanged (mem.width<<L == new_width<<(L-K))
			} else {
				// L < K: port reads narrow_width bits.  Replace data with a wider
				// wire (one wide word) and bmux out the requested slice.
				int new_wide_log2 = 0;
				SigSpec lo_addr = rd.addr.extract(0, K);
				SigSpec hi_addr = rd.addr.extract_end(K);
				SigSpec wide_data = module->addWire(NEW_ID, new_width << new_wide_log2);
				rd.data = wide_data;
				rd.addr = hi_addr;
				rd.wide_log2 = new_wide_log2;
				rd.init_value = Const(State::Sx, new_width << new_wide_log2);
				rd.arst_value = Const(State::Sx, new_width << new_wide_log2);
				rd.srst_value = Const(State::Sx, new_width << new_wide_log2);

				Cell *bmux = module->addCell(NEW_ID, ID($bmux));
				bmux->setParam(ID::WIDTH, narrow_width);
				bmux->setParam(ID::S_WIDTH, K - L);
				bmux->setPort(ID::A, wide_data);
				bmux->setPort(ID::S, lo_addr.extract(L, K - L));
				bmux->setPort(ID::Y, narrow_data);
				rd_ports_muxed++;
			}
		}

		// --- write ports ---
		for (size_t i = 0; i < mem.wr_ports.size(); i++) {
			auto &wr = mem.wr_ports[i];
			if (wr.removed) continue;

			int L = wr.wide_log2;
			SigSpec narrow_data = wr.data;
			SigSpec narrow_en = wr.en;
			int narrow_width = mem.width << L;

			if (L >= K) {
				int new_wide_log2 = L - K;
				wr.addr = wr.addr.extract_end(K);
				wr.wide_log2 = new_wide_log2;
				// data/en sizes unchanged
			} else {
				int new_wide_log2 = 0;
				SigSpec lo_addr = wr.addr.extract(0, K);
				SigSpec hi_addr = wr.addr.extract_end(K);

				int n_slots = 1 << (K - L);
				SigSpec wide_data;
				SigSpec wide_en;
				SigSpec slot_sel = lo_addr.extract(L, K - L);

				// Determine if narrow_en is "uniform" (all bits same signal).  If so,
				// we can produce a clean byte-en pattern: each slot has narrow_width
				// bits of EN that are all the same bit (slot_match AND narrow_en[0]).
				// That lets libmap recognise the byte-enable structure and use a
				// single wide BSRAM cell instead of one cell per bit lane.
				bool en_uniform = true;
				if (GetSize(narrow_en) > 0) {
					SigBit b0 = narrow_en[0];
					for (int b = 1; b < narrow_width; b++) {
						if (narrow_en[b] != b0) { en_uniform = false; break; }
					}
				}

				for (int slot = 0; slot < n_slots; slot++) {
					wide_data.append(narrow_data);
					SigSpec slot_match = module->Eq(NEW_ID, slot_sel, Const(slot, K - L));
					if (en_uniform) {
						// Single-bit EN: combine narrow_en[0] with slot_match,
						// then replicate that ONE bit across narrow_width.  libmap
						// should now see narrow_width identical EN bits per slot
						// = a clean byte-en structure.
						SigBit eff_en;
						if (narrow_en[0] == State::S1) {
							eff_en = SigSpec(slot_match)[0];
						} else {
							SigSpec g = module->And(NEW_ID, slot_match, SigSpec(narrow_en[0]));
							eff_en = g[0];
						}
						for (int b = 0; b < narrow_width; b++)
							wide_en.append(eff_en);
					} else {
						// Mixed EN signals — fall back to per-bit AND.
						SigSpec slot_match_rep;
						for (int b = 0; b < narrow_width; b++)
							slot_match_rep.append(slot_match);
						SigSpec slot_en = module->And(NEW_ID, narrow_en, slot_match_rep);
						wide_en.append(slot_en);
					}
				}
				wr.data = wide_data;
				wr.en = wide_en;
				wr.addr = hi_addr;
				wr.wide_log2 = new_wide_log2;
				wr_ports_byteen++;
			}
		}

		// --- init values ---
		// For now skip init values: implementing the repacking correctly under
		// arbitrary alignment is non-trivial (init blocks may not align with
		// 2^K boundaries).  We log a warning and leave inits untouched; for
		// memories that have init data, this pass should not be applied.
		if (!mem.inits.empty()) {
			log_warning("memory_widen_mixed: memory %s.%s has init values, "
				"which are not yet repacked by this pass.  Inits will be "
				"discarded.\n", log_id(module), log_id(mem.memid));
			for (auto &init : mem.inits)
				init.removed = true;
		}

		// --- update mem itself ---
		mem.width = new_width;
		mem.size = new_depth;
		mem.start_offset >>= K;

		mem.emit();
		memories_widened++;
	}
};

struct MemoryWidenMixedPass : public Pass {
	MemoryWidenMixedPass() : Pass("memory_widen_mixed", "pack mixed-width memories into wider storage") {}

	void help() override {
		log("\n");
		log("    memory_widen_mixed [-min-bits N] [-target-log2 K] [-only-mem name1,...] [selection]\n");
		log("\n");
		log("Pack memories with mixed narrow/wide port widths into wider underlying\n");
		log("storage so memory_libmap can map them to fewer BSRAM cells.  Each narrow\n");
		log("port gets a $bmux on read or byte-enabled write to recover its original\n");
		log("semantics from the wider word.\n");
		log("\n");
		log("    -min-bits N          only widen memories with width*depth >= N (default 1024)\n");
		log("    -target-log2 K       widening factor (default 2 = 4x).  Result width = base << K.\n");
		log("    -only-mem name1,...  comma-separated allowlist; names match exact or hierarchical suffix (preceded by '.')\n");
		log("\n");
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override {
		log_header(design, "Executing MEMORY_WIDEN_MIXED pass.\n");

		int min_bits = 1024;
		int target_log2 = 2;
		pool<IdString> only_mem;
		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-min-bits" && argidx + 1 < args.size()) {
				min_bits = atoi(args[++argidx].c_str());
				continue;
			}
			if (args[argidx] == "-target-log2" && argidx + 1 < args.size()) {
				target_log2 = atoi(args[++argidx].c_str());
				continue;
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

		int total_mems = 0, total_rd = 0, total_wr = 0;
		for (auto mod : design->selected_modules()) {
			MemoryWidenMixedWorker worker(mod, min_bits, target_log2, only_mem);
			worker.run();
			total_mems += worker.memories_widened;
			total_rd += worker.rd_ports_muxed;
			total_wr += worker.wr_ports_byteen;
		}
		log("memory_widen_mixed: widened %d memories (%d rd ports muxed, %d wr ports byte-enabled).\n",
			total_mems, total_rd, total_wr);
	}
} MemoryWidenMixedPass;

PRIVATE_NAMESPACE_END
