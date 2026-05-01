/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  memory_demote_small: tag memories below a bit-count threshold with
 *  ram_style="logic" so memory_libmap forces them to FF mapping.
 *
 *  Use when BSRAM count is the binding constraint and the design has many
 *  small memories (e.g. register file, status string buffers) that don't
 *  benefit much from BRAM mapping but consume one BSRAM cell each.
 *
 *  Usage: memory_demote_small [-max-bits N] [selection]
 *      -max-bits N    demote any memory with width*depth < N (default 2048)
 *
 *  Should run BEFORE memory_libmap.
 */

#include "kernel/yosys.h"
#include "kernel/mem.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct MemoryDemoteSmallPass : public Pass {
	MemoryDemoteSmallPass()
		: Pass("memory_demote_small",
			"tag small memories with ram_style=logic to skip BRAM mapping") {}

	void help() override {
		log("\n");
		log("    memory_demote_small [-max-bits N] [selection]\n");
		log("\n");
		log("Sets the ram_style=\"logic\" attribute on every selected memory whose total\n");
		log("size (width * depth) is strictly less than N bits.  Use BEFORE memory_libmap\n");
		log("when you want to free BSRAM cells for larger memories.\n");
		log("\n");
		log("    -max-bits N\n");
		log("        Demote memories with width*depth < N (default 2048).\n");
		log("\n");
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override {
		log_header(design, "Executing MEMORY_DEMOTE_SMALL pass.\n");

		int max_bits = 2048;
		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-max-bits" && argidx + 1 < args.size()) {
				max_bits = atoi(args[++argidx].c_str());
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		int total_demoted = 0;
		for (auto module : design->selected_modules()) {
			auto mems = Mem::get_selected_memories(module);
			for (auto &mem : mems) {
				int bits = mem.width * mem.size;
				if (bits >= max_bits) continue;
				// Skip if already has a ram_style attribute set.
				if (mem.attributes.count(ID::ram_style)) continue;
				mem.attributes[ID::ram_style] = RTLIL::Const("logic");
				mem.emit();
				total_demoted++;
				log("memory_demote_small: %s.%s (%d bits) -> ram_style=logic\n",
					log_id(module->name), log_id(mem.memid), bits);
			}
		}
		log("memory_demote_small: demoted %d memories with < %d bits.\n",
			total_demoted, max_bits);
	}
} MemoryDemoteSmallPass;

PRIVATE_NAMESPACE_END
