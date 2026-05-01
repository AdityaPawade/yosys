/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  memory_demote_small: tag memories with ram_style="logic" so
 *  memory_libmap forces them to FF mapping.  Use to free BSRAM cells when
 *  block-RAM count is the binding constraint.
 *
 *  Three selection modes:
 *      -max-bits N             demote any memory with width*depth < N
 *      -only-mem name1,name2,  demote only the listed memories (by memid).
 *      -only-mem-with-ports K  demote any memory whose total port count >= K.
 *  Multiple modes are AND'd unless -only-mem is given (which overrides).
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
			"tag memories with ram_style=logic to skip BRAM mapping") {}

	void help() override {
		log("\n");
		log("    memory_demote_small [options] [selection]\n");
		log("\n");
		log("Sets the ram_style=\"logic\" attribute on the selected memories so a\n");
		log("subsequent memory_libmap pass skips them and falls back to FF mapping.\n");
		log("\n");
		log("    -max-bits N\n");
		log("        Demote memories with width*depth < N (default: 2048).  Set to 0 to\n");
		log("        disable the size filter.\n");
		log("\n");
		log("    -only-mem name1,name2,...\n");
		log("        Comma-separated allowlist of memory names.  When set, the size\n");
		log("        filter is ignored and only listed memories are demoted.  Names\n");
		log("        match by suffix (e.g. \"mbr_reader_inst.sector_buffer\" matches the\n");
		log("        full \\\\u_risc.fw_inst.sd_drv_inst.mbr_reader_inst.sector_buffer).\n");
		log("\n");
		log("    -min-ports K\n");
		log("        Only demote memories with read+write port count >= K.  Combined\n");
		log("        with -max-bits as AND.  Default: 0 (no port-count filter).\n");
		log("\n");
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override {
		log_header(design, "Executing MEMORY_DEMOTE_SMALL pass.\n");

		int max_bits = 2048;
		int min_ports = 0;
		std::vector<std::string> only_mem;
		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-max-bits" && argidx + 1 < args.size()) {
				max_bits = atoi(args[++argidx].c_str());
				continue;
			}
			if (args[argidx] == "-min-ports" && argidx + 1 < args.size()) {
				min_ports = atoi(args[++argidx].c_str());
				continue;
			}
			if (args[argidx] == "-only-mem" && argidx + 1 < args.size()) {
				std::string list = args[++argidx];
				size_t pos = 0, prev = 0;
				while ((pos = list.find(',', prev)) != std::string::npos) {
					std::string name = list.substr(prev, pos - prev);
					if (!name.empty()) only_mem.push_back(name);
					prev = pos + 1;
				}
				std::string name = list.substr(prev);
				if (!name.empty()) only_mem.push_back(name);
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
				int n_ports = (int)mem.rd_ports.size() + (int)mem.wr_ports.size();

				// Decide if we want to demote this memory.
				bool wanted;
				if (!only_mem.empty()) {
					// only-mem mode: name suffix match overrides size filter.
					std::string memid = std::string(mem.memid.c_str());
					if (!memid.empty() && memid[0] == '\\')
						memid = memid.substr(1);
					wanted = false;
					for (const auto &needle : only_mem) {
						if (memid.find(needle) != std::string::npos) {
							wanted = true;
							break;
						}
					}
				} else {
					wanted = (max_bits == 0 || bits < max_bits);
					if (n_ports < min_ports)
						wanted = false;
				}
				if (!wanted) continue;
				if (mem.attributes.count(ID::ram_style)) continue;

				mem.attributes[ID::ram_style] = RTLIL::Const("logic");
				mem.emit();
				total_demoted++;
				log("memory_demote_small: %s.%s (%d bits, %d ports) -> ram_style=logic\n",
					log_id(module->name), log_id(mem.memid), bits, n_ports);
			}
		}
		log("memory_demote_small: demoted %d memories.\n", total_demoted);
	}
} MemoryDemoteSmallPass;

PRIVATE_NAMESPACE_END
