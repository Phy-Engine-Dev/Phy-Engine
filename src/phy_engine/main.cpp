﻿/**************
 * Phy Engine *
 *************/

#include "../phy_engine_utils/fast_io/fast_io.h" // fast_io

#include "devices/native_io.h" // native_io
#include "version/phy_engine.h"
#include "../phy_engine_utils/ansies/impl.h"
#include "../phy_engine_utils/command_line/impl.h"
#include "command_line/impl.h"
#include "command_line/parsing_result.h"
#include "operation/run.h"

int main(int argc, char** argv) noexcept {

	auto& parse_res{::phy_engine::parsing_result};

	int pr{::phy_engine::parsing(argc, reinterpret_cast<char8_t**>(argv), parse_res, ::phy_engine::hash_table)};

	if (pr != 0) {
		return pr;
	}

	if (parse_res.size() > 1) {
		if (parse_res[1].type != ::phy_engine::command_line::parameter_parsing_results_type::file) {
			constexpr auto& version_para{::phy_engine::parameter::version};
			constexpr auto& help_para{::phy_engine::parameter::help};
			constexpr auto& contributor_para{::phy_engine::parameter::contributor};

			if (*version_para.is_exist || *help_para.is_exist || *contributor_para.is_exist) {
				return 0;
			}
		}
	}

	return ::phy_engine::run();
}