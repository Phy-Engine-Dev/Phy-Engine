#include "help.h"
#include "../parameters.h"
#include "../../../phy_engine_utils/ansies/impl.h"

::phy_engine::command_line::parameter_return_type(::phy_engine::parameter::details::help_callback)(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept {
	::fast_io::basic_obuf<::fast_io::u8native_io_observer> buf_u8out{::phy_engine::u8out};
	::fast_io::io::print(buf_u8out, ::phy_engine::ansi_escape_sequences::rst::all, u8"Arguments:\n");
	for (auto& p : ::phy_engine::parameters) {
		::fast_io::io::print(buf_u8out, p->name);
		if (p->alias.array_size != 0) {
			::fast_io::io::print(buf_u8out, u8" (aka: ");
			for (::std::size_t i{}; i < p->alias.array_size - 1; i++) {
				::fast_io::io::print(buf_u8out, p->alias[i], u8", ");
			}
			::fast_io::io::print(buf_u8out, p->alias.back(), ::fast_io::mnp::chvw(u8')'));
		}
		::fast_io::io::println(buf_u8out, u8" --- ", p->describe);
	}
	return ::phy_engine::command_line::parameter_return_type::def;
}