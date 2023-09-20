/**************
 * Phy Engine *
 *************/

#pragma once

#include <type_traits>
#include <concepts>
#include <string_view>
#include <algorithm>

#include "../../phy_engine_utils/fast_io/fast_io_dsal/vector.h"
#include "../../phy_engine_utils/fast_io/fast_io_crypto.h"
#include "../freestanding/array_view.h"

namespace phy_engine::command_line {

enum class parameter_parsing_results_type : ::std::uint_fast8_t {
	arg = 0,
	occupied_arg,
	parameter,
	invalid_parameter,
	duplicate_parameter
};

struct parameter;
struct parameter_parsing_results {
	::std::u8string_view str{};
	parameter const* para{};
	parameter_parsing_results_type type{};
};

enum class parameter_return_type : ::std::uint_fast8_t {
	def = 0,
	return_imme,
	return_soon
};

struct parameter {
	::std::u8string_view const name{};
	::std::u8string_view const describe{};
	::phy_engine::freestanding::array_view<::std::u8string_view const> alias{};
	parameter_return_type (*callback)(::std::size_t argc, ::fast_io::vector<parameter_parsing_results>& vec) noexcept {};
	bool* is_exist{};
};

template <::std::size_t N>
inline consteval auto parameter_sort(parameter const* const (&punsort)[N]) noexcept {
	::fast_io::freestanding::array<parameter const*, N> res{};
	for (::std::size_t i{}; i < N; i++) {
		res[i] = punsort[i];
	}
	::std::ranges::sort(res, [](parameter const* const a, parameter const* const b) noexcept -> bool {
		return a->name < b->name;
	});
	return res;
}

template <::std::size_t N>
inline constexpr void parameter_clean(::fast_io::freestanding::array<parameter const*, N>& punsort) noexcept {
	for (auto i : punsort) {
		i.callback = nullptr;
	}
}

struct all_parameter {
	::std::u8string_view str{};
	parameter const* para{};
};

template <::std::size_t N>
inline consteval ::std::size_t calculate_all_parameters_size(::fast_io::freestanding::array<parameter const*, N> const& punsort) noexcept {
	::std::size_t res{};
	for (::std::size_t i{}; i < N; i++) {
		res++;
		res += punsort[i]->alias.size();
	}
	return res;
}

template <::std::size_t Nres, ::std::size_t N>
inline consteval auto expand_all_parameters_and_check(::fast_io::freestanding::array<parameter const*, N> const& punsort) noexcept {
	::fast_io::freestanding::array<all_parameter, Nres> res{};
	::std::size_t res_pos{};
	for (::std::size_t i{}; i < N; i++) {
		if (punsort[i]->is_exist == nullptr) 
			::fast_io::fast_terminate();  // The first character of the parameter must be '-'

		res[res_pos++] = {punsort[i]->name, punsort[i]};
		for (auto j : punsort[i]->alias) {
			res[res_pos++] = {j, punsort[i]};
		}
	}
	::std::ranges::sort(res, [](all_parameter const& a, all_parameter const& b) noexcept -> bool {
		return a.str < b.str;
	});
	::std::u8string_view check{};  // Empty strings will be sorted and placed first.
	for (auto& i : res) {
		if (i.str == check || i.str.front() != u8'-') {
			::fast_io::fast_terminate();  // The first character of the parameter must be '-'
		} else {
			if (i.str.size() == 1) 
				::fast_io::fast_terminate();  // "-" is invalid
			check = i.str;
		}
	}
	return res;
}

inline constexpr ::std::size_t hash_size_base{4u};
inline constexpr ::std::size_t max_conflict_size{8u};

struct calculate_hash_table_size_res {
	::std::size_t hash_table_size{};
	::std::size_t extra_size{};
};

template <::std::size_t N>
inline consteval calculate_hash_table_size_res calculate_hash_table_size(::fast_io::freestanding::array<all_parameter, N> const& ord) noexcept {
	constexpr auto sizet_d10{::std::numeric_limits<::std::size_t>::digits10};

	::fast_io::crc32c_context crc32c{};
	for (int i{hash_size_base}; i < sizet_d10; i++) {
		::std::size_t hash_size{static_cast<::std::size_t>(1u) << i};
		bool c{};
		::std::size_t extra_size{};
		::std::size_t* const hash_size_array{new ::std::size_t[hash_size]{}};
		for (auto &j : ord) {
			::std::size_t const j_str_size{j.str.size()};
			::std::byte* const ptr{new ::std::byte[j_str_size]{}};
			for (::std::size_t k{}; k < j_str_size; k++) {
				ptr[k] = static_cast<::std::byte>(j.str[k]);
			}
			crc32c.reset();
			crc32c.update(ptr, ptr + j_str_size);
			delete[] ptr;
			auto const val{crc32c.digest_value() % hash_size};
			hash_size_array[val]++;
			if (hash_size_array[val] == 2) {
				extra_size++;
			}
			if (hash_size_array[val] > max_conflict_size) {
				c = true;
			}
		}

		delete[] hash_size_array;
		if (!c) {
			return {hash_size, extra_size};
		}
	}
	::fast_io::fast_terminate(); // error
}

struct ht_para_cpos {
	::std::u8string_view str{};
	parameter const* para{};
	::std::size_t conflict_pos{}; // garbage private
};

#if 0
struct ct_para_str {
	::std::u8string_view str{};
	parameter const* para{};
};
#else
using ct_para_str = all_parameter;
#endif  // 0

struct conflict_table {
	::fast_io::freestanding::array<ct_para_str, max_conflict_size> ctmem{};
};

template <::std::size_t hash_table_size, ::std::size_t conflict_size>
struct parameters_hash_table {
	static_assert(hash_table_size > 1 && conflict_size > 1);

	::fast_io::freestanding::array<ht_para_cpos, hash_table_size> ht{};
	::fast_io::freestanding::array<conflict_table, conflict_size> ct{};
};

// to do generatr_hash_table  ************************************************************

template <::std::size_t hash_table_size, ::std::size_t conflict_size, ::std::size_t N>
inline consteval auto generate_hash_table(::fast_io::freestanding::array<all_parameter, N> const& ord) noexcept {
	parameters_hash_table<hash_table_size, conflict_size> res{};

	::fast_io::crc32c_context crc32c{};
	::std::size_t conflictplace{1u};

	for (auto& j : ord) {
		::std::size_t const j_str_size{j.str.size()};
		::std::byte* const ptr{new ::std::byte[j_str_size]{}};
		for (::std::size_t k{}; k < j_str_size; k++) {
			ptr[k] = static_cast<::std::byte>(j.str[k]);
		}
		crc32c.reset();
		crc32c.update(ptr, ptr + j_str_size);
		delete[] ptr;
		auto const val{crc32c.digest_value() % hash_table_size};
		if (res.ht[val].conflict_pos != 0) {
			for (auto& i : res.ct[res.ht[val].conflict_pos - 1].ctmem) {
				if (i.para == nullptr) {
					i.para = j.para;
					i.str = j.str;
					break;
				}
			}
		} else if (res.ht[val].para != nullptr) {
			res.ht[val].conflict_pos = conflictplace;
			res.ct[conflictplace - 1].ctmem[0].para = res.ht[val].para;
			res.ct[conflictplace - 1].ctmem[0].str = res.ht[val].str;
			res.ht[val].para = nullptr;
			res.ht[val].str = ::std::u8string_view{};
			res.ct[conflictplace - 1].ctmem[1].para = j.para;
			res.ct[conflictplace - 1].ctmem[1].str = j.str;
			conflictplace++;
		} else {
			res.ht[val].para = j.para;
			res.ht[val].str = j.str;
		}
	}
	return res;
}

template <::std::size_t hash_table_size, ::std::size_t conflict_size>
inline constexpr parameter const* find_from_hash_table(parameters_hash_table<hash_table_size, conflict_size> const& ht, ::std::u8string_view str) noexcept {
	::fast_io::crc32c_context crc32c{};

#if __cpp_if_consteval >= 202106L
	if consteval
#else
	if (__builtin_is_constant_evaluated())
#endif
	{
		auto const str_size{str.size()};
		::std::byte* const ptr{new ::std::byte[str_size]{}};
		for (::std::size_t k{}; k < str_size; k++) {
			ptr[k] = static_cast<::std::byte>(str[k]);
		}
		crc32c.update(ptr, ptr + str_size);
		delete[] ptr;
	} else {
		auto i{reinterpret_cast<::std::byte const*>(str.data())};
		crc32c.update(i, i + str.size());
	}
	auto const val{crc32c.digest_value() % hash_table_size};
	auto const htval{ht.ht[val]};
	if (htval.para != nullptr) {
		if (str == htval.str) {
			return htval.para;
		} else {
			return nullptr;
		}
	} else if (htval.conflict_pos != 0) {
		auto& ct{ht.ct[htval.conflict_pos - 1].ctmem};
		for (::std::size_t i{}; i < max_conflict_size; i++) {
			if (ct[i].str == str) {
				return ct[i].para;
			} else if (ct[i].para == nullptr) {
				return nullptr;
			}
		}
		return nullptr;
	} else {
		return nullptr;
	}
}

}  // namespace phy_engine::command_line