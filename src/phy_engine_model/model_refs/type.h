#pragma once
#include <cstdint>
#include <cstddef>

namespace phy_engine::model {
enum class model_type : ::std::size_t {

};

enum class model_device_type : ::std::uint_fast8_t {
	linear,
	non_linear,
	digital
};
}