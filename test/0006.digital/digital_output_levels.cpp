#include <phy_engine/phy_engine.h>
#include <phy_engine/circuits/mixed_signal.h>
#include <phy_engine/model/models/digital/combinational/t_ff.h>
#include <phy_engine/model/models/digital/combinational/t_bar_ff.h>
#include <phy_engine/model/models/digital/combinational/jk_ff.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace pe = phy_engine;
static void close(double a,double b) { assert(std::isfinite(a) && std::abs(a-b)<1e-10); }
template<class T> void attributes(T& m,std::size_t lo,std::size_t hi)
{
    using pe::model::variant_type;
    assert(set_attribute_define(pe::model::model_reserve_type<T>,m,lo,{.d{-1},.type{variant_type::d}}));
    assert(set_attribute_define(pe::model::model_reserve_type<T>,m,hi,{.d{3.3},.type{variant_type::d}}));
    close(get_attribute_define(pe::model::model_reserve_type<T>,m,lo).d,-1);
    close(get_attribute_define(pe::model::model_reserve_type<T>,m,hi).d,3.3);
    assert(get_attribute_name_define(pe::model::model_reserve_type<T>,lo)==u8"Ll");
    assert(get_attribute_name_define(pe::model::model_reserve_type<T>,hi)==u8"Hl");
    assert(!set_attribute_define(pe::model::model_reserve_type<T>,m,hi,{.d{std::numeric_limits<double>::infinity()},.type{variant_type::d}}));
    assert(!set_attribute_define(pe::model::model_reserve_type<T>,m,lo,{.d{std::numeric_limits<double>::quiet_NaN()},.type{variant_type::d}}));
    assert(!set_attribute_define(pe::model::model_reserve_type<T>,m,hi,{.boolean{true},.type{variant_type::boolean}}));
}
static void input_loaded()
{
    for(auto state:{pe::model::digital_node_statement_t::L,pe::model::digital_node_statement_t::H})
    {
        pe::circult c{}; c.at=pe::analyze_type::TR;
        auto& g=pe::netlist::get_ground_node(c.nl); auto& out=pe::netlist::create_node(c.nl);
        pe::model::INPUT input{.outputA=state}; attributes(input,1,2);
        auto [p,pp]=pe::netlist::add_model(c.nl,std::move(input));
        auto [r,rp]=pe::netlist::add_model(c.nl,pe::model::resistance{.r=100});
        assert(pe::netlist::add_to_node(c.nl,*p,0,out));
        assert(pe::netlist::add_to_node(c.nl,*r,0,out)); assert(pe::netlist::add_to_node(c.nl,*r,1,g));
        // Explicit mixed-signal clocking is required by the upstream API.
        c.prepare(); assert(pe::solve_mixed_signal(c));
        close(out.node_information.an.voltage.real(),state==pe::model::digital_node_statement_t::H?3.3:-1);
    }
}
template<class T> void gate_drive(T m,std::size_t output_pin)
{
    attributes(m,0,1);
    pe::digital::digital_node_update_table table{};
    pe::model::node_t node[4]{};
    for(std::size_t i=0;i<output_pin;++i)
    {
        node[i].node_information.dn.state=pe::model::digital_node_statement_t::H;
        m.pins[i].nodes=&node[i];
    }
    m.pins[output_pin].nodes=&node[output_pin]; node[output_pin].num_of_analog_node=1;
    if constexpr(requires { m.Tsu; }) { m.Tsu=0;m.Th=0;node[0].node_information.dn.state=pe::model::digital_node_statement_t::L; }
    if constexpr(std::is_same_v<T,pe::model::T_BAR_FF>) node[0].node_information.dn.state=pe::model::digital_node_statement_t::L;
    if constexpr(std::is_same_v<T,pe::model::JKFF>) node[1].node_information.dn.state=pe::model::digital_node_statement_t::L;
    auto drive=update_digital_clk_define(pe::model::model_reserve_type<T>,m,table,1.0,pe::model::digital_update_method_t::update_table);
    assert(drive.need_to_operate_analog_node==&node[output_pin]); close(drive.voltage,3.3);
    // A held clock/input must retain the configured output, not revert to 5 V.
    drive=update_digital_clk_define(pe::model::model_reserve_type<T>,m,table,2.0,pe::model::digital_update_method_t::update_table);
    close(drive.voltage,3.3);
}
static void observer_thresholds()
{
    using state = pe::model::digital_node_statement_t;
    using pe::model::variant_type;
    pe::model::OUTPUT output{};
    attributes(output,1,2);
    assert(!set_attribute_define(pe::model::model_reserve_type<pe::model::OUTPUT>,output,0,
                                {.digital{state::H},.type{variant_type::digital}}));
    assert(!set_attribute_define(pe::model::model_reserve_type<pe::model::OUTPUT>,output,0,
                                {.d{1},.type{variant_type::d}}));
    assert(get_attribute_define(pe::model::model_reserve_type<pe::model::OUTPUT>,output,0).type==variant_type::digital);
    assert(get_attribute_name_define(pe::model::model_reserve_type<pe::model::OUTPUT>,0)==u8"value");
    // Real analog-node observations use the configured -1/3.3 V thresholds,
    // not legacy 0/5 V. No analog drive is created by this input-only device.
    pe::model::node_t node{}; node.num_of_analog_node=1; output.pins.nodes=&node;
    pe::digital::digital_node_update_table table{};
    auto read = [&](double voltage, double time)
    {
        node.node_information.an.voltage={voltage,0};
        auto drive=update_digital_clk_define(pe::model::model_reserve_type<pe::model::OUTPUT>,output,
                                            table,time,pe::model::digital_update_method_t::update_table);
        assert(drive.need_to_operate_analog_node==nullptr);
        return get_attribute_define(pe::model::model_reserve_type<pe::model::OUTPUT>,output,0).digital;
    };
    assert(read(3.4,1.0)==state::H);
    assert(read(-1.1,2.0)==state::X); // Original transition settling time remains intact.
    assert(read(-1.1,2.1)==state::L);
    assert(read(0.0,3.0)==state::L); // Above Ll but below Hl retains the low state.
    assert(read(3.4,4.0)==state::X);
    assert(read(3.4,4.1)==state::H);
    // Pure-digital observation is still direct and unchanged by analog thresholds.
    node.num_of_analog_node=0;node.node_information.dn.state=state::Z;
    auto drive=update_digital_clk_define(pe::model::model_reserve_type<pe::model::OUTPUT>,output,
                                        table,5.0,pe::model::digital_update_method_t::update_table);
    assert(drive.need_to_operate_analog_node==nullptr);
    assert(get_attribute_define(pe::model::model_reserve_type<pe::model::OUTPUT>,output,0).digital==state::Z);
}
int main()
{
    input_loaded();
    observer_thresholds();
    gate_drive(pe::model::NOT{},1); gate_drive(pe::model::TFF{},2);
    gate_drive(pe::model::T_BAR_FF{},2); gate_drive(pe::model::JKFF{},3);
    std::puts("digital levels: finite attributes, legacy state, loaded INPUT, NOT/T/T-bar/JK drives and OUTPUT analog thresholds passed");
}
