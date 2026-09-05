#include <phy_engine/phy_engine.h>
#include <phy_engine/circuits/mixed_signal.h>
#include <phy_engine/model/models/controller/clamped_op_amp.h>
#include <phy_engine/model/models/controller/fuse_latched.h>
#include <phy_engine/model/models/non-linear/BJT_NPN.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

namespace pe=phy_engine;
template<class M> auto add(pe::circult& c,M model,std::initializer_list<pe::model::node_t*> pins)
{
    auto [p,pos]=pe::netlist::add_model(c.nl,std::move(model));std::size_t index=0;
    for(auto n:pins) assert(pe::netlist::add_to_node(c.nl,*p,index++,*n));
    return p;
}
static void nonlinear_order(bool fuse_first,double gain,double trip,bool expected_blown)
{
    pe::circult c{};c.at=pe::analyze_type::OP;
    auto& g=pe::netlist::get_ground_node(c.nl);auto& input=pe::netlist::create_node(c.nl);
    auto& sum=pe::netlist::create_node(c.nl);auto& out=pe::netlist::create_node(c.nl);auto& load=pe::netlist::create_node(c.nl);
    pe::model::model_base* fuse{};
    auto add_fuse=[&] {fuse=add(c,pe::model::fuse_latched{.rated_current=.005,.trip_current=trip},{&out,&load});};
    if(fuse_first) add_fuse();
    add(c,pe::model::VDC{.V=1},{&input,&g});
    add(c,pe::model::resistance{.r=1000},{&input,&sum});
    add(c,pe::model::clamped_op_amp{.mu=gain},{&g,&sum,&out,&g});
    pe::model::BJT_NPN transistor{};transistor.Is=1e-22;
    add(c,std::move(transistor),{&g,&sum,&out});
    add(c,pe::model::resistance{.r=100},{&load,&g});
    if(!fuse_first) add_fuse();
    c.prepare();assert(c.solve());
    double const actual_output{out.node_information.an.voltage.real()};
    assert(std::abs(actual_output+1.01245645)<1e-7);
    assert((fuse->ptr->get_attribute(7).d!=0)==expected_blown);
    if(!expected_blown)
    {
        double const current{fuse->ptr->generate_branch_view().branches[0].current.real()};
        assert(std::abs(current)<trip);
        assert(std::abs(current-actual_output/100.01)<1e-12);
        assert(fuse->ptr->get_attribute(9).d==0);
    }
}
static void mixed_fixed_point_precedes_trip()
{
    // The initial analog input is still 0 before VDC is solved, so NOT's first
    // trial drive is 3 V. The actual settled input is 3 V and output is 0 V.
    // An intermediate analog-only solve must never irreversibly blow the fuse.
    pe::circult c{};c.at=pe::analyze_type::DC;
    auto& g=pe::netlist::get_ground_node(c.nl);auto& in=pe::netlist::create_node(c.nl);
    auto& out=pe::netlist::create_node(c.nl);auto& load=pe::netlist::create_node(c.nl);
    add(c,pe::model::VDC{.V=3},{&in,&g});
    add(c,pe::model::NOT{.Ll=0,.Hl=3,.Tsu=0,.Th=0},{&in,&out});
    auto fuse=add(c,pe::model::fuse_latched{.rated_current=.01,.trip_current=.02},{&out,&load});
    add(c,pe::model::resistance{.r=100},{&load,&g});
    assert(pe::solve_mixed_signal(c));
    assert(std::abs(out.node_information.an.voltage.real())<1e-12);
    assert(fuse->ptr->get_attribute(7).d==0);
    assert(fuse->ptr->get_attribute(9).d==0);
}
int main()
{
    // Constitutive checks are side-effect free even at a large overload.
    pe::model::fuse_latched f{};f.branches[0].current=2;
    assert(check_convergence_define(pe::model::model_reserve_type<pe::model::fuse_latched>,f));assert(!f.blown);
    for(double gain:{1e7,1e9}) for(bool first:{false,true})
    {
        nonlinear_order(first,gain,.010123553,false);
        nonlinear_order(first,gain,.01,true);
    }
    mixed_fixed_point_precedes_trip();
    std::puts("fuse: true nonlinear model-order invariance and full mixed fixed-point commit passed");
}
