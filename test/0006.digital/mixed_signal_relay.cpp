#include <phy_engine/phy_engine.h>
#include <phy_engine/circuits/mixed_signal.h>
#include <phy_engine/model/models/controller/relay_current_spdt.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

namespace pe=phy_engine;
template<class M> auto add(pe::circult& c,M model,std::initializer_list<pe::model::node_t*> nodes)
{
    auto [p,pos]=pe::netlist::add_model(c.nl,std::move(model));std::size_t i=0;
    for(auto n:nodes) assert(pe::netlist::add_to_node(c.nl,*p,i++,*n));
    return p;
}
static void close(double a,double b,double tol=1e-9) { assert(std::isfinite(a)&&std::abs(a-b)<tol); }
struct fixture
{
    pe::circult c{};
    pe::model::model_base* input{};
    pe::model::model_base* relay{};
    pe::model::node_t* control{};
    pe::model::node_t* no{};
    explicit fixture(bool invert)
    {
        c.at=pe::analyze_type::TR;
        auto& g=pe::netlist::get_ground_node(c.nl);
        auto& in=pe::netlist::create_node(c.nl);auto& out=pe::netlist::create_node(c.nl);
        auto& com=pe::netlist::create_node(c.nl);auto& nc=pe::netlist::create_node(c.nl);no=&pe::netlist::create_node(c.nl);
        control=invert?&out:&in;
        input=add(c,pe::model::INPUT{.Ll=0,.Hl=.6,.outputA=invert?pe::model::digital_node_statement_t::L:pe::model::digital_node_statement_t::H},{&in});
        if(invert) add(c,pe::model::NOT{.Ll=0,.Hl=.6,.Tsu=0,.Th=0},{&in,&out});
        add(c,pe::model::VDC{.V=5},{&com,&g});
        relay=add(c,pe::model::relay_current_spdt{},{&nc,&com,no,control,&g});
        add(c,pe::model::resistance{.r=100},{&nc,&g});add(c,pe::model::resistance{.r=100},{no,&g});
        c.prepare();
    }
    void state(bool high)
    { assert(input->ptr->set_attribute(0,{.digital{high?pe::model::digital_node_statement_t::H:pe::model::digital_node_statement_t::L},.type{pe::model::variant_type::digital}})); }
    double current() { return relay->ptr->generate_branch_view().branches[0].current.real(); }
    void tick(double dt) { c.update_tr_step(dt); c.tr_duration+=dt;assert(pe::solve_mixed_signal(c)); }
};
int main()
{
    for(bool invert:{false,true})
    {
        fixture f(invert);
        constexpr double dt=1e-4;
        for(int n=1;n<=500;++n)
        {
            f.tick(dt);close(f.control->node_information.an.voltage.real(),.6);
            close(f.current(),.03*(1-std::pow(1/1.01,n)),1e-10);
        }
        assert(f.relay->ptr->get_attribute(9).d==1);
        close(f.no->node_information.an.voltage.real(),5*100/100.01);
        double initial=f.current();f.state(invert);
        for(int n=1;n<=300;++n)
        { f.tick(dt);close(f.current(),initial*std::pow(1/1.01,n),1e-10); }
        assert(f.relay->ptr->get_attribute(9).d==0);
        close(f.no->node_information.an.voltage.real(),5*100/(1e12+100));
        // Static DC must settle logic before the analog coil is solved.
        f.state(!invert);f.c.at=pe::analyze_type::DC;assert(pe::solve_mixed_signal(f.c));
        close(f.current(),.03);assert(f.relay->ptr->get_attribute(9).d==1);
    }
    // Conflicting voltage drivers are an explicit failure, never last-wins.
    {
        pe::circult c{};c.at=pe::analyze_type::DC;
        auto& g=pe::netlist::get_ground_node(c.nl);auto& n=pe::netlist::create_node(c.nl);
        add(c,pe::model::INPUT{.Hl=3,.outputA=pe::model::digital_node_statement_t::H},{&n});
        add(c,pe::model::INPUT{.Hl=5,.outputA=pe::model::digital_node_statement_t::H},{&n});
        add(c,pe::model::resistance{.r=100},{&n,&g});assert(!pe::solve_mixed_signal(c));
    }
    // Analog feedback inverter with no delay has no DC fixed point.
    {
        pe::circult c{};c.at=pe::analyze_type::DC;
        auto& g=pe::netlist::get_ground_node(c.nl);auto& n=pe::netlist::create_node(c.nl);
        add(c,pe::model::NOT{.Ll=0,.Hl=3,.Tsu=0,.Th=0},{&n,&n});
        add(c,pe::model::resistance{.r=100},{&n,&g});assert(!pe::solve_mixed_signal(c));
    }
    std::puts("mixed signal: INPUT/NOT relay DC, 1600 true RL steps, contact KCL, conflicts and oscillation checks passed");
}
