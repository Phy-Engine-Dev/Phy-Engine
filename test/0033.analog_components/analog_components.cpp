#include <phy_engine/phy_engine.h>
#include <phy_engine/model/models/controller/clamped_op_amp.h>
#include <phy_engine/model/models/controller/analog_schmitt.h>
#include <phy_engine/model/models/linear/voltage_meter.h>
#include <phy_engine/model/models/generator/triangle.h>
#include <phy_engine/model/models/non-linear/BJT_NPN.h>
#include <phy_engine/model/models/non-linear/BJT_PNP.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>

namespace pe=phy_engine;
using pe::netlist::add_model;
using pe::netlist::add_to_node;
using pe::netlist::create_node;
using pe::netlist::get_ground_node;
using pe::model::variant_type;

static void close(double actual,double expected,double tolerance=1e-8)
{
    if(!std::isfinite(actual) || std::abs(actual-expected)>tolerance*(1+std::abs(expected)))
    { std::fprintf(stderr,"actual %.17g expected %.17g\n",actual,expected);std::abort(); }
}
template<class Model> auto add(pe::circult& c,Model model,std::initializer_list<pe::model::node_t*> nodes)
{
    auto [p,pos]=add_model(c.nl,model);
    std::size_t pin=0;
    for(auto n:nodes) { assert(add_to_node(c.nl,*p,pin++,*n)); }
    return p;
}
static double voltage(pe::model::node_t const& node) { return node.node_information.an.voltage.real(); }
static void solve(pe::circult& c) { c.prepare();assert(c.solve()); }

static void opamp_tests()
{
    for(double input:{-2.0,-0.1,0.0,0.1,2.0})
    {
        pe::circult c{};c.at=pe::analyze_type::OP;
        auto& g=get_ground_node(c.nl);auto& in=create_node(c.nl);auto& out=create_node(c.nl);
        add(c,pe::model::VDC{.V=input},{&in,&g});
        add(c,pe::model::clamped_op_amp{.mu=100,.Vmin=-15,.Vmax=15},{&in,&g,&out,&g});
        add(c,pe::model::resistance{.r=1000},{&out,&g});
        solve(c);close(voltage(out),std::clamp(100*input,-15.0,15.0));
    }
    // Finite-gain closed-loop follower, independent exact linear oracle.
    {
        pe::circult c{};c.at=pe::analyze_type::OP;
        auto& g=get_ground_node(c.nl);auto& in=create_node(c.nl);auto& out=create_node(c.nl);
        add(c,pe::model::VDC{.V=2},{&in,&g});
        add(c,pe::model::clamped_op_amp{.mu=1e5},{&in,&out,&out,&g});
        add(c,pe::model::resistance{.r=1000},{&out,&g});
        solve(c);close(voltage(out),2e5/100001.0);
    }
    // Incompatible ideal voltage drivers are not reported as success.
    {
        pe::circult c{};c.at=pe::analyze_type::OP;
        auto& g=get_ground_node(c.nl);auto& in=create_node(c.nl);auto& out=create_node(c.nl);
        add(c,pe::model::VDC{.V=1},{&in,&g});
        add(c,pe::model::clamped_op_amp{.mu=100},{&in,&g,&out,&g});
        add(c,pe::model::VDC{.V=0},{&out,&g});
        c.prepare();assert(!c.solve());
    }
}
static void meter_tests()
{
    pe::circult c{};c.at=pe::analyze_type::OP;
    auto& g=get_ground_node(c.nl);auto& vin=create_node(c.nl);auto& out=create_node(c.nl);
    add(c,pe::model::VDC{.V=10},{&vin,&g});
    add(c,pe::model::resistance{.r=1e6},{&vin,&out});
    auto meter=add(c,pe::model::voltage_meter{.Rinput=1e9},{&out,&g});
    solve(c);
    close(voltage(out),10*1e9/(1e9+1e6));
    close(meter->ptr->get_attribute(16).d,10/(1e9+1e6),1e-14);
    close(meter->ptr->get_attribute(17).d,-10/(1e9+1e6),1e-14);
    assert(!meter->ptr->set_attribute(0,{.d{0},.type{variant_type::d}}));
    assert(!meter->ptr->set_attribute(0,{.d{std::numeric_limits<double>::infinity()},.type{variant_type::d}}));
    // AC phasor signs belong to the finite conductance, not an invented meter
    // source; the bias-independent real and imaginary currents are observable.
    {
        pe::circult ac{};ac.at=pe::analyze_type::AC;
        auto& zero=get_ground_node(ac.nl);auto& a=create_node(ac.nl);
        add(ac,pe::model::VAC{.m_Vp=2,.m_omega=1000,.m_phase=std::numbers::pi/2},{&a,&zero});
        auto m=add(ac,pe::model::voltage_meter{.Rinput=1000},{&a,&zero});
        ac.analyzer_setting.ac.omega=1000;assert(ac.analyze());
        close(m->ptr->get_attribute(16).d,0,1e-14);
        close(m->ptr->get_attribute(18).d,.002,1e-14);
        close(m->ptr->get_attribute(19).d,-.002,1e-14);
    }
}
static void triangle_tests()
{
    pe::model::triangle_gen triangle{.Vh=20,.Vl=0,.freq=1000,.phase=0,.duty=.25,.series_resistance=500};
    close(pe::model::triangle_voltage(triangle,0),0);
    close(pe::model::triangle_voltage(triangle,.000125),10);
    close(pe::model::triangle_voltage(triangle,.00025),20);
    close(pe::model::triangle_voltage(triangle,.000625),10);
    close(pe::model::triangle_voltage(triangle,-.000375),10);
    pe::circult c{};c.at=pe::analyze_type::TR;
    auto& g=get_ground_node(c.nl);auto& out=create_node(c.nl);
    add(c,triangle,{&out,&g});add(c,pe::model::resistance{.r=500},{&out,&g});
    c.analyzer_setting.tr.t_step=.000125;c.analyzer_setting.tr.t_stop=.001;
    c.prepare();
    for(int i=1;i<=8;++i)
    {
        c.tr_duration=i*.000125;c.update_tr_step(.000125);assert(c.solve());
        close(voltage(out),.5*pe::model::triangle_voltage(triangle,c.tr_duration));
    }
}
static void schmitt_tests()
{
    // DC polarity and equal-level degeneracy do not depend on a digital clock.
    for(bool inverted:{false,true}) for(double input:{0.0,5.0})
    {
        pe::circult c{};c.at=pe::analyze_type::OP;
        auto& g=get_ground_node(c.nl);auto& in=create_node(c.nl);auto& out=create_node(c.nl);
        add(c,pe::model::VDC{.V=input},{&in,&g});
        add(c,pe::model::analog_schmitt{.Vth_low=1,.Vth_high=3,.inverted=inverted,.Ll=-2,.Hl=7},{&in,&out,&g});
        add(c,pe::model::resistance{.r=100},{&out,&g});solve(c);
        close(voltage(out),((input>3)!=inverted)?7:-2);
    }
    {
        pe::circult c{};c.at=pe::analyze_type::TR;
        auto& g=get_ground_node(c.nl);auto& in=create_node(c.nl);auto& out=create_node(c.nl);
        auto source=add(c,pe::model::VDC{.V=0},{&in,&g});
        add(c,pe::model::analog_schmitt{.Vth_low=1,.Vth_high=3,.Ll=0,.Hl=5,.slew_v_per_s=1000},{&in,&out,&g});
        add(c,pe::model::resistance{.r=1000},{&out,&g});
        c.analyzer_setting.tr.t_step=.001;c.prepare();
        double inputs[]{0,4,2,2,2,2,0,2};
        double outputs[]{0,1,2,3,4,5,4,3};
        for(std::size_t i=0;i<8;++i)
        {
            assert(source->ptr->set_attribute(0,{.d{inputs[i]},.type{variant_type::d}}));
            c.update_tr_step(.001);c.tr_duration=(i+1)*.001;assert(c.solve());
            close(voltage(out),outputs[i]);
        }
    }
    {
        pe::circult c{};c.at=pe::analyze_type::OP;
        auto& g=get_ground_node(c.nl);auto& out=create_node(c.nl);
        add(c,pe::model::analog_schmitt{.inverted=true,.Ll=-.01,.Hl=-.01,.slew_v_per_s=.5},{&g,&out,&g});
        add(c,pe::model::resistance{.r=100},{&out,&g});solve(c);close(voltage(out),-.01);
    }
}
static void nonlinear_feedback_tests()
{
    // Generic logarithmic feedback, independently checked against terminal KCL
    // and the unchanged Ebers-Moll constitutive law. This is not a community
    // circuit fixture, a saved solution, or a prescribed application design.
    for(double vin:{.001,.01,.1,1.0,10.0}) for(bool pnp:{false,true})
    {
        pe::circult c{};c.at=pe::analyze_type::OP;
        auto& g=get_ground_node(c.nl);auto& input=create_node(c.nl);auto& sum=create_node(c.nl);auto& out=create_node(c.nl);
        double sign=pnp?-1:1;
        add(c,pe::model::VDC{.V=sign*vin},{&input,&g});
        add(c,pe::model::resistance{.r=1000},{&input,&sum});
        add(c,pe::model::clamped_op_amp{.mu=1e7},{&g,&sum,&out,&g});
        pe::model::model_base* model{};
        if(pnp) { pe::model::BJT_PNP q{};q.Is=1e-14;model=add(c,q,{&g,&sum,&out}); }
        else { pe::model::BJT_NPN q{};q.Is=1e-14;model=add(c,q,{&g,&sum,&out}); }
        solve(c);
        close(voltage(out),-1e7*voltage(sum));
        double collector_i=model->ptr->get_attribute(17).d;
        close(collector_i,(voltage(input)-voltage(sum))/1000,1e-10);
        double thermal=1.380650524e-23*(27+273.15)/1.6021765314e-19;
        double forward=1e-12*std::expm1(-sign*voltage(out)/thermal);
        double reverse=1e-12*std::expm1(-sign*voltage(sum)/thermal);
        close(collector_i,sign*(forward-2*reverse),1e-10);
    }
    // Returning to cutoff between excitation pulses must not reintroduce the
    // all-zero-Jacobian startup failure on a later transient step.
    {
        pe::circult c{};c.at=pe::analyze_type::TR;
        auto& g=get_ground_node(c.nl);auto& input=create_node(c.nl);auto& sum=create_node(c.nl);auto& out=create_node(c.nl);
        auto source=add(c,pe::model::VDC{.V=0},{&input,&g});
        add(c,pe::model::resistance{.r=1000},{&input,&sum});
        add(c,pe::model::clamped_op_amp{.mu=1e7},{&g,&sum,&out,&g});
        pe::model::BJT_NPN q{};q.Is=1e-14;auto model=add(c,q,{&g,&sum,&out});
        c.analyzer_setting.tr.t_step=1e-6;c.prepare();
        int step=0;
        for(int cycle=0;cycle<4;++cycle) for(double value:{0.0,.1,1.0,.1,0.0})
        {
            assert(source->ptr->set_attribute(0,{.d{value},.type{variant_type::d}}));
            c.update_tr_step(1e-6);c.tr_duration=++step*1e-6;assert(c.solve());
            close(model->ptr->get_attribute(17).d,(voltage(input)-voltage(sum))/1000,1e-10);
        }
    }
}
int main() { opamp_tests();meter_tests();triangle_tests();schmitt_tests();nonlinear_feedback_tests();std::puts("analog components: all independent native checks passed"); }
