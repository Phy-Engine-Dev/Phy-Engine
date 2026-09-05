#include <phy_engine/model/models/non-linear/full_bridge_rectifier.h>
#include <cassert>
#include <cstdio>

int main()
{
    using namespace phy_engine::model;
    node_t nodes[4]{};
    // A deliberately inconsistent stamp inside a wrapper must not bypass the
    // underlying PN junction's residual check.
    full_bridge_rectifier bridge{};
    for(unsigned i=0;i<4;++i) bridge.pins[i].nodes=&nodes[i];
    nodes[0].node_information.an.voltage=1;
    nodes[1].node_information.an.voltage=0;
    nodes[2].node_information.an.voltage=.5;
    nodes[3].node_information.an.voltage=-.5;
    assert(prepare_foundation_define(model_reserve_type<full_bridge_rectifier>,bridge));
    assert(!check_convergence_define(model_reserve_type<full_bridge_rectifier>,bridge));
    for(auto* diode:{&bridge.D1,&bridge.D2,&bridge.D3,&bridge.D4})
    {
        auto v=diode->pins[0].nodes->node_information.an.voltage.real()-diode->pins[1].nodes->node_information.an.voltage.real();
        auto actual=pn_details::conduction(*diode,v);
        diode->geq=actual.conductance;diode->Ieq=actual.current-v*actual.conductance;
    }
    assert(check_convergence_define(model_reserve_type<full_bridge_rectifier>,bridge));
    bridge.D4.Ieq+=.001;
    assert(!check_convergence_define(model_reserve_type<full_bridge_rectifier>,bridge));
    std::puts("bridge: all four internal PN residual checks forwarded");
}
