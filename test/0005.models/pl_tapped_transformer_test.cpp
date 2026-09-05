#include <cmath>
#include <cstdio>
#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/linear/transformer_center_tap.h>
#include <phy_engine/netlist/impl.h>

int main()
{
    // Declared engineering convention: PL rated secondary voltage is the full
    // end-to-end winding, with an equal tap. This tests PE, not closed-app parity.
    phy_engine::circult c{};
    c.set_analyze_type(phy_engine::analyze_type::DC);
    auto& nl{c.get_netlist()};
    auto [v, vp]{add_model(nl, phy_engine::model::VDC{.V = 4.0})};
    auto [tx, tp]{add_model(nl, phy_engine::model::transformer_center_tap{.n_total = 2.0})};
    auto [r1, rp1]{add_model(nl, phy_engine::model::resistance{.r = 100.0})};
    auto [r2, rp2]{add_model(nl, phy_engine::model::resistance{.r = 250.0})};
    auto& primary{create_node(nl)};
    auto& upper{create_node(nl)};
    auto& lower{create_node(nl)};
    auto& gnd{nl.ground_node};
    add_to_node(nl, *v, 0, primary);
    add_to_node(nl, *v, 1, gnd);
    add_to_node(nl, *tx, 0, primary);  // PL0
    add_to_node(nl, *tx, 1, gnd);      // PL1
    add_to_node(nl, *tx, 2, upper);    // PL2
    add_to_node(nl, *tx, 3, gnd);      // PL4 (center tap, NOT PL3)
    add_to_node(nl, *tx, 4, lower);    // PL3
    add_to_node(nl, *r1, 0, upper);
    add_to_node(nl, *r1, 1, gnd);
    add_to_node(nl, *r2, 0, lower);
    add_to_node(nl, *r2, 1, gnd);
    if(!c.analyze()) { return 1; }
    double const u{upper.node_information.an.voltage.real()};
    double const l{lower.node_information.an.voltage.real()};
    if(::std::abs(u - 1.0) > 1e-9 || ::std::abs(l + 1.0) > 1e-9 || ::std::abs(u - l - 2.0) > 1e-9) { return 2; }
    auto const currents{tx->ptr->generate_branch_view()};
    if(currents.size != 3) { return 3; }
    double const ip{currents.branches[0].current.real()};
    double const i1{currents.branches[1].current.real()};
    double const i2{currents.branches[2].current.real()};
    if(::std::abs(i1 + u / 100.0) > 1e-9 || ::std::abs(i2 - l / 250.0) > 1e-9) { return 4; }
    if(::std::abs(4.0 * ip - (u * u / 100.0 + l * l / 250.0)) > 1e-9) { return 5; }
    ::std::puts("PASS: total 2V, equal +/-1V halves, unequal-load KCL and power conservation");
}
