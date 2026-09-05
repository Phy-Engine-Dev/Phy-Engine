#include <phy_engine/model/models/non-linear/PN_junction.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace pe = phy_engine;

static void near(double actual, double expected, double relative = 2e-7)
{
    assert(std::isfinite(actual));
    assert(std::abs(actual - expected) <= relative * std::max(1e-30, std::abs(expected)));
}

int main()
{
    // Test the derivative of the actual numerical continuation, including its
    // upper tangent and lower constant floor, not exp(x) outside its domain.
    for(double x : {-70.0, -20.0, 0.0, 20.0, 49.0, 51.0, 70.0})
    {
        double const h = 1e-5;
        double const numerical = (pe::model::pn_details::limexp(x + h) - pe::model::pn_details::limexp(x - h)) / (2 * h);
        near(pe::model::pn_details::limexp_derivative(x), numerical);
    }

    pe::model::PN_junction pn{};
    pn.N = 2;
    pn.Nr = 2;
    pn.Ut = 1.380650524e-23 * 300.15 / 1.6021765314e-19;
    double const reference_voltage = 2.1024259;
    double const reference_current = .01;
    double const junction_voltage = .95 * reference_voltage;
    pn.Is_eff = reference_current / std::expm1(junction_voltage / (pn.N * pn.Ut));
    pn.Isr_eff = 0;
    pn.Bv_set = false;
    near(pe::model::pn_details::conduction(pn, junction_voltage).current, reference_current, 1e-12);

    for(double v : {-.2, .1, 1.0, junction_voltage, 3.0})
    {
        double const h = 1e-6;
        auto actual = pe::model::pn_details::conduction(pn, v);
        double const numerical = (pe::model::pn_details::conduction(pn, v + h).current
                                - pe::model::pn_details::conduction(pn, v - h).current) / (2 * h);
        near(actual.conductance, numerical);
    }

    pe::model::node_t anode{}, cathode{};
    pn.pins[0].nodes = &anode;
    pn.pins[1].nodes = &cathode;
    anode.node_information.an.voltage = junction_voltage;
    cathode.node_information.an.voltage = 0;
    pn.geq = 0;
    pn.Ieq = 0;
    pn.Ud_last = 0; // A stale limited voltage must NOT serve as the oracle.
    assert(!pe::model::check_convergence_define(pe::model::model_reserve_type<pe::model::PN_junction>, pn));
    auto actual = pe::model::pn_details::conduction(pn, junction_voltage);
    pn.geq = actual.conductance;
    pn.Ieq = actual.current - pn.geq * junction_voltage;
    assert(pe::model::check_convergence_define(pe::model::model_reserve_type<pe::model::PN_junction>, pn));
    pn.Ieq += 1e-5;
    assert(!pe::model::check_convergence_define(pe::model::model_reserve_type<pe::model::PN_junction>, pn));
    anode.node_information.an.voltage = std::numeric_limits<double>::infinity();
    assert(!pe::model::check_convergence_define(pe::model::model_reserve_type<pe::model::PN_junction>, pn));

    // Existing avalanche mode must retain the same signed constitutive current
    // with a consistent derivative even though PL import does not enable it.
    pn.Bv_set = true;
    pn.Bv_eff = 4;
    for(double v : {-4.1, -5.5})
    {
        double const h = 1e-6;
        auto point = pe::model::pn_details::conduction(pn, v);
        double const numerical = (pe::model::pn_details::conduction(pn, v + h).current
                                - pe::model::pn_details::conduction(pn, v - h).current) / (2 * h);
        assert(point.current < 0);
        near(point.conductance, numerical);
    }
    std::puts("PN derivative, reference point, unlimited residual and avalanche checks passed");
}
