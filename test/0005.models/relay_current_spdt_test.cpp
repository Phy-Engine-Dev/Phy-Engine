#include <cmath>
#include <cstdio>
#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/controller/relay_current_spdt.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    using namespace phy_engine;
    constexpr double eps{1e-8};

    struct fixture
    {
        circult c{};
        netlist::model_pos source_pos{};
        netlist::model_pos contact_source_pos{};
        netlist::model_pos relay_pos{};
        model::node_t* nc{};
        model::node_t* no{};

        explicit fixture(model::relay_current_spdt device, double voltage)
        {
            c.set_analyze_type(analyze_type::DC);
            auto& nl{c.get_netlist()};
            auto [source, sp]{add_model(nl, model::VDC{.V = voltage})};
            source_pos = sp;
            auto [supply, vp]{add_model(nl, model::VDC{.V = 5.0})};
            contact_source_pos = vp;
            auto [relay, rp]{add_model(nl, ::std::move(device))};
            relay_pos = rp;
            auto [rn, rnp]{add_model(nl, model::resistance{.r = 100.0})};
            auto [ro, rop]{add_model(nl, model::resistance{.r = 100.0})};
            auto& ctrl{create_node(nl)};
            auto& common{create_node(nl)};
            auto& n{create_node(nl)};
            auto& o{create_node(nl)};
            nc = &n;
            no = &o;
            auto& ground{nl.ground_node};
            add_to_node(nl, *source, 0, ctrl);
            add_to_node(nl, *source, 1, ground);
            add_to_node(nl, *supply, 0, common);
            add_to_node(nl, *supply, 1, ground);
            add_to_node(nl, *relay, 0, n);
            add_to_node(nl, *relay, 1, common);
            add_to_node(nl, *relay, 2, o);
            add_to_node(nl, *relay, 3, ctrl);
            add_to_node(nl, *relay, 4, ground);
            add_to_node(nl, *rn, 0, n);
            add_to_node(nl, *rn, 1, ground);
            add_to_node(nl, *ro, 0, o);
            add_to_node(nl, *ro, 1, ground);
        }

        auto& relay() { return *get_model(c.get_netlist(), relay_pos)->ptr; }

        bool state() { return relay().get_attribute(9).d != 0.0; }

        double current() { return relay().generate_branch_view().branches[0].current.real(); }

        bool voltage(double v) { return get_model(c.get_netlist(), source_pos)->ptr->set_attribute(0, {.d{v}, .type{model::variant_type::d}}); }

        bool tick(double dt)
        {
            c.update_tr_step(dt);
            c.tr_duration += dt;
            return c.solve();
        }

        bool contacts_consistent(bool expected)
        {
            double const closed_v{5.0 * 100.0 / 100.01};
            double const open_v{5.0 * 100.0 / (1e12 + 100.0)};
            double const vn{nc->node_information.an.voltage.real()};
            double const vo{no->node_information.an.voltage.real()};
            if(::std::abs(vn - (expected ? open_v : closed_v)) > eps || ::std::abs(vo - (expected ? closed_v : open_v)) > eps) { return false; }
            auto const currents{relay().generate_branch_view()};
            if(::std::abs(currents.branches[1].current.real() - vn / 100.0) > eps || ::std::abs(currents.branches[2].current.real() - vo / 100.0) > eps)
            {
                return false;
            }
            double const supply{get_model(c.get_netlist(), contact_source_pos)->ptr->generate_branch_view().branches[0].current.real()};
            return ::std::abs(supply + (vn + vo) / 100.0) < eps;
        }
    };

    bool dc_pickup_release_and_kcl()
    {
        fixture f{{}, 0.0};
        if(!f.c.analyze() || f.state() || !f.contacts_consistent(false)) { return false; }
        if(!f.voltage(.6) || !f.c.analyze() || !f.state() || !f.contacts_consistent(true)) { return false; }
        if(::std::abs(f.current() - .03) > eps) { return false; }
        // Between thresholds, hold the preceding state rather than chatter.
        if(!f.voltage(.36) || !f.c.analyze() || !f.state()) { return false; }
        if(!f.voltage(.2) || !f.c.analyze() || f.state() || !f.contacts_consistent(false)) { return false; }
        // Non-polarized magnetic pull depends on |I|, not invented diode action.
        if(!f.voltage(-.6) || !f.c.analyze() || !f.state()) { return false; }
        return ::std::abs(f.current() + .03) < eps;
    }

    bool actual_rl_transient_and_release()
    {
        fixture f{{}, .6};
        f.c.set_analyze_type(analyze_type::TR);
        f.c.prepare();
        constexpr double dt{1e-4};
        unsigned pickup_step{};
        for(unsigned i{1}; i <= 500; ++i)
        {
            if(!f.tick(dt)) { return false; }
            double const exact{.03 * (1.0 - ::std::exp(-static_cast<double>(i) * dt / .01))};
            if(::std::abs(f.current() - exact) > 6e-5) { return false; }
            if(f.state() && pickup_step == 0) { pickup_step = i; }
        }
        // Analytic pickup at -tau*ln(1-Ipull/Iinf)=10.986 ms;
        // backward Euler and discrete threshold sampling add < two steps.
        if(pickup_step < 109 || pickup_step > 112 || !f.contacts_consistent(true)) { return false; }
        double const initial{f.current()};
        if(!f.voltage(0.0)) { return false; }
        unsigned release_step{};
        for(unsigned i{1}; i <= 200; ++i)
        {
            if(!f.tick(dt)) { return false; }
            double const exact{initial * ::std::exp(-static_cast<double>(i) * dt / .01)};
            if(::std::abs(f.current() - exact) > 6e-5) { return false; }
            if(!f.state() && release_step == 0) { release_step = i; }
        }
        return release_step >= 61 && release_step <= 64 && f.contacts_consistent(false);
    }

    bool mechanical_delays_use_time_not_newton_iterations()
    {
        model::relay_current_spdt r{};
        r.L = 0.0;
        r.operate_delay = .003;
        r.release_delay = .002;
        fixture f{r, .6};
        f.c.set_analyze_type(analyze_type::TR);
        f.c.prepare();
        constexpr double dt{.001};
        for(unsigned i{1}; i <= 6; ++i)
        {
            if(!f.tick(dt)) { return false; }
            if(i < 4 && f.state()) { return false; }
        }
        if(!f.state() || !f.contacts_consistent(true) || !f.voltage(0.0)) { return false; }
        if(!f.tick(dt) || !f.state()) { return false; }
        if(!f.tick(dt) || !f.state()) { return false; }
        if(!f.tick(dt) || f.state()) { return false; }
        return f.contacts_consistent(false);
    }

    bool ac_coil_impedance_and_fixed_contact_state()
    {
        circult c{};
        c.set_analyze_type(analyze_type::AC);
        c.get_analyze_setting().ac.omega = 100.0;
        auto& nl{c.get_netlist()};
        auto [v, vp]{add_model(nl, model::VAC{.m_Vp = 1.0})};
        auto [bias, bp]{add_model(nl, model::VDC{.V = .6})};
        model::relay_current_spdt device{};
        device.initial_engaged = true;
        auto [relay, rp]{add_model(nl, ::std::move(device))};
        auto& ctrl{create_node(nl)};
        auto& biased{create_node(nl)};
        auto& ground{nl.ground_node};
        add_to_node(nl, *v, 0, ctrl);
        add_to_node(nl, *v, 1, biased);
        add_to_node(nl, *bias, 0, biased);
        add_to_node(nl, *bias, 1, ground);
        // All contact pins intentionally shorted to one node. Finite branch
        // impedances must not stamp a bogus nonzero node coefficient here.
        for(unsigned pin{}; pin < 3; ++pin) { add_to_node(nl, *relay, pin, ground); }
        add_to_node(nl, *relay, 3, ctrl);
        add_to_node(nl, *relay, 4, ground);
        if(!c.analyze())
        {
            ::std::puts("AC analyze failed");
            return false;
        }
        auto const branches{relay->ptr->generate_branch_view()};
        auto const expected{
            1.0 / ::std::complex<double>{20.0, 100.0 * .2}
        };
        if(::std::abs(branches.branches[0].current - expected) > eps)
        {
            ::std::printf("AC got %.12g %.12g expected %.12g %.12g\n",
                          branches.branches[0].current.real(),
                          branches.branches[0].current.imag(),
                          expected.real(),
                          expected.imag());
            return false;
        }
        if(::std::abs(branches.branches[1].current) > eps || ::std::abs(branches.branches[2].current) > eps) { return false; }
        if(relay->ptr->get_attribute(9).d != 1.0)
        {
            ::std::printf("AC held state %.12g\n", relay->ptr->get_attribute(9).d);
            return false;
        }
        return true;
    }

    bool invalid_parameters_are_explicitly_rejected()
    {
        model::relay_current_spdt r{};
        auto const tag{model::model_reserve_type<model::relay_current_spdt>};
        if(set_attribute_define(tag, r, 1, {.d{0.0}, .type{model::variant_type::d}})) { return false; }
        if(set_attribute_define(tag, r, 8, {.d{.5}, .type{model::variant_type::d}})) { return false; }
        if(set_attribute_define(tag, r, 9, {.d{1.0}, .type{model::variant_type::d}})) { return false; }
        if(!set_attribute_define(tag, r, 3, {.d{1.0}, .type{model::variant_type::d}})) { return false; }
        // Cross-field constraints are enforced before prepare/solve, not by
        // assuming the C ABI assigns properties in a particular order.
        return !prepare_foundation_define(tag, r);
    }
}  // namespace

int main()
{
    if(!dc_pickup_release_and_kcl())
    {
        ::std::puts("FAIL dc pickup/release/contact KCL");
        return 1;
    }
    if(!actual_rl_transient_and_release())
    {
        ::std::puts("FAIL native RL transient");
        return 2;
    }
    if(!mechanical_delays_use_time_not_newton_iterations())
    {
        ::std::puts("FAIL mechanical delay");
        return 3;
    }
    if(!ac_coil_impedance_and_fixed_contact_state())
    {
        ::std::puts("FAIL AC coil impedance / held contacts");
        return 4;
    }
    if(!invalid_parameters_are_explicitly_rejected())
    {
        ::std::puts("FAIL parameter validation");
        return 5;
    }
    ::std::puts("PASS: DC coil loading + pickup/release/contact KCL, native RL transient, explicit delays, AC impedance, parameter validation");
}
