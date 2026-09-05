#include <cmath>
#include <cstdio>
#include <limits>
#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/controller/fuse_latched.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    using namespace phy_engine;
    constexpr double eps{1e-9};

    struct fixture
    {
        circult c{};
        netlist::model_pos source_pos{}, fuse_pos{};
        model::node_t* load{};
        double resistance{10.0};

        explicit fixture(model::fuse_latched device, double voltage)
        {
            c.set_analyze_type(analyze_type::DC);
            auto& nl{c.get_netlist()};
            auto [v, vp]{add_model(nl, model::VDC{.V = voltage})};
            source_pos = vp;
            auto [f, fp]{add_model(nl, ::std::move(device))};
            fuse_pos = fp;
            auto [r, rp]{add_model(nl, model::resistance{.r = resistance})};
            auto& supply{create_node(nl)};
            auto& output{create_node(nl)};
            load = &output;
            auto& gnd{nl.ground_node};
            add_to_node(nl, *v, 0, supply);
            add_to_node(nl, *v, 1, gnd);
            add_to_node(nl, *f, 0, supply);
            add_to_node(nl, *f, 1, output);
            add_to_node(nl, *r, 0, output);
            add_to_node(nl, *r, 1, gnd);
        }

        auto& fuse() { return *get_model(c.get_netlist(), fuse_pos)->ptr; }

        bool blown() { return fuse().get_attribute(7).d == 1.0; }

        double current() { return fuse().generate_branch_view().branches[0].current.real(); }

        bool voltage(double v) { return get_model(c.get_netlist(), source_pos)->ptr->set_attribute(0, {.d{v}, .type{model::variant_type::d}}); }

        bool reset() { return fuse().set_attribute(6, {.d{1.0}, .type{model::variant_type::d}}); }

        bool kcl(double v, bool open)
        {
            double const r{fuse().get_attribute(open ? 3 : 2).d};
            double const expected{v / (resistance + r)};
            double const out{load->node_information.an.voltage.real()};
            double const source_i{get_model(c.get_netlist(), source_pos)->ptr->generate_branch_view().branches[0].current.real()};
            double const tol{::std::max(1e-18, ::std::abs(expected) * 1e-9)};
            // Node voltage is the difference of order-one source voltages;
            // tolerate floating-point cancellation, still far below Roff's
            // nonzero leakage. The measured branch must meet the tight bound.
            double const kcl_tol{::std::max(tol, 8 * ::std::numeric_limits<double>::epsilon() * ::std::abs(v) / resistance)};
            return ::std::abs(current() - expected) < tol && ::std::abs(current() - out / resistance) < kcl_tol && ::std::abs(source_i + current()) < kcl_tol;
        }

        bool tick(double dt)
        {
            c.update_tr_step(dt);
            c.tr_duration += dt;
            return c.solve();
        }
    };

    bool dc_trip_latch_reset()
    {
        // Above the nominal rating but below the distinct trip threshold:
        // the rating must not silently replace the source's trip current.
        fixture f{{}, 4.0};
        if(!f.c.analyze() || f.blown() || !f.kcl(4.0, false)) { return false; }
        if(!f.voltage(6.0) || !f.c.analyze() || !f.blown() || !f.kcl(6.0, true)) { return false; }
        if(::std::abs(f.fuse().get_attribute(9).d - 6.0 / 10.01) > eps) { return false; }
        // Removal of the fault must never heal a fuse by itself.
        if(!f.voltage(2.0) || !f.c.analyze() || !f.blown() || !f.kcl(2.0, true)) { return false; }
        if(!f.reset() || !f.c.analyze() || f.blown() || !f.kcl(2.0, false)) { return false; }
        // Absolute current, including reverse overload, trips the same latch.
        if(!f.voltage(-6.0) || !f.c.analyze() || !f.blown() || !f.kcl(-6.0, true)) { return false; }
        if(!f.reset() || !f.c.analyze() || !f.blown() || !f.kcl(-6.0, true)) { return false; }
        return f.fuse().get_attribute(9).d < -0.5;
    }

    bool actual_transient_pre_post_and_persistence()
    {
        fixture f{{}, 2.0};
        f.c.set_analyze_type(analyze_type::TR);
        f.c.prepare();
        constexpr double dt{.001};
        for(unsigned i{1}; i <= 100; ++i)
        {
            double const v{i < 20 ? 2.0 : i < 30 ? 6.0 : 1.0};
            if(!f.voltage(v) || !f.tick(dt)) { return false; }
            if(f.blown() != (i >= 20) || !f.kcl(v, i >= 20)) { return false; }
        }
        // A new prepare does not clear a physical state already initialized.
        f.c.prepare();
        return f.tick(dt) && f.blown() && f.kcl(1.0, true);
    }

    bool initially_blown_and_manual_switch_do_not_heal()
    {
        model::fuse_latched device{};
        device.initial_blown = true;
        fixture f{device, 3.0};
        if(!f.c.analyze() || !f.blown() || !f.kcl(3.0, true)) { return false; }
        if(!f.fuse().set_attribute(4, {.d{0.0}, .type{model::variant_type::d}}) || !f.c.analyze()) { return false; }
        if(!f.fuse().set_attribute(4, {.d{1.0}, .type{model::variant_type::d}}) || !f.c.analyze() || !f.blown()) { return false; }
        if(!f.reset() || !f.c.analyze() || f.blown() || !f.kcl(3.0, false)) { return false; }
        if(!f.fuse().set_attribute(4, {.d{0.0}, .type{model::variant_type::d}}) || !f.c.analyze()) { return false; }
        return !f.blown() && f.kcl(3.0, true);
    }

    bool ac_is_small_signal_not_instantaneous_trip()
    {
        circult c{};
        c.set_analyze_type(analyze_type::AC);
        c.get_analyze_setting().ac.omega = 100.0;
        auto& nl{c.get_netlist()};
        auto [v, vp]{add_model(nl, model::VAC{.m_Vp = 100.0})};
        auto [f, fp]{add_model(nl, model::fuse_latched{})};
        auto [r, rp]{add_model(nl, model::resistance{.r = 10.0})};
        auto& supply{create_node(nl)};
        auto& load{create_node(nl)};
        auto& gnd{nl.ground_node};
        add_to_node(nl, *v, 0, supply);
        add_to_node(nl, *v, 1, gnd);
        add_to_node(nl, *f, 0, supply);
        add_to_node(nl, *f, 1, load);
        add_to_node(nl, *r, 0, load);
        add_to_node(nl, *r, 1, gnd);
        if(!c.analyze() || f->ptr->get_attribute(7).d != 0.0) { return false; }
        return ::std::abs(f->ptr->generate_branch_view().branches[0].current - 100.0 / 10.01) < eps;
    }

    bool validation_and_shorted_pins()
    {
        model::fuse_latched f{};
        auto const tag{model::model_reserve_type<model::fuse_latched>};
        if(set_attribute_define(tag, f, 0, {.d{0.0}, .type{model::variant_type::d}}) ||
           set_attribute_define(tag, f, 4, {.d{.5}, .type{model::variant_type::d}}) ||
           set_attribute_define(tag, f, 6, {.d{0.0}, .type{model::variant_type::d}}) ||
           set_attribute_define(tag, f, 7, {.d{0.0}, .type{model::variant_type::d}}) ||
           set_attribute_define(tag, f, 1, {.d{::std::numeric_limits<double>::infinity()}, .type{model::variant_type::d}}))
        {
            return false;
        }
        f.Roff = f.Ron;
        if(prepare_foundation_define(tag, f)) { return false; }
        circult c{};
        c.set_analyze_type(analyze_type::DC);
        auto& nl{c.get_netlist()};
        auto [m, p]{add_model(nl, model::fuse_latched{})};
        add_to_node(nl, *m, 0, nl.ground_node);
        add_to_node(nl, *m, 1, nl.ground_node);
        return c.analyze() && ::std::abs(m->ptr->generate_branch_view().branches[0].current) < eps && m->ptr->get_attribute(7).d == 0.0;
    }
}  // namespace

int main()
{
    if(!dc_trip_latch_reset())
    {
        ::std::puts("FAIL DC trip/latch/reset/reverse/KCL");
        return 1;
    }
    if(!actual_transient_pre_post_and_persistence())
    {
        ::std::puts("FAIL transient fuse/KCL/persistence");
        return 2;
    }
    if(!initially_blown_and_manual_switch_do_not_heal())
    {
        ::std::puts("FAIL initial/manual state");
        return 3;
    }
    if(!ac_is_small_signal_not_instantaneous_trip())
    {
        ::std::puts("FAIL AC scope");
        return 4;
    }
    if(!validation_and_shorted_pins())
    {
        ::std::puts("FAIL validation/shorted pins");
        return 5;
    }
    ::std::puts("PASS Fuse: real DC/TR branch, overload latch, reset/retrip, " "KCL, initial/manual state, held AC, invalid parameters");
}
