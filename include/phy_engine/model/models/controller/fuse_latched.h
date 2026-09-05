#pragma once
#include "../../model_refs/base.h"
#include <cmath>
#include <fast_io/fast_io_dsal/string_view.h>

namespace phy_engine::model
{
    // Generic instantaneous overcurrent fuse, not a thermal I^2*t model or
    // a claim of equivalence to a closed-source PhysicsLab solver. The real
    // solved branch current can trip only after ALL circuit constitutive
    // residuals pass. A trip forces another solve with finite Roff and latches.
    // AC is small-signal about the preceding DC operating point: AC phasors
    // do not trip a fuse. Use transient analysis for time-varying overloads.
    struct fuse_latched
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"Latched Overcurrent Fuse"};
        inline static constexpr model_device_type device_type{model_device_type::non_linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"FUSE"};

        double rated_current{0.3};
        double trip_current{0.5};
        double Ron{0.01};
        double Roff{1e12};
        bool enabled{true};
        bool initial_blown{};
        bool blown{};
        bool check_trip{};
        double last_trip_current{};
        pin pins[2]{{{u8"P"}}, {{u8"N"}}};
        // Positive current flows P -> N, including finite open-state leakage.
        branch branches[1]{};

        [[nodiscard]] bool valid() const noexcept
        {
            return ::std::isfinite(rated_current) && rated_current > 0.0 && ::std::isfinite(trip_current) && trip_current > 0.0 && ::std::isfinite(Ron) &&
                   Ron > 0.0 && ::std::isfinite(Roff) && Roff > Ron;
        }
    };

    static_assert(model<fuse_latched>);

    inline bool set_attribute_define(model_reserve_type_t<fuse_latched>, fuse_latched& f, ::std::size_t index, variant value) noexcept
    {
        if(value.type != variant_type::d || !::std::isfinite(value.d)) { return false; }
        switch(index)
        {
            case 0:
                if(value.d <= 0.0) { return false; }
                f.rated_current = value.d;
                return true;
            case 1:
                if(value.d <= 0.0) { return false; }
                f.trip_current = value.d;
                return true;
            case 2:
                if(value.d <= 0.0) { return false; }
                f.Ron = value.d;
                return true;
            case 3:
                if(value.d <= 0.0) { return false; }
                f.Roff = value.d;
                return true;
            case 4:
                if(value.d != 0.0 && value.d != 1.0) { return false; }
                f.enabled = value.d == 1.0;
                return true;
            case 5:
                if(value.d != 0.0 && value.d != 1.0) { return false; }
                f.initial_blown = value.d == 1.0;
                return true;
            case 6:
                // An explicit reset represents replacing/resetting the fuse,
                // not an automatic time-dependent healing process. An extant
                // overload will immediately trip it again on the next solve.
                if(value.d != 1.0) { return false; }
                f.blown = false;
                f.last_trip_current = 0.0;
                return true;
            default: return false;
        }
    }

    inline variant get_attribute_define(model_reserve_type_t<fuse_latched>, fuse_latched const& f, ::std::size_t index) noexcept
    {
        switch(index)
        {
            case 0: return {.d{f.rated_current}, .type{variant_type::d}};
            case 1: return {.d{f.trip_current}, .type{variant_type::d}};
            case 2: return {.d{f.Ron}, .type{variant_type::d}};
            case 3: return {.d{f.Roff}, .type{variant_type::d}};
            case 4: return {.d{f.enabled ? 1.0 : 0.0}, .type{variant_type::d}};
            case 5: return {.d{f.initial_blown ? 1.0 : 0.0}, .type{variant_type::d}};
            case 7: return {.d{f.blown ? 1.0 : 0.0}, .type{variant_type::d}};
            case 8: return {.d{f.branches[0].current.real()}, .type{variant_type::d}};
            case 9: return {.d{f.last_trip_current}, .type{variant_type::d}};
            default: return {};
        }
    }

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(model_reserve_type_t<fuse_latched>, ::std::size_t index) noexcept
    {
        switch(index)
        {
            case 0: return u8"RatedCurrent";
            case 1: return u8"TripCurrent";
            case 2: return u8"Ron";
            case 3: return u8"Roff";
            case 4: return u8"Enabled";
            case 5: return u8"InitialBlown";
            case 6: return u8"Reset";
            case 7: return u8"Blown";
            case 8: return u8"BranchCurrent";
            case 9: return u8"LastTripCurrent";
            default: return {};
        }
    }

    inline bool init_define(model_reserve_type_t<fuse_latched>, fuse_latched& f) noexcept
    {
        if(!f.valid()) { return false; }
        f.blown = f.initial_blown;
        f.last_trip_current = 0.0;
        return true;
    }

    inline bool prepare_foundation_define(model_reserve_type_t<fuse_latched>, fuse_latched& f) noexcept { return f.valid(); }

    inline bool fuse_latched_stamp(fuse_latched const& f, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const p{f.pins[0].nodes};
        auto const n{f.pins[1].nodes};
        if(p == nullptr || n == nullptr) { return false; }
        auto const k{f.branches[0].index};
        // Add rather than assign: P and N may intentionally share a node.
        mna.B_ref(p->node_index, k) += 1.0;
        mna.B_ref(n->node_index, k) -= 1.0;
        mna.C_ref(k, p->node_index) += 1.0;
        mna.C_ref(k, n->node_index) -= 1.0;
        mna.D_ref(k, k) -= f.enabled && !f.blown ? f.Ron : f.Roff;
        return true;
    }

    inline bool iterate_dc_define(model_reserve_type_t<fuse_latched>, fuse_latched& f, ::phy_engine::MNA::MNA& mna) noexcept
    {
        if(!f.valid()) { return false; }
        f.check_trip = true;
        return fuse_latched_stamp(f, mna);
    }

    inline bool iterate_tr_define(model_reserve_type_t<fuse_latched>, fuse_latched& f, ::phy_engine::MNA::MNA& mna, double time) noexcept
    {
        if(!::std::isfinite(time) || time < 0.0) { return false; }
        return iterate_dc_define(model_reserve_type<fuse_latched>, f, mna);
    }

    inline bool iterate_ac_define(model_reserve_type_t<fuse_latched>, fuse_latched& f, ::phy_engine::MNA::MNA& mna, double omega) noexcept
    {
        if(!f.valid() || !::std::isfinite(omega) || omega < 0.0) { return false; }
        f.check_trip = false;
        return fuse_latched_stamp(f, mna);
    }

    inline bool check_convergence_define(model_reserve_type_t<fuse_latched>, fuse_latched const& f) noexcept
    {
        auto const current{f.branches[0].current};
        return ::std::isfinite(current.real()) && ::std::isfinite(current.imag());
    }

    inline bool commit_converged_state_define(model_reserve_type_t<fuse_latched>, fuse_latched& f) noexcept
    {
        auto const current{f.branches[0].current};
        if(!check_convergence_define(model_reserve_type<fuse_latched>, f)) { return false; }
        if(f.check_trip && f.enabled && !f.blown && ::std::abs(current.real()) >= f.trip_current)
        {
            f.last_trip_current = current.real();
            f.blown = true;
            // Never return the pre-trip overloaded circuit as a solution.
            return false;
        }
        return true;
    }

    inline constexpr pin_view generate_pin_view_define(model_reserve_type_t<fuse_latched>, fuse_latched& f) noexcept { return {f.pins, 2}; }

    inline constexpr branch_view generate_branch_view_define(model_reserve_type_t<fuse_latched>, fuse_latched& f) noexcept { return {f.branches, 1}; }
}  // namespace phy_engine::model
