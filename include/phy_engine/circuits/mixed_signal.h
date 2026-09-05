#pragma once
#include "circuit.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace phy_engine
{
    inline bool has_digital_models(circult const& c) noexcept
    {
        for(auto const& block:c.nl.models) for(auto m=block.begin;m!=block.curr;++m)
            if(m->type==model::model_type::normal && m->ptr && m->ptr->get_device_type()==model::model_device_type::digital) return true;
        return false;
    }

    // A zero-time digital/analog fixed point. The caller updates reactive
    // history once per actual time step, never once per settling iteration.
    // Returns false for conflicting ideal drivers or non-settling feedback.
    // This interface models ideal voltage drive, not finite digital Imax/Rout.
    inline bool solve_mixed_signal(circult& c)
    {
        if(!has_digital_models(c)) return c.solve();
        using drive=digital::need_operate_analog_node_t;
        std::vector<drive> previous;
        bool have_solution=false;
        // Ensure model lists/node indices exist, without resetting already
        // initialized model state or a prior step's L/C companion history.
        c.prepare();
        for(unsigned iteration=0;iteration!=64;++iteration)
        {
            // prepare() clears the event queue. Re-seed original circuit nodes
            // so a retained input still reaches all connected logic gates.
            for(auto& block:c.nl.nodes) for(auto node=block.begin;node!=block.curr;++node)
                if(!node->pins.empty() && node->num_of_analog_node!=node->pins.size()) c.digital_update_tables.tables.insert(node);
            c.digital_clk();
            if(!c.digital_update_tables.tables.empty()) return false;
            std::vector<drive> current(c.digital_out.begin(),c.digital_out.end());
            std::sort(current.begin(),current.end(),[](drive a,drive b)
            { return std::less<model::node_t*>{}(a.need_to_operate_analog_node,b.need_to_operate_analog_node); });
            std::vector<drive> unique;
            for(auto d:current)
            {
                if(!d.need_to_operate_analog_node || !std::isfinite(d.voltage)) return false;
                if(!unique.empty() && unique.back().need_to_operate_analog_node==d.need_to_operate_analog_node)
                {
                    // Repeated visits to one output are normal. Opposing
                    // drivers on the same wire are not silently overwritten.
                    if(unique.back().voltage!=d.voltage) return false;
                }
                else unique.push_back(d);
            }
            c.digital_out.clear();
            for(auto d:unique) c.digital_out.push_back(d);
            bool unchanged=have_solution && unique.size()==previous.size();
            if(unchanged) for(std::size_t i=0;i<unique.size();++i)
                if(unique[i].voltage!=previous[i].voltage || unique[i].need_to_operate_analog_node!=previous[i].need_to_operate_analog_node)
                { unchanged=false;break; }
            // An irreversible event (e.g. fuse) must also wait for the digital
            // drive fixed point, not merely one analog sub-solve.
            if(unchanged && c.commit_converged_states()) return true;
            previous=std::move(unique);
            // The number of active ideal drives may change (e.g. tri-state).
            // Re-index branches before stamping to avoid sharing an inductor
            // or relay branch index with a digital voltage source.
            c.prepare();
            if(!c.solve(false)) return false;
            have_solution=true;
        }
        return false;
    }
}
