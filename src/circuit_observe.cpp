#include <phy_engine/phy_engine.h>
#include <phy_engine/circuits/mixed_signal.h>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>

// Bounded observation ABI. Unlike the legacy real-only sampler, this preserves
// AC phase and four-state digital values and never reads analog union storage
// for an exclusively digital node.
extern "C" int circuit_sample_complex(void* circuit_ptr, std::size_t* vec_pos,
    std::size_t* chunk_pos, std::size_t comp_size, std::size_t capacity,
    double* vr, double* vi, std::size_t* vo, double* ir, double* ii,
    std::size_t* io, std::uint8_t* digital)
{
    if (!circuit_ptr || !vec_pos || !chunk_pos || !vr || !vi || !vo || !ir || !ii || !io || !digital) return 1;
    auto* circuit = static_cast<phy_engine::circult*>(circuit_ptr);
    auto& netlist = circuit->get_netlist();
    vo[0] = io[0] = 0;
    for (std::size_t c = 0; c < comp_size; ++c)
    {
        auto* model = phy_engine::netlist::get_model(netlist, phy_engine::netlist::model_pos{vec_pos[c], chunk_pos[c]});
        if (!model || !model->ptr) return 2;
        auto pins = model->ptr->generate_pin_view();
        auto branches = model->ptr->generate_branch_view();
        if (pins.size > capacity - vo[c] || branches.size > capacity - io[c]) return 3;
        for (std::size_t p = 0; p < pins.size; ++p)
        {
            auto* node = pins.pins[p].nodes;
            auto n = vo[c] + p;
            vr[n] = vi[n] = 0;
            digital[n] = 2;
            if (!node) continue;
            if (node->num_of_analog_node != 0)
            {
                vr[n] = node->node_information.an.voltage.real();
                vi[n] = node->node_information.an.voltage.imag();
                if(!std::isfinite(vr[n]) || !std::isfinite(vi[n])) return 4;
            }
            else
            {
                switch (node->node_information.dn.state)
                {
                    case phy_engine::model::digital_node_statement_t::L: digital[n] = 0; break;
                    case phy_engine::model::digital_node_statement_t::H: digital[n] = 1; break;
                    case phy_engine::model::digital_node_statement_t::Z: digital[n] = 3; break;
                    default: digital[n] = 2;
                }
            }
        }
        for (std::size_t b = 0; b < branches.size; ++b)
        {
            ir[io[c] + b] = branches.branches[b].current.real();
            ii[io[c] + b] = branches.branches[b].current.imag();
            if(!std::isfinite(ir[io[c] + b]) || !std::isfinite(ii[io[c] + b])) return 4;
        }
        vo[c + 1] = vo[c] + pins.size;
        io[c + 1] = io[c] + branches.size;
    }
    return 0;
}

// Read-only typed model attributes (including behavior-state flags). The
// caller chooses catalog-declared indices; no guessed memory layout is read.
extern "C" int circuit_get_model_scalar(void* circuit_ptr, std::size_t vec_pos,
    std::size_t chunk_pos, std::size_t attribute, double* output)
{
    if(!circuit_ptr || !output) return 1;
    auto* circuit=static_cast<phy_engine::circult*>(circuit_ptr);
    auto* model=phy_engine::netlist::get_model(circuit->get_netlist(),phy_engine::netlist::model_pos{vec_pos,chunk_pos});
    if(!model || !model->ptr) return 2;
    auto value=model->ptr->get_attribute(attribute);
    if(value.type==phy_engine::model::variant_type::d) *output=value.d;
    else if(value.type==phy_engine::model::variant_type::boolean) *output=value.boolean?1:0;
    else return 3;
    return std::isfinite(*output)?0:4;
}

// A bounded exact-endpoint TR driver using the same native prepare/stamp/
// nonlinear solve path. Unlike the legacy accumulated floating-point while
// loop, 0.5s / 50us is exactly 10000 solves and ends at precisely base+0.5s.
using circuit_trace_callback = int (*)(void*, double, std::size_t);

static bool circuit_solution_finite(phy_engine::circult const& circuit)
{
    for(auto node:circuit.size_t_to_node_p)
    {
        auto v=node->node_information.an.voltage;
        if(!std::isfinite(v.real())||!std::isfinite(v.imag())) return false;
    }
    for(auto branch:circuit.size_t_to_branch_p)
        if(!std::isfinite(branch->current.real())||!std::isfinite(branch->current.imag())) return false;
    return true;
}

// Explicit static mixed-signal entry point; AC phasor/event co-simulation is
// intentionally not inferred. OP/DC use ideal configured Ll/Hl voltage drive.
extern "C" int circuit_run_mixed_dc(void* circuit_ptr, std::uint32_t analyze_type)
{
    if(!circuit_ptr) return 1;
    if(analyze_type>1) return 2;
    auto& circuit=*static_cast<phy_engine::circult*>(circuit_ptr);
    circuit.set_analyze_type(static_cast<phy_engine::analyze_type>(analyze_type));
    circuit.prepare();
    return phy_engine::solve_mixed_signal(circuit) && circuit_solution_finite(circuit)?0:3;
}

extern "C" int circuit_run_transient_trace(void* circuit_ptr, double step,
    double stop, std::size_t max_steps, std::size_t sample_every,
    circuit_trace_callback callback, void* user, double* actual_stop,
    std::size_t* actual_steps, std::size_t* actual_samples)
{
    if(!circuit_ptr || !actual_stop || !actual_steps || !actual_samples) return 1;
    if(!std::isfinite(step)||!std::isfinite(stop)||step<=0||stop<=0||step>stop||max_steps==0||max_steps>100000) return 2;
    double ratio=stop/step,nearest=std::round(ratio);
    if(!std::isfinite(ratio)||ratio>static_cast<double>(max_steps)+1e-8) return 2;
    if(std::abs(ratio-nearest)<=32*std::numeric_limits<double>::epsilon()*std::max(1.0,ratio)) ratio=nearest;
    auto count=static_cast<std::size_t>(std::ceil(ratio));
    if(count==0||count>max_steps) return 2;
    // Sample only completed native solves. No invented t=0 operating point.
    // The final point is always included, without duplicating a regular sample.
    if(callback && (sample_every==0 || (count/sample_every+(count%sample_every!=0))>201)) return 2;
    auto* circuit=static_cast<phy_engine::circult*>(circuit_ptr);
    double base=circuit->tr_duration;
    if(!std::isfinite(base)||!std::isfinite(base+stop)||base+stop<=base) return 2;
    *actual_stop=base;*actual_steps=0;*actual_samples=0;
    circuit->set_analyze_type(phy_engine::analyze_type::TR);
    circuit->analyzer_setting.tr.t_step=step;
    circuit->analyzer_setting.tr.t_stop=stop;
    circuit->prepare();
    for(std::size_t i=0;i<count;++i)
    {
        double target=base+(i+1==count?stop:std::min(stop,static_cast<double>(i+1)*step));
        double prior=circuit->tr_duration;
        circuit->update_tr_step(target-prior);
        circuit->tr_duration=target;
        if(!phy_engine::solve_mixed_signal(*circuit)||!circuit_solution_finite(*circuit)){circuit->tr_duration=prior;return 3;}
        *actual_stop=target;*actual_steps=i+1;
        if(callback && ((i+1)%sample_every==0 || i+1==count))
        {
            ++*actual_samples;
            if(callback(user,circuit->tr_duration,i+1)!=0) return 4;
        }
    }
    return 0;
}

extern "C" int circuit_run_transient_bounded(void* circuit_ptr, double step,
    double stop, std::size_t max_steps, double* actual_stop, std::size_t* actual_steps)
{
    std::size_t samples{};
    return circuit_run_transient_trace(circuit_ptr,step,stop,max_steps,0,nullptr,
        nullptr,actual_stop,actual_steps,&samples);
}
