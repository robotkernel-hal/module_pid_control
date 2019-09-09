//! robotkernel module pid_control
/*!
 * author: Robert Burger
 *
 * $Id$
 */

// vim: tabstop=4 softtabstop=4 shiftwidth=4 expandtab:

/*
 * This file is part of robotkernel.
 *
 * robotkernel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robotkernel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with robotkernel.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

#include <string_util/string_util.h>

#include "pid_control.h"
#include "robotkernel/exceptions.h"
#include "robotkernel/helpers.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

MODULE_DEF(pid_control, module_pid_control::pid_control)

#define min(a, b) ((a) < (b) ? (a) : (b))
using namespace robotkernel;
using namespace std;
using namespace module_pid_control;
using namespace string_util;

std::map<std::string, size_t> dt_to_len = {
    { "float",    4 },
    { "double",   8 },
    { "uint8_t",  1 },
    { "uint16_t", 2 },
    { "uint32_t", 4 },
    { "int8_t",   1 },
    { "int16_t",  2 },
    { "int32_t",  4 },
};

std::map<std::string, pd_data_types> pd_dt_map = {
    { "float",    PD_DT_FLOAT  },
    { "double",   PD_DT_DOUBLE },
    { "uint8_t",  PD_DT_UINT8  },
    { "uint16_t", PD_DT_UINT16 },
    { "uint32_t", PD_DT_UINT32 },
    { "int8_t",   PD_DT_INT8   },
    { "int16_t",  PD_DT_INT16  },
    { "int32_t",  PD_DT_INT32  },
};

inline double val_to_double(uint8_t *base, const struct pid_control::controller::pd_item& item) {
    switch (item.type) {
#define CASE_PD_DT(dt_enum, dtype)                          \
        case dt_enum: {                                     \
            dtype tmp = *(dtype *)(&base[item.offset]);     \
            return (double)tmp * item.scale;                \
        }

        CASE_PD_DT(PD_DT_FLOAT, float)
        CASE_PD_DT(PD_DT_DOUBLE, double)
        CASE_PD_DT(PD_DT_UINT8, uint8_t)
        CASE_PD_DT(PD_DT_UINT16, uint16_t)
        CASE_PD_DT(PD_DT_UINT32, uint32_t)
        CASE_PD_DT(PD_DT_INT8, int8_t)
        CASE_PD_DT(PD_DT_INT16, int16_t)
        CASE_PD_DT(PD_DT_INT32, int32_t)

#undef CASE_PD_DT

    }

    return 0.;
}

inline void double_to_val(uint8_t *base, const struct pid_control::controller::pd_item& item, double in) {
    switch (item.type) {
#define CASE_PD_DT(dt_enum, dtype)                              \
        case dt_enum: {                                         \
            *(dtype *)(&base[item.offset]) = (in / item.scale); \
            break;                                              \
        }

        CASE_PD_DT(PD_DT_FLOAT, float)
        CASE_PD_DT(PD_DT_DOUBLE, double)
        CASE_PD_DT(PD_DT_UINT8, uint8_t)
        CASE_PD_DT(PD_DT_UINT16, uint16_t)
        CASE_PD_DT(PD_DT_UINT32, uint32_t)
        CASE_PD_DT(PD_DT_INT8, int8_t)
        CASE_PD_DT(PD_DT_INT16, int16_t)
        CASE_PD_DT(PD_DT_INT32, int32_t)

#undef CASE_PD_DT
    }
}

inline void convert_str_val(const pd_data_types& type, const std::string& value_str,
        std::vector<uint8_t>& value) {
    switch (type) {
#define CASE_PD_DT(dt_enum, dtype, cvrt)                                    \
        case dt_enum: {                                                     \
            value.resize(sizeof(dtype));                                    \
            *(dtype *)&value[0] = cvrt(value_str.c_str());                  \
            break;                                                          \
        }
        
        CASE_PD_DT(PD_DT_FLOAT, float, atof)
        CASE_PD_DT(PD_DT_DOUBLE, double, atof)
        CASE_PD_DT(PD_DT_UINT8, uint8_t, atoi)
        CASE_PD_DT(PD_DT_UINT16, uint16_t, atoi)
        CASE_PD_DT(PD_DT_UINT32, uint32_t, atoi)
        CASE_PD_DT(PD_DT_INT8, int8_t, atoi)
        CASE_PD_DT(PD_DT_INT16, int16_t, atoi)
        CASE_PD_DT(PD_DT_INT32, int32_t, atoi)
        
#undef CASE_PD_DT
    }
}

inline bool check_state(uint8_t *base, const struct pid_control::controller::pd_item& item) {
    switch (item.type) {
        default: 
            return false;
#define CASE_PD_DT(dt_enum, dtype)                                                                              \
        case dt_enum: {                                                                                         \
            if ((*(dtype *)(base+item.offset) & *(dtype *)&item.mask[0]) == *(dtype *)&item.value[0])           \
                return true;                                                                                    \
            break;                                                                                              \
        }        

        CASE_PD_DT(PD_DT_UINT8, uint8_t)
        CASE_PD_DT(PD_DT_UINT16, uint16_t)
        CASE_PD_DT(PD_DT_UINT32, uint32_t)
        CASE_PD_DT(PD_DT_INT8, int8_t)
        CASE_PD_DT(PD_DT_INT16, int16_t)
        CASE_PD_DT(PD_DT_INT32, int32_t)

#undef CASE_PD_DT
    }

    return false;
}

pid_control::controller::controller(std::shared_ptr<pid_control> parent, const YAML::Node& node) :
    pd_provider(format_string("%s.%s", parent->name.c_str(), get_as<string>(node, "name").c_str())),
    pd_consumer(format_string("%s.%s", parent->name.c_str(), get_as<string>(node, "name").c_str())),
    service_provider::process_data_inspection::base(parent->name, get_as<string>(node, "name")),
    parent(parent)
{
    name = get_as<string>(node, "name");
    ts = 0.001;
    with_torque = false;

    filter_freq           = get_as<double>(node, "filter_freq",           DEFAULT_FILTER_FREQ);
    filter_t_const        = (1.0 / (2.0 * M_PI * filter_freq));
                
    // controller gains
    gain_pos_proportional = get_as<double>(node, "gain_pos_proportional", DEFAULT_GAIN_POS_PROPORTIONAL);
    gain_pos_derivative   = get_as<double>(node, "gain_pos_derivative",   DEFAULT_GAIN_POS_DERIVATIVE);
    gain_tor_proportional = get_as<double>(node, "gain_tor_proportional", DEFAULT_GAIN_TOR_PROPORTIONAL);
    gain_tor_derivative   = get_as<double>(node, "gain_tor_derivative",   DEFAULT_GAIN_TOR_DERIVATIVE);
    gain_tau_to_i         = get_as<double>(node, "gain_tau_to_i",         DEFAULT_GAIN_TAU_TO_I);

    limit_current         = get_as<double>(node, "limit_current",         DEFAULT_LIMIT_CURRENT);

    for (const auto& kv : node["inputs"]) {
        string name = kv.first.as<std::string>();
        const YAML::Node& ci_node = kv.second;
        inputs.insert( { name, input(ci_node) } );
    }
    
    for (const auto& kv : node["outputs"]) {
        string name = kv.first.as<std::string>();
        const YAML::Node& co_node = kv.second;
        outputs.insert( { name, output(co_node) } );
    }
        
    const auto& buf_out = pd_ctrl_outputs->pop(pd_ctrl_outputs_hash);

    for (auto& input : inputs) {
        const auto& buf_in = pds[input.pd].pd->peek();
        double msr = input.get_msr(buf_in);
        double des = input.get_des(buf_out);
        double d_msr = (msr - input.msr_old) / ts;
        double d_msr_filt = filter_first_order(ts, d_msr, filter_t_const, &input.d_msr_filt_old);
        double d_des = (des - input.des_old) / ts;
        double d_des_filt = filter_first_order(ts, d_des, filter_t_const, &input.d_des_filt_old);

        outputs[input.target].act_val += 
            input.kp * (cmd - msr) + 
            input.kd * (d_des_filt - d_msr_filt);

        input.msr_old = msr;
        input.des_old = des;
    }

    if (!node["measure_inputs"]) 
        throw str_exception("missing \"measure_inputs\" section in module config!\n");
    if (!node["command_outputs"]) 
        throw str_exception("missing \"command_outputs\" section in module config!\n");

    measure_inputs.dev_name              = get_as<string>(node["measure_inputs"], "dev_name");
    command_outputs.dev_name             = get_as<string>(node["command_outputs"], "dev_name");

#define get_pd_item(pdnode, base)                                                           \
    if (pdnode) {                                                                           \
        (base).name     = get_as<string>(pdnode, "name", "");                               \
        (base).offset   = get_as<off_t> (pdnode, "offset", -1);                             \
        (base).type_str = get_as<string>(pdnode, "type", "");                               \
        (base).scale    = get_as<double>(pdnode, "scale", 0.);                              \
        (base).value_str= get_as<string>(pdnode, "value", "");                              \
        (base).mask_str = get_as<string>(pdnode, "mask", "");                               \
    }
    
    get_pd_item(node["measure_inputs"]["position"], measure_inputs.position);
    get_pd_item(node["measure_inputs"]["torque"],   measure_inputs.torque);
    get_pd_item(node["command_outputs"]["current"], command_outputs.current);

    if (node["overrides"]) {
        for (const auto& ovr : node["overrides"]) {
            pd_item_t tmp;
            get_pd_item(ovr, tmp);

            parent->log(info, "adding override for field \"%s\" to %s\n", 
                    tmp.name.c_str(), tmp.value_str.c_str());

            overrides.push_back(tmp);
        }
    }
    
    if (node["power_states"]) {
        for (const auto& ovr : node["power_states"]) {
            pd_item_t tmp;
            get_pd_item(ovr, tmp);

            parent->log(info, "adding power_state for field \"%s\" with mask %s and value %s\n", 
                    tmp.name.c_str(), tmp.mask_str.c_str(), tmp.value_str.c_str());

            power_states.push_back(tmp);
        }
    }
}

void find_pd_offset_and_type(pid_control::controller::pd_item_t& item, sp_process_data_t pd) {
    if (item.offset == -1) {
        // need to find offset and type
        if (pd->process_data_definition == "")
            throw str_exception("process data \"%s\" has no description, "
                    "cannot determine pos offset!\n", pd->id().c_str());

        YAML::Node pdd_node = YAML::Load(pd->process_data_definition);

        item.offset = 0;

        for (const auto& list_el : pdd_node) {
            for (const auto& kv : list_el) {
                string act_dt = kv.first.as<string>();
                string act_name = kv.second.as<string>();

                if (act_name == item.name) {
                    item.type_str = act_dt;
                    item.type = pd_dt_map[act_dt];

                    if (item.value_str != "") {
                        convert_str_val(item.type, item.value_str, item.value);
                    }
                    
                    if (item.mask_str != "") {
                        convert_str_val(item.type, item.mask_str, item.mask);
                    }

                    return;
                }

                if (dt_to_len.find(act_dt) == dt_to_len.end())
                    throw str_exception("unsupported data type in pd description: %s\n", act_dt.c_str());

                item.offset += dt_to_len[act_dt];
            }
        }

        throw str_exception("member \"%s\" not found in measurement process data description:\n%s\n",
                item.name.c_str(), pd->process_data_definition.c_str());
    }
}

//! creating process data input and trigger
void pid_control::controller::start() {
    kernel& k = *kernel::get_instance();

    measure_inputs.pd = k.get_process_data(measure_inputs.dev_name);
    command_outputs.pd = k.get_process_data(command_outputs.dev_name);
    command_outputs.pd_hash = command_outputs.pd->set_provider(shared_from_this());

    local_outputs.resize(command_outputs.pd->length);

    find_pd_offset_and_type(measure_inputs.position, measure_inputs.pd);
    find_pd_offset_and_type(measure_inputs.torque, measure_inputs.pd);
    find_pd_offset_and_type(command_outputs.current, command_outputs.pd);

    for (auto& pdi : overrides) {
        find_pd_offset_and_type(pdi, command_outputs.pd);
    }

    for (auto& ps : power_states) {
        find_pd_offset_and_type(ps, command_outputs.pd);
    }

    // create process data 
    size_t cc_outputs_struct_length = with_torque ? sizeof(pos_tor_outputs_t) : sizeof(pos_outputs_t);
    string cc_ctrl_outputs_desc = with_torque ? pos_tor_outputs_desc : pos_outputs_desc;
    string pd_ctrl_outputs_desc = command_outputs.pd->process_data_definition + cc_ctrl_outputs_desc;
    pd_ctrl_outputs = make_shared<triple_buffer>(command_outputs.pd->length + cc_outputs_struct_length, 
            parent->name, format_string("%s.outputs", name.c_str()), pd_ctrl_outputs_desc);
    pd_ctrl_outputs_hash = pd_ctrl_outputs->set_consumer(shared_from_this());
    k.add_device(pd_ctrl_outputs);

    // process data inspection
    k.add_device(shared_from_this());

    if (measure_inputs.pd->clk_device != "") {
        auto clk_dev = k.get_trigger(measure_inputs.pd->clk_device);
        clk_dev->add_trigger(shared_from_this());

        if (clk_dev->get_rate() != 0) {
            ts = 1. / clk_dev->get_rate();
            parent->log(info, "added to measurements trigger %s, got clock interval %10.6f\n", 
                    clk_dev->id().c_str(), ts);
        } else {
            parent->log(info, "added to measurements trigger %s, using pre-defined clock interval %10.6f\n", 
                    clk_dev->id().c_str(), ts);
        }
    }
}

//! destroying process data input and trigger
void pid_control::controller::stop() {
    kernel& k = *kernel::get_instance();

    // process data inspection
    k.remove_device(shared_from_this());
}

//! discrete filter first order
/*!
 * \param time in [s]
 * \param x_n act unfiltered value
 * \param t_const time constant in [s]
 * \param y_n_minus_1 last filtered value
 * \return y_n act filtered value
 */
inline double filter_first_order(double time, double x_n, double t_const, double *y_n_minus_1) {
    double c;
    c =  t_const / (time + t_const);
    *y_n_minus_1 = ((1 - c) * x_n) + (c * *y_n_minus_1);
    return *y_n_minus_1;
}            

//! trigger tick
void pid_control::controller::tick() {
    if (parent->state != module_state_op)
        return;

    auto msr_buf = measure_inputs.pd->peek();
    auto ctrl_outputs_buf = pd_ctrl_outputs->pop(pd_ctrl_outputs_hash);    

    double q_msr = 0., dq_msr = 0., dq_msr_filt = 0.,
           tau_msr = 0., dtau_msr = 0., dtau_msr_filt = 0.;

    double q_des = 0., dq_des = 0., dq_des_filt = 0., 
           tau_des = 0., dtau_des = 0., dtau_des_filt = 0.;

    // gains 
    double kp = gain_pos_proportional;
    double kd = gain_pos_derivative;
    double kt = gain_tor_proportional;
    double ks = gain_tor_derivative;

    // check power states
    for (auto& ps : power_states) {
        if (!check_state(ctrl_outputs_buf, ps))
            return;
    }

    auto tmp = ((pos_outputs_t *)(ctrl_outputs_buf + command_outputs.pd->length));
    q_des = tmp->target_pos;

    if (tmp->mode == 1) {
        kp = tmp->gain_pos_proportional;
        kd = tmp->gain_pos_derivative;
        filter_t_const = 1 / (2 * M_PI * tmp->filter_freq);
    }

    q_msr = val_to_double(msr_buf, measure_inputs.position);
    dq_msr = (q_msr - q_msr_old);
    dq_msr_filt = filter_first_order(ts, dq_msr, filter_t_const, &dq_msr_filt_old) / ts;

    if (with_torque) {
        tau_msr = val_to_double(msr_buf, measure_inputs.torque);
        dtau_msr = (tau_msr - tau_msr_old) / ts;
        dtau_msr_filt = filter_first_order(ts, dtau_msr, filter_t_const, &dtau_msr_filt_old);
    
        dtau_des = (tau_des - tau_des_old) / ts;
        dtau_des_filt = filter_first_order(ts, dtau_des, filter_t_const, &dtau_des_filt_old);

        auto tmp = ((pos_tor_outputs_t *)(ctrl_outputs_buf + command_outputs.pd->length));
        tau_des = tmp->target_tor;

        if (tmp->mode == 1) {
            ks = tmp->gain_tor_proportional;
            kt = tmp->gain_tor_derivative;
        }
    }

    // derive and filter desired position and torque
    dq_des = (q_des - q_des_old) / ts;
    dq_des_filt = filter_first_order(ts, dq_des, filter_t_const, &dq_des_filt_old);

    if (do_reset) {
        dtau_des            = 0.;
        dtau_des_filt       = 0.;
        dtau_des_filt_old   = 0.;
        dq_des              = 0.;
        dq_des_filt         = 0.;
        dq_des_filt_old     = 0.;
        dq_msr_filt         = 0.;
        dtau_msr_filt       = 0.;

        do_reset = false;
    }

    double tau_m =  
        kp * (q_des - q_msr) +
        kd * (dq_des_filt - dq_msr_filt);

//    parent->log(info, "got q des %10.6f, msr %10.6f, tau_m %10.6f\n", q_des, q_msr, tau_m);
    if (with_torque)
        tau_m += 
            kt * (tau_des - tau_msr) +
            ks * (dtau_des_filt - dtau_msr_filt);

    double des_current = gain_tau_to_i * tau_m;
    if (des_current > limit_current) des_current = limit_current;
    if (des_current < -limit_current) des_current = -limit_current;

    // passing values
    memcpy(&local_outputs[0], &ctrl_outputs_buf[0], command_outputs.pd->length);

    // setting overrides
    for (auto& pdi : overrides) {
        uint8_t *adr = &local_outputs[pdi.offset];
        memcpy(adr, &pdi.value[0], pdi.value.size());
    }

    // setting calculated current
    double_to_val(&local_outputs[0], command_outputs.current, des_current);

    command_outputs.pd->write(command_outputs.pd_hash, 0, &local_outputs[0], local_outputs.size()); 

    q_des_old = q_des;
    tau_des_old = tau_des;
}
                
// process data inspection
void pid_control::controller::get_pdin(service_provider::process_data_inspection::pd_t& pd) {
}

void pid_control::controller::get_pdout(service_provider::process_data_inspection::pd_t& pd) {
    const auto& buf = command_outputs.pd->peek();
    pd.resize(command_outputs.pd->length);
    memcpy(&pd[0], buf, pd.size());
//    std::copy(buf.begin(), buf.end(), std::back_inserter(pd));
}

//! construction
/*!
 * \param node yaml intialization node
 */
pid_control::pid_control(const std::string& name, const YAML::Node& node) : 
    module_base("module_pid_control", name, node)
{
    config = YAML::Clone(node);
}

//! destruction 
pid_control::~pid_control() {
    set_state(module_state_init);
}

void pid_control::init() {
    if (config["controllers"]) {
        for (const auto& ctrl_node : config["controllers"]) {
            auto d = std::make_shared<controller>(shared_from_this(), ctrl_node);
            ctrl_list.push_back(d);
        }
    }
}
        
//! set module state machine to defined state
/*!
 * \param state requested state
 * \return success or failure
 */
int pid_control::set_state(module_state_t state) {
    // get transition
    uint32_t transition = GEN_STATE(this->state, state);

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands
            for (auto& d : ctrl_list) {
                d->stop();
            }
            
            if (state == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
        case init_2_init:
            // ====> re-/open ethercat device
            if (state == module_state_init)
                break;
        case init_2_boot:
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open ethercat device
            if (state == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop:
            // ====> initial devices            
            if (state == module_state_preop)
                break;
        case preop_2_op:
        case preop_2_safeop: {
            // ====> start receiving measurements
            if (state == module_state_safeop)
                break;
        }
        case safeop_2_op: {
            // ====> start sending commands
            for (auto& d : ctrl_list) {
                d->start();
            }

            break;
        }
        case op_2_op:
        case safeop_2_safeop:
        case preop_2_preop:
            // ====> do nothing
            break;

        default:
            break;
    }

    return (this->state = state);
}

void pid_control::tick() {
    if (state != module_state_op)
        return;

    for (auto& d : ctrl_list)
        d->tick();
}

