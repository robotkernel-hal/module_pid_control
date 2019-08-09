//! robotkernel module current_control
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

#include "current_control.h"
#include "robotkernel/exceptions.h"
#include "robotkernel/helpers.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

MODULE_DEF(current_control, module_current_control::current_control)

#define min(a, b) ((a) < (b) ? (a) : (b))
using namespace robotkernel;
using namespace std;
using namespace module_current_control;
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

inline double val_to_double(uint8_t *base, const struct current_control::controller::pd_item& item) {
    switch (item.type) {
        case PD_DT_FLOAT: {
            float tmp = *(float *)(&base[item.offset]);
            return (double)tmp * item.scale;
        }
        case PD_DT_DOUBLE: {
            return *(double *)(&base[item.offset]) * item.scale;
        }
        case PD_DT_UINT8: {
            uint8_t tmp = *(uint8_t *)(&base[item.offset]);
            return (double)tmp * item.scale;
        }
        case PD_DT_UINT16: {
            uint16_t tmp = *(uint16_t *)(&base[item.offset]);
            return (double)tmp * item.scale;
        }
        case PD_DT_UINT32: {
            uint32_t tmp = *(uint32_t *)(&base[item.offset]);
            return (double)tmp * item.scale;
        }
        case PD_DT_INT8: {
            int8_t tmp = *(int8_t *)(&base[item.offset]);
            return (double)tmp * item.scale;
        }
        case PD_DT_INT16: {
            int16_t tmp = *(int16_t *)(&base[item.offset]);
            return (double)tmp * item.scale;
        }
        case PD_DT_INT32: {
            int32_t tmp = *(int32_t *)(&base[item.offset]);
            return (double)tmp * item.scale;
        }
    }

    return 0.;
}

current_control::controller::controller(std::shared_ptr<current_control> parent, const YAML::Node& node) :
    pd_provider(format_string("%s.%s", parent->name.c_str(), get_as<string>(node, "name").c_str())),
    pd_consumer(format_string("%s.%s", parent->name.c_str(), get_as<string>(node, "name").c_str())),
    service_provider::process_data_inspection::base(parent->name, get_as<string>(node, "name")),
    parent(parent)
{
    filter_freq = get_as<double>(node, "filter_freq");
    filter_t_const = (1.0 / (2.0 * M_PI * filter_freq));
                
    // controller gains
    gain_pos_proportional = get_as<double>(node, "gain_pos_proportional", 0.);
    gain_pos_derivative   = get_as<double>(node, "gain_pos_derivative",   0.);
    gain_tor_proportional = get_as<double>(node, "gain_tor_proportional", 0.);
    gain_tor_derivative   = get_as<double>(node, "gain_tor_derivative",   0.);

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
        if (pd_dt_map.find((base).type_str) == pd_dt_map.end())                             \
            throw str_exception("unsupported data type: %s\n", (base).type_str.c_str());    \
                                                                                            \
        (base).type     = pd_dt_map[(base).type_str];                                       \
    }
    
    get_pd_item(node["measure_inputs"]["position"], measure_inputs.position);
    get_pd_item(node["measure_inputs"]["torque"],   measure_inputs.torque);
    get_pd_item(node["command_outputs"]["current"], command_outputs.current);
}
    
void find_pd_offset_and_type(current_control::controller::pd_item_t& item, sp_process_data_t pd) {
    if (item.offset == -1) {
        // need to find offset and type
        if (pd->process_data_definition == "")
            throw str_exception("process data \"%s\" has no description, "
                    "cannot determine pos offset!\n", pd->id().c_str());

        YAML::Node pdd_node = YAML::Load(pd->process_data_definition);

        bool found = false;
        item.offset = 0;

        for (const auto& kv : pdd_node) {
            string act_dt = kv.first.as<string>();
            string act_name = kv.second.as<string>();

            if (act_name == item.name) {
                found = true;
                item.type_str = act_dt;
                item.type = pd_dt_map[act_dt];
                break;
            }

            if (dt_to_len.find(act_dt) == dt_to_len.end())
                throw str_exception("unsupported data type in pd description: %s\n", act_dt.c_str());

            item.offset += dt_to_len[act_dt];
        }

        if (!found)
            throw str_exception("member \"%s\" not found in measurement process data description:\n%s\n",
                    item.name.c_str(), pd->process_data_definition.c_str());
            
    }
}

//! creating process data input and trigger
void current_control::controller::start() {
    kernel& k = *kernel::get_instance();

    measure_inputs.pd = k.get_process_data(measure_inputs.dev_name);
    measure_inputs.pd_hash = measure_inputs.pd->set_consumer(shared_from_this());

    command_outputs.pd = k.get_process_data(command_outputs.dev_name);
    command_outputs.pd_hash = command_outputs.pd->set_provider(shared_from_this());

    find_pd_offset_and_type(measure_inputs.position, measure_inputs.pd);
    find_pd_offset_and_type(measure_inputs.torque, measure_inputs.pd);
    find_pd_offset_and_type(command_outputs.current, command_outputs.pd);
}

//! destroying process data input and trigger
void current_control::controller::stop() {
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
void current_control::controller::tick() {
    auto msr_buf = measure_inputs.pd->pop(measure_inputs.pd_hash);
    auto ctrl_inputs_buf = pd_ctrl_inputs->pop(pd_ctrl_inputs_hash);

    double q_msr = 0., dq_msr = 0., dq_msr_filt = 0.,
           tau_msr = 0., dtau_msr = 0., dtau_msr_filt = 0.;

    double q_des = 0., dq_des = 0., dq_des_filt = 0., 
           tau_des = 0., dtau_des = 0., dtau_des_filt = 0.;

    // gains 
    double kd = gain_pos_proportional;
    double kp = gain_pos_derivative;
    double ks = gain_tor_proportional;
    double kt = gain_tor_derivative;

    q_msr = val_to_double(msr_buf, measure_inputs.position);
    dq_msr = (q_msr - q_msr_old);
    dq_msr_filt = filter_first_order(ts, dq_msr, filter_t_const, &dq_msr_filt_old) / ts;

    auto tmp = ((pos_inputs_t *)ctrl_inputs_buf);
    q_des = tmp->target_pos;

    if (tmp->mode == 1) {
        kd = tmp->gain_pos_proportional;
        kp = tmp->gain_pos_derivative;
    }

    if (with_torque) {
        tau_msr = val_to_double(msr_buf, measure_inputs.torque);
        dtau_msr = (tau_msr - tau_msr_old) / ts;
        dtau_msr_filt = filter_first_order(ts, dtau_msr, filter_t_const, &dtau_msr_filt_old);
    
        dtau_des = (tau_des - tau_des_old) / ts;
        dtau_des_filt = filter_first_order(ts, dtau_des, filter_t_const, &dtau_des_filt_old);

        auto tmp = ((pos_tor_inputs_t *)ctrl_inputs_buf);
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

    if ((tau_msr > tau_des - 0.1f) && (tau_msr < tau_des + 0.1f)) {
        tau_msr = tau_des;
    }

    double tau_m =  
        kp * (q_des - q_msr) +
        kd * (dq_des_filt - dq_msr_filt) +
        kt * (tau_des - tau_msr) +
        ks * (dtau_des_filt - dtau_msr_filt);

    double des_current = -0.096f * tau_m;
    if (des_current > 5.0)
        des_current = 5.0;
    if (des_current < -5.0)
        des_current = -5.0;
    int16_t des_current_mA = (int16_t)(des_current * 1000.0);

    q_des_old = q_des;
    tau_des_old = tau_des;
}

//! construction
/*!
 * \param node yaml intialization node
 */
current_control::current_control(const std::string& name, const YAML::Node& node) : 
    module_base("module_current_control", name, node)
{
    config = YAML::Clone(node);
}

//! destruction 
current_control::~current_control() {
    set_state(module_state_init);
}

void current_control::init() {
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
int current_control::set_state(module_state_t state) {
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

void current_control::tick() {
    for (auto& d : ctrl_list)
        d->tick();
}

