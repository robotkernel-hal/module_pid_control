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
#include <algorithm>
#include <regex>

MODULE_DEF(pid_control, module_pid_control::pid_control)

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

inline double val_to_double(uint8_t *base, const struct pid_control::controller::io_base& item) {
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

inline void double_to_val(uint8_t *base, const struct pid_control::controller::io_base& item, double in) {
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

inline bool check_state(uint8_t *base, const struct pid_control::controller::override_state& item) {
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

//! construction
/*!
 * example configuration
 *
 * name: my_pid_name
 * ts: 0.0002
 * inputs:
 * - { name: position, pd: device.inputs.pd, field_name: act_position, 
 *   kp: 2500., ki: 0., kd: 0.005, filter: 100., i_window: 0.0005, target: target_current }
 * outputs:
 * - { name: target_current, pd: device.outputs.pd, field_name: target_current, kt: 0.5, limit: 5000. }
 * overrides:
 * - { name: mode, field_name: $device.outputs.pd.mode, value: 16 } # current control
 * power_states:
 * - { name: control, field_name: device.outputs.pd.control, value: 1, mask: 1 }    
 * trigger: device.inputs.trigger
 * 
 */
pid_control::controller::controller(std::shared_ptr<pid_control> parent, const YAML::Node& node) :
    pd_provider(format_string("%s.%s", parent->name.c_str(), get_as<string>(node, "name").c_str())),
    pd_consumer(format_string("%s.%s", parent->name.c_str(), get_as<string>(node, "name").c_str())),
    service_provider::process_data_inspection::base(parent->name, get_as<string>(node, "name")),
    parent(parent)
{
    name = get_as<string>(node, "name");
    ts   = get_as<double>(node, "ts", 0.001);

    for (const auto& ci_node : node["inputs"]) {
        string name = get_as<string>(ci_node, "name");
        input_order.push_back(name);
        inputs.insert( { name, input(ci_node) } );
    }
    
    for (const auto& co_node : node["outputs"]) {
        string name = get_as<string>(co_node, "name");
        output_order.push_back(name);
        outputs.insert( { name, output(co_node) } );
    }

    if (node["overrides"]) {
        for (const auto& ovr_node : node["overrides"]) {
            string name = get_as<string>(ovr_node, "name");
            override_state_t ovr(ovr_node);
            overrides.insert( { name, ovr } );

            parent->log(info, "adding override for field \"%s\" to %s\n", 
                    name.c_str(), ovr.value_str.c_str());
        }
    }
    
    if (node["power_states"]) {
        for (const auto& ps_node : node["power_states"]) {
            string name = get_as<string>(ps_node, "name");
            override_state_t ps(ps_node);
            states.insert( { name, ps } );

            parent->log(info, "adding power_state for field \"%s\" with mask %s and value %s\n", 
                    name.c_str(), ps.mask_str.c_str(), ps.value_str.c_str());
        }
    }

    trigger_dev_name = get_as<string>(node, "trigger", "");
}

void find_pd_offset_and_type(pid_control::controller::io_base_t& item, sp_process_data_t pd) {
    if (item.field_name != "") {
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

                if (act_name == item.field_name) {
                    item.type_str = act_dt;
                    item.type = pd_dt_map[act_dt];
    
                    try {
                        auto& os = dynamic_cast<pid_control::controller::override_state&>(item);
                        if (os.value_str != "") {
                            convert_str_val(os.type, os.value_str, os.value);
                        }

                        if (os.mask_str != "") {
                            convert_str_val(os.type, os.mask_str, os.mask);
                        }
                    } catch (std::bad_cast exp) {}
                    
                    return;
                }

                if (dt_to_len.find(act_dt) == dt_to_len.end())
                    throw str_exception("unsupported data type in pd description: %s\n", act_dt.c_str());

                item.offset += dt_to_len[act_dt];
            }
        }

        throw str_exception("member \"%s\" not found in measurement process data description:\n%s\n",
                item.field_name.c_str(), pd->process_data_definition.c_str());
    }
}

template <typename T, typename pd_type>
inline void add_pds(T& item, std::map<std::string, pd_type>& pds) {
    if (pds.find(item.pd) == pds.end()) {
        pds[item.pd].dev_name = item.pd;
        pds[item.pd].pd = kernel::get_instance()->get_process_data(item.pd);
        pds[item.pd].pd_hash = 0;
    }

    find_pd_offset_and_type(item, pds[item.pd].pd);
}

template <typename T, typename pd_type>
inline void add_pds(std::map<std::string, T>& tmp_map, std::map<std::string, pd_type>& pds) {
    for (auto& kv : tmp_map) {
        auto &item = kv.second;
        add_pds(item, pds);
    }
}

template <typename T>
inline bool contains(std::list<T> &tmp_list, const T& element) {
    auto it = std::find(tmp_list.begin(), tmp_list.end(), element);
    return it != tmp_list.end();
}

template <typename key_type, typename value_type>
value_type& get_map_entry(std::map<key_type, value_type>& tmp_map, key_type& tmp_key) {
    auto _it = tmp_map.find(tmp_key);
    if (_it == tmp_map.end())
        throw str_exception_tb("no map entry found!\n");

    value_type& result = (*_it).second;
    return result;
}

//! creating process data input and trigger
void pid_control::controller::start() {
    kernel& k = *kernel::get_instance();

    std::for_each(outputs.begin(), outputs.end(), [&](pair<const string, pid_control::controller::output>& kv) {
            add_pds(kv.second, pds); });
    std::for_each(inputs.begin(), inputs.end(), [&](pair<const string, pid_control::controller::input>& kv) { 
            add_pds(kv.second, pds); });

    for (auto& kv : outputs) {
        auto& output = kv.second;
        auto& output_pd = pds[output.pd];

        output_pd.pd_hash = output_pd.pd->set_provider(shared_from_this());
        output_pd.local_outputs.resize(output_pd.pd->length);
    }

    // create process data 
    std::list<std::string> processed_pd;
    size_t cc_outputs_struct_length = 0;
    YAML::Emitter emitter;
    emitter << YAML::BeginSeq;

    for (auto& name : input_order) {
        auto& input = get_map_entry(inputs, name);
        auto& output = get_map_entry(outputs, input.target);

        if (!contains(processed_pd, output.pd)) {
            auto& output_pd = get_map_entry(pds, output.pd);
            output_pd.pd_outputs_offset = cc_outputs_struct_length;
            cc_outputs_struct_length += output_pd.pd->length;

            string type_prefix = output.pd;

            YAML::Node pd_node = YAML::Load(output_pd.pd->process_data_definition);
            for (const auto& list_node : pd_node) {
                for (const auto& map_node : list_node) {
                    emitter << YAML::BeginMap << map_node.first 
                        << format_string("%s.%s", type_prefix.c_str(), map_node.second.as<string>().c_str()) << YAML::EndMap;
                }
            }

            processed_pd.push_back(output.pd);
        }

        input.pd_ctrl_outputs_offset = cc_outputs_struct_length;
        cc_outputs_struct_length += 8 + 8 + 8 + 8 + 8; // val + p + i + d + filter
        
        emitter << YAML::BeginMap << YAML::Key << "double" << YAML::Value << format_string("cc_%s_value", name.c_str()) << YAML::EndMap;
        emitter << YAML::BeginMap << YAML::Key << "uint32_t" << YAML::Value << format_string("cc_%s_mode", name.c_str()) << YAML::EndMap;
        emitter << YAML::BeginMap << YAML::Key << "double" << YAML::Value << format_string("cc_%s_gain_p", name.c_str()) << YAML::EndMap;
        emitter << YAML::BeginMap << YAML::Key << "double" << YAML::Value << format_string("cc_%s_gain_i", name.c_str()) << YAML::EndMap;
        emitter << YAML::BeginMap << YAML::Key << "double" << YAML::Value << format_string("cc_%s_gain_d", name.c_str()) << YAML::EndMap;
        emitter << YAML::BeginMap << YAML::Key << "double" << YAML::Value << format_string("cc_%s_filter", name.c_str()) << YAML::EndMap;
    }

    emitter << YAML::EndSeq;

    pd_ctrl_outputs.pd = make_shared<triple_buffer>(cc_outputs_struct_length, 
            parent->name, format_string("%s.outputs", name.c_str()), emitter.c_str());
    pd_ctrl_outputs.pd_hash = pd_ctrl_outputs.pd->set_consumer(shared_from_this());
    k.add_device(pd_ctrl_outputs.pd);

    pds[pd_ctrl_outputs.pd->id()] = pd_ctrl_outputs;

    for (auto& kv : overrides) {
        auto& item = kv.second;

        if (item.pd == "")
            item.pd = pd_ctrl_outputs.pd->id();
            
        add_pds(item, pds);
    }

    for (auto& kv : states) {
        auto& item = kv.second;

        if (item.pd == "")
            item.pd = pd_ctrl_outputs.pd->id();
        
        add_pds(item, pds);
    }

    // process data inspection
    k.add_device(shared_from_this());

    if (trigger_dev_name != "") {
        auto clk_dev = k.get_trigger(trigger_dev_name);
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

    if (trigger_dev_name != "") {
        auto clk_dev = k.get_trigger(trigger_dev_name);
        clk_dev->remove_trigger(shared_from_this());
    }

    // process data inspection
    k.remove_device(shared_from_this());

    k.remove_device(pd_ctrl_outputs.pd);
    pd_ctrl_outputs.pd->reset_consumer(pd_ctrl_outputs.pd_hash);

    pd_ctrl_outputs.pd_hash = 0;
    pd_ctrl_outputs.pd = nullptr;
    
    for (auto& kv : outputs) {
        auto& output = kv.second;
        auto& output_pd = pds[output.pd];

        if (!output_pd.pd)
            continue;

        output_pd.pd->reset_provider(output_pd.pd_hash);
        output_pd.pd_hash = 0;
        output_pd.pd = nullptr;

        output_pd.local_outputs.resize(0);
    }
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

    // setting overrides
    for (auto& pdi : overrides) {
        auto& output = pdi.second;
        auto& output_pd = get_map_entry(pds, output.pd);

        uint8_t *adr = &output_pd.pd->peek()[output.offset];
        memcpy(adr, &output.value[0], output.value.size());
    }

    const auto& buf_out = pd_ctrl_outputs.pd->pop(pd_ctrl_outputs.pd_hash);

    for (auto& kv : outputs) {
        auto& output = kv.second;
        auto& output_pd = get_map_entry(pds, output.pd);
        
        output.act_val = output.default_val;    
        
        // passing values
        memcpy(&output_pd.local_outputs[0], &buf_out[output_pd.pd_outputs_offset], output_pd.pd->length);
    }

    // check power states
    for (auto& kv : states) {
        auto& ps = kv.second;
        auto& ps_pd = get_map_entry(pds, ps.pd);

        uint8_t *adr = ps_pd.pd->peek();

        if (!check_state(adr, ps)) {
            for (auto& name : input_order) {
                // reset stuff
                auto& input = get_map_entry(inputs, name);
                input.i_part = 0.;
            }

            goto tick_exit;
        }
    }

    for (auto& name : input_order) {
        auto& input = get_map_entry(inputs, name);
        auto& output = get_map_entry(outputs, input.target);

        const auto& buf_in = pds[input.pd].pd->peek();
        cc_outputs_item_t *cc_outputs = (cc_outputs_item_t *)&buf_out[input.pd_ctrl_outputs_offset];
        
        double kp = input.kp,
               ki = input.ki,
               kd = input.kd,
               filter = input.filter;

        if (cc_outputs->mode == 1) {
            // use dynamic gains
            kp = cc_outputs->p;
            ki = cc_outputs->i;
            kd = cc_outputs->d;
            filter = cc_outputs->filter;
        }

        double filter_t_const = (1.0 / (2.0 * M_PI * filter));

        double msr = val_to_double(buf_in, input);
        double des = cc_outputs->value;
        double d_msr = (msr - input.msr_old) / ts;
        double d_msr_filt = filter_first_order(ts, d_msr, filter_t_const, &input.d_msr_filt_old);
        double d_des = (des - input.des_old) / ts;
        double d_des_filt = filter_first_order(ts, d_des, filter_t_const, &input.d_des_filt_old);

        double i_part = (des - msr);
        if (fabs(i_part) < input.i_window)
            input.i_part += i_part;

        output.act_val += 
            kp * (des - msr) + 
            ki * input.i_part + 
            kd * (d_des_filt - d_msr_filt);

        input.msr_old = msr;
        input.des_old = des;
    }

    for (auto& kv : outputs) {
        auto& output = kv.second;
        auto& output_pd = get_map_entry(pds, output.pd);

        if (output.limit > 0.) {
            if (output.act_val > output.limit)
                output.act_val = output.limit;
            else if (output.act_val < -output.limit)
                output.act_val = -output.limit;
        }
        
        // setting calculated value
        double_to_val(&output_pd.local_outputs[0], output, output.act_val);
    }

tick_exit:
    for (auto& kv : pds) {
        auto& output_pd = kv.second;

        if (output_pd.pd_hash)
            output_pd.pd->write(output_pd.pd_hash, 0, &output_pd.local_outputs[0], output_pd.local_outputs.size()); 
    }
}
                
// process data inspection
void pid_control::controller::get_pdin(service_provider::process_data_inspection::pd_t& pd) {
}

void pid_control::controller::get_pdout(service_provider::process_data_inspection::pd_t& pd) {
#ifdef oldcode
    const auto& buf = command_outputs.pd->peek();
    pd.resize(command_outputs.pd->length);
    memcpy(&pd[0], buf, pd.size());
#endif
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
    std::map<std::string, std::string> class_map;

    for (const auto& cls : config["classes"]) {
        auto class_name = cls.first.as<string>();

        YAML::Emitter out;
        out << cls.second;
    
        auto class_config = out.c_str();
        class_map[class_name] = class_config;
    }

    for (const auto& inst : config["instances"]) {
        auto class_name = get_as<string>(inst, "use_class");
        auto inst_config = class_map[class_name];

        std::map<string, string> inst_map;
        for (const auto& kv : inst) {
            inst_map[kv.first.as<string>()] = kv.second.as<string>();
            
            string var = string("(\\$") + kv.first.as<string>() + string(")");

            std::string result;
            std::regex e (var);
            std::regex_replace(std::back_inserter(result), inst_config.begin(), inst_config.end(), 
                    e, kv.second.as<string>());
            inst_config = result;
        }

        auto d = std::make_shared<controller>(shared_from_this(), YAML::Load(inst_config));
        ctrl_list.push_back(d);
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

    for (auto& d : ctrl_list) {
        if (d->trigger_dev_name == "")
            d->tick();
    }
}

