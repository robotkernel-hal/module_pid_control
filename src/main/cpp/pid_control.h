//! robotkernel module pid_control
/*!
 * author: Robert Burger
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

#ifndef MODULE_PID_CONTROL_H
#define MODULE_PID_CONTROL_H

#include "robotkernel/module.h"
#include "robotkernel/module_base.h"
#include "robotkernel/kernel.h"
#include "robotkernel/trigger.h"
#include "robotkernel/process_data.h"

#include "service_provider/process_data_inspection/base.h"

namespace module_pid_control {
#ifdef EMACS
}
#endif

enum pd_data_types {
    PD_DT_FLOAT = 1,
    PD_DT_DOUBLE,
    PD_DT_UINT8,
    PD_DT_UINT16,
    PD_DT_UINT32,
    PD_DT_INT8,
    PD_DT_INT16,
    PD_DT_INT32
};

const double DEFAULT_FILTER_FREQ           = 100.;
    
const double DEFAULT_GAIN_POS_PROPORTIONAL = 1.;
const double DEFAULT_GAIN_POS_DERIVATIVE   = 1.;
const double DEFAULT_GAIN_TOR_PROPORTIONAL = 1.;
const double DEFAULT_GAIN_TOR_DERIVATIVE   = 1.;
const double DEFAULT_GAIN_TAU_TO_I         = 0.1;

const double DEFAULT_LIMIT_CURRENT         = 1.;

class pid_control :
    public std::enable_shared_from_this<pid_control>,
    public robotkernel::module_base
{
    public:
        class controller : 
            public std::enable_shared_from_this<controller>,
            public robotkernel::trigger_base,
            public robotkernel::pd_provider,
            public robotkernel::pd_consumer,
            public service_provider::process_data_inspection::base
        {
            public:
                std::string name;

                typedef struct io_base {
                    io_base(const YAML::Node& ci_node) {
                        pd          = get_as<std::string>(ci_node, "pd"            );
                        field_name  = get_as<std::string>(ci_node, "field_name", "");
                        offset      = get_as<uint32_t>   (ci_node, "offset",     0 );
                        type_str    = get_as<std::string>(ci_node, "type",       "");
                        scale       = get_as<double>     (ci_node, "scale",      1.);
                    }

                    virtual ~io_base() {};

                    std::string pd;                     //!< process data map entry name
                    std::string field_name;             //!< field name in process data
                    off_t offset;                       //!< offset of process data field
                    std::string type_str;               //!< data type name of field
                    pd_data_types type;                 //!< data type of field
                    double scale;                       //!< field scaling
                } io_base_t;

                typedef struct input : io_base_t {
                    input(const YAML::Node& ci_node) : io_base(ci_node) {
                        kp          = get_as<double>     (ci_node, "kp",         1.);
                        ki          = get_as<double>     (ci_node, "ki",         1.);
                        kd          = get_as<double>     (ci_node, "kd",         1.);
                        filter      = get_as<double>     (ci_node, "filter",   100.);
                        target      = get_as<std::string>(ci_node, "target"        );
                    }

                    double kp;                          //!< proportional gain
                    double ki;                          //!< integral gain
                    double kd;                          //!< derivative gain
                    double filter;                      //!< filter frequency
                    std::string target;                 //!< target value

                    off_t pd_ctrl_outputs_offset;
                
                    double msr_old        = 0.;
                    double d_msr_filt_old = 0.;
                    double des_old        = 0.;
                    double d_des_filt_old = 0.;
                } input_t;

                typedef struct output : io_base_t {
                    output(const YAML::Node& ci_node) : io_base(ci_node) {
                        kt          = get_as<double>     (ci_node, "kt",         1.);
                    }

                    double kt;                          //!< gain

                    double act_val;
                } output_t;

                typedef struct override_state : io_base_t {
                    override_state(const YAML::Node& ci_node) : io_base(ci_node) {
                    }

                    std::string value_str;              //!< default value for overrides/states
                    std::vector<uint8_t> value;         //!< same

                    std::string mask_str;               //!< default value for state mask
                    std::vector<uint8_t> mask;          //!< same
                } override_state_t;

                std::map<std::string, input_t> inputs;
                std::map<std::string, output_t> outputs;
                std::map<std::string, override_state_t> overrides;
                std::map<std::string, override_state_t> states;

                typedef struct pd {
                    std::string dev_name;               //!< process data device name
                    robotkernel::sp_process_data_t pd;  //!< process data device from other module
                    size_t pd_hash;                     //!< provider hash

                    off_t pd_ctrl_outputs_offset;              
                } pd_t;

                std::map<std::string, pd_t> pds;

//                typedef struct pd_item {
//                    std::string name;                   //!< field name of process data item
//                    off_t offset;                       //!< offset of process data field
//                    std::string type_str;               //!< data type name of field
//                    pd_data_types type;                 //!< data type of field
//                    double scale;                       //!< field scaling
//
//                    std::string value_str;              //!< default value for overrides/states
//                    std::vector<uint8_t> value;         //!< same
//
//                    std::string mask_str;               //!< default value for state mask
//                    std::vector<uint8_t> mask;          //!< same
//                } pd_item_t;
//
//                struct {
//                    std::string dev_name;               //!< process data device name
//                    robotkernel::sp_process_data_t pd;  //!< process data device from other module
//                    pd_item position;                   //!< position field in process data
//                    pd_item torque;                     //!< torque field in process data
//                } measure_inputs;
//
//                struct {
//                    std::string dev_name;               //!< process data device name
//                    robotkernel::sp_process_data_t pd;  //!< process data device from other module
//                    size_t pd_hash;                     //!< provider hash
//                    pd_item current;                    //!< current field in process data
//                } command_outputs;
//                
//                std::list<pd_item_t> overrides;
//                std::list<pd_item_t> power_states;
//
//                bool with_torque;
//                bool do_reset;

                std::vector<uint8_t> local_outputs;

                typedef struct __attribute__((__packed__)) cc_outputs_item {
                    double value;
                    double p;
                    double i;
                    double d;
                    double filter;
                } cc_outputs_item_t;

                const std::string pos_outputs_desc = 
                    "- uint32_t: cc_mode\n"
                    "- double: cc_target_pos\n"
                    "- double: cc_gain_pos_proportional\n"
                    "- double: cc_gain_pos_derivative\n"
                    "- double: cc_filter_freq\n";

                typedef struct __attribute__((__packed__)) pos_outputs {
                    uint32_t mode;
                    double target_pos;
                    double gain_pos_proportional;
                    double gain_pos_derivative;
                    double filter_freq;
                } __attribute__((__packed__)) pos_outputs_t;

                const std::string pos_tor_outputs_desc = 
                    "- uint32_t: cc_mode\n"
                    "- double: cc_target_pos\n"
                    "- double: cc_gain_pos_proportional\n"
                    "- double: cc_gain_pos_derivative\n"
                    "- double: cc_target_tor\n"
                    "- double: cc_gain_tor_proportional\n"
                    "- double: cc_gain_tor_derivative\n"
                    "- double: cc_filter_freq\n";

                typedef struct __attribute__((__packed__)) pos_tor_outputs {
                    uint32_t mode;
                    double target_pos;
                    double gain_pos_proportional;
                    double gain_pos_derivative;
                    double target_tor;
                    double gain_tor_proportional;
                    double gain_tor_derivative;
                    double filter_freq;
                } __attribute__((__packed__)) pos_tor_outputs_t;

                robotkernel::sp_process_data_t pd_ctrl_outputs;
                size_t pd_ctrl_outputs_hash;

            private:
//                double filter_freq;
//                double filter_t_const;
//
//                // controller gains
//                double gain_pos_proportional;
//                double gain_pos_derivative;
//                double gain_tor_proportional;
//                double gain_tor_derivative;
//                double gain_tau_to_i;

                double ts;

//                double q_msr_old            = 0.;
//                double dq_msr_filt_old      = 0.;
//                double q_des_old            = 0.;
//                double dq_des_filt_old      = 0.;
//                double tau_msr_old          = 0.;
//                double dtau_msr_filt_old    = 0.;
//                double tau_des_old          = 0.;
//                double dtau_des_filt_old    = 0.;

//                double limit_current        = 0.;

                std::shared_ptr<pid_control> parent;

            public:
                //! construction
                /*!
                 * \param node yaml intialization node
                 */
                controller(std::shared_ptr<pid_control> parent, const YAML::Node& node);
                ~controller() {};

                //! creating process data output and trigger
                void start();

                //! destroying process data output and trigger
                void stop();

                //! trigger tick
                void tick();                
                
                // process data inspection
                void get_pdin(service_provider::process_data_inspection::pd_t& pd);
                void get_pdout(service_provider::process_data_inspection::pd_t& pd);
        };
        
        typedef std::shared_ptr<controller> sp_ctrls_t;
        typedef std::list<sp_ctrls_t> ctrls_list_t;
        ctrls_list_t ctrl_list;

        YAML::Node config;
    public:
        //! construction
        /*!
         * \param node yaml intialization node
         */
        pid_control(const std::string& name, const YAML::Node& node);

        //! destruction 
        ~pid_control();

        //! initializaion
        void init();

        //! set module state machine to defined state
        /*!
         * \param state requested state
         * \return success or failure
         */
        int set_state(module_state_t state);

        //! trigger tick
        void tick();                
};

#ifdef EMACS
{
#endif
}; // namespace module_pid_control

#endif // MODULE_PID_CONTROL_H

