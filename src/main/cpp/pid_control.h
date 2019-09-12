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
                        pd          = get_as<std::string>(ci_node, "pd",         "");
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
                        pd          = get_as<std::string>(ci_node, "pd"            );
                        kp          = get_as<double>     (ci_node, "kp",         1.);
                        ki          = get_as<double>     (ci_node, "ki",         1.);
                        kd          = get_as<double>     (ci_node, "kd",         1.);
                        i_window    = get_as<double>     (ci_node, "i_window",   1.);
                        filter      = get_as<double>     (ci_node, "filter",   100.);
                        target      = get_as<std::string>(ci_node, "target"        );
                    }

                    double kp;                          //!< proportional gain
                    double ki;                          //!< integral gain
                    double kd;                          //!< derivative gain
                    double filter;                      //!< filter frequency
                    double i_window;                    //!< window in between which we integrate
                    std::string target;                 //!< target value

                    double i_part         = 0.;
                    double msr_old        = 0.;
                    double d_msr_filt_old = 0.;
                    double des_old        = 0.;
                    double d_des_filt_old = 0.;
                    
                    off_t pd_ctrl_outputs_offset;
                } input_t;

                typedef struct output : io_base_t {
                    output(const YAML::Node& ci_node) : io_base(ci_node) {
                        pd          = get_as<std::string>(ci_node, "pd"            );
                        kt          = get_as<double>     (ci_node, "kt",         1.);
                        default_val = get_as<double>     (ci_node, "default",    0.);
                        limit       = get_as<double>     (ci_node, "limit",      0.);
                    }

                    double kt;                          //!< gain
                    double default_val;
                    double limit;
                    double act_val      = 0.;
                } output_t;

                typedef struct override_state : io_base_t {
                    override_state(const YAML::Node& ci_node) : io_base(ci_node) {
                        value_str = get_as<std::string>(ci_node, "value", "");
                        mask_str  = get_as<std::string>(ci_node, "mask", "");
                    }

                    std::string value_str;              //!< default value for overrides/states
                    std::vector<uint8_t> value;         //!< same

                    std::string mask_str;               //!< default value for state mask
                    std::vector<uint8_t> mask;          //!< same
                } override_state_t;

                std::map<std::string, input_t> inputs;
                std::list<std::string> input_order;
                std::map<std::string, output_t> outputs;
                std::list<std::string> output_order;
                std::map<std::string, override_state_t> overrides;
                std::map<std::string, override_state_t> states;

                typedef struct pd {
                    std::string dev_name;               //!< process data device name
                    robotkernel::sp_process_data_t pd;  //!< process data device from other module
                    size_t pd_hash;                     //!< provider hash
                } pd_t, input_pd_t;

                typedef struct output_pd : pd_t {
                    std::vector<uint8_t> local_outputs;
                    off_t pd_outputs_offset;
                } output_pd_t;

                std::map<std::string, input_pd_t> input_pds;
                std::map<std::string, output_pd_t> output_pds;

                bool do_reset;

                typedef struct __attribute__((__packed__)) cc_outputs_item {
                    double value;
                    uint32_t mode;
                    double p;
                    double i;
                    double d;
                    double filter;
                } cc_outputs_item_t;

                robotkernel::sp_process_data_t pd_ctrl_outputs;
                size_t pd_ctrl_outputs_hash;

            private:
                double ts;

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

