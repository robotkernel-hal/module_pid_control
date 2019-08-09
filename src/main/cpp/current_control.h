//! robotkernel module current_control
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

#ifndef MODULE_CURRENT_CONTROL_H
#define MODULE_CURRENT_CONTROL_H

#include "robotkernel/module.h"
#include "robotkernel/module_base.h"
#include "robotkernel/kernel.h"
#include "robotkernel/trigger.h"
#include "robotkernel/process_data.h"

#include "service_provider/process_data_inspection/base.h"

namespace module_current_control {
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

class current_control :
    public std::enable_shared_from_this<current_control>,
    public robotkernel::module_base
{
    public:
        /*
         *  controller:
         *    measure_inputs:
         *      dev_name: <name>
         *      position: { name: <name>, offset: 0, type: uint32_t )
         *      torque: { name: <name>, offset: 0, type: uint32_t )
         *
         *      pd_input_device: <name>
         *      pd_output_device: <name>
         *      outputs:
         *      -   name: left
         *          len: 8
         *      -   name: right
         *          len: 8
         *
         *
         */

        class controller : 
            public std::enable_shared_from_this<controller>,
            public robotkernel::trigger_base,
            public robotkernel::pd_provider,
            public robotkernel::pd_consumer,
            public service_provider::process_data_inspection::base
        {
            public:
                class output {
                    public: 
                        output(const std::string& name, const uint32_t& len) :
                            name(name), len(len), pdout(nullptr), pdtr(nullptr)
                        {
                        }

                        std::string name;
                        uint32_t len;
                        robotkernel::sp_process_data_t pdout;
                        robotkernel::sp_trigger_t      pdtr;
                        ssize_t hash;
                };

                struct pd_item {
                    std::string name;
                    off_t offset;
                    std::string type_str;
                    pd_data_types type;
                    double scale;
                };

                struct {
                    std::string dev_name;
                    pd_item position;
                    pd_item torque;
                } measure_inputs;

                struct {
                    std::string dev_name;
                    pd_item current;
                } command_outputs;

                robotkernel::sp_process_data_t pd_measure_inputs;
                size_t pd_measure_inputs_hash;
                robotkernel::sp_process_data_t pd_command_outputs;
                size_t pd_command_outputs_hash;

                bool with_torque;
                bool do_reset;

                const std::string pos_inputs_desc = 
                    "- uint32_t: mode\n"
                    "- double: target_pos\n"
                    "- double: gain_pos_proportional\n"
                    "- double: gain_pos_derivative\n";

                typedef struct __attribute__((__packed__)) pos_inputs {
                    uint32_t mode;
                    double target_pos;
                    double gain_pos_proportional;
                    double gain_pos_derivative;
                } __attribute__((__packed__)) pos_inputs_t;

                const std::string pos_tor_inputs_desc = 
                    "- uint32_t: mode\n"
                    "- double: target_pos\n"
                    "- double: gain_pos_proportional\n"
                    "- double: gain_pos_derivative\n"
                    "- double: target_tor\n"
                    "- double: gain_tor_proportional\n"
                    "- double: gain_tor_derivative\n";

                typedef struct __attribute__((__packed__)) pos_tor_inputs {
                    uint32_t mode;
                    double target_pos;
                    double gain_pos_proportional;
                    double gain_pos_derivative;
                    double target_tor;
                    double gain_tor_proportional;
                    double gain_tor_derivative;
                } __attribute__((__packed__)) pos_tor_inputs_t;

                robotkernel::sp_process_data_t pd_ctrl_inputs;
                size_t pd_ctrl_inputs_hash;

            private:
                double filter_freq;
                double filter_t_const;

                // controller gains
                double gain_pos_proportional;
                double gain_pos_derivative;
                double gain_tor_proportional;
                double gain_tor_derivative;

                double ts;

                double q_msr_old;
                double dq_msr_filt_old;
                double q_des_old;
                double dq_des_filt_old;
                double tau_msr_old;
                double dtau_msr_filt_old;
                double tau_des_old;
                double dtau_des_filt_old;

                std::shared_ptr<current_control> parent;
                std::list<output> outputs;
                std::string name; 

                struct {
                    std::string                     name;
                    ssize_t                         hash;
                    robotkernel::sp_process_data_t  dev;
                } pdin;

            public:
                //! construction
                /*!
                 * \param node yaml intialization node
                 */
                controller(std::shared_ptr<current_control> parent, const YAML::Node& node);
                ~controller() {};

                //! creating process data output and trigger
                void start();

                //! destroying process data output and trigger
                void stop();

                //! trigger tick
                void tick();                
                
                // process data inspection
                void get_pdin(service_provider::process_data_inspection::pd_t& pd) {};
                void get_pdout(service_provider::process_data_inspection::pd_t& pd) {};
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
        current_control(const std::string& name, const YAML::Node& node);

        //! destruction 
        ~current_control();

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
}; // namespace module_current_control

#endif // MODULE_CURRENT_CONTROL_H

