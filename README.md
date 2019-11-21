# module_pid_control

This module implements a pid controller.


d<sub>actual</sub> = (in<sub>actual</sub> - in<sub>actual_old</sub>) / ts  
d<sub>actual_filtered</sub> = filter_first_order(ts, d<sub>actual</sub>, filter_t_const, d<sub>actual_filtered_old</sub>)  
d<sub>desired</sub> = (in<sub>desired</sub> - in<sub>desired_old</sub>) / ts  
d<sub>desired_filtered</sub> = filter_first_order(ts, d<sub>desired</sub>, filter_t_const, d<sub>desired_filtered_old</sub>)  
  
i<sub>part</sub> += clamp(k<sub>i</sub> * (in<sub>desired</sub> - in<sub>actual</sub>), i<sub>limit</sub>)  
  
out<sub>desired</sub> = k<sub>p</sub> * (in<sub>desired</sub> - in<sub>actual</sub>) + i<sub>part</sub> + k<sub>d</sub> * (d<sub>desired_filtered</sub> - d<sub>actual_filtered</sub>)  

## Example configuration

The following example shows a configuration used to calculate linmot demand current depending on actual and target position.

```
- name: linmot_pid_control
  so_file: libmodule_pid_control.so
  config:
    loglevel: verbose
    classes:
      linmot_cc:
        name: $name
        inputs:
        - { name: position, pd: $lm_handler.inputs.pd, field_name: act_position, 
          kp: 3000., ki: 30., kd: 5., filter: 75., i_limit: 1., target: target_current }
        outputs:
        - { name: target_current, pd: $lm_handler.outputs.pd, field_name: target_current, kt: 1.0, limit: 10. }
        overrides:
        - { name: mode, field_name: $lm_handler.outputs.pd.mode, value: 16 } # current control
        power_states:
        - { name: control, field_name: $lm_handler.outputs.pd.control, value: 1, mask: 1 }    
        trigger: $trigger
    instances:
    - { name: linmot_1_cc, use_class: linmot_cc, lm_handler: linmot_1_handler, trigger: linmot_1_handler.inputs.trigger }
    #- { name: linmot_2_cc, use_class: linmot_cc, lm_handler: linmot_2_handler }
    #- { name: linmot_3_cc, use_class: linmot_cc, lm_handler: linmot_3_handler }
    #- { name: linmot_4_cc, use_class: linmot_cc, lm_handler: linmot_4_handler }
    #- { name: linmot_5_cc, use_class: linmot_cc, lm_handler: linmot_5_handler }
  depends: [ linmot_1_handler ] #, linmot_2_handler, linmot_3_handler
  power_up: op
```

