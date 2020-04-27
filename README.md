# module_pid_control

The **module_pid_control** is used to do PID control of given measurements to a command value. 

```yaml
- name: my_pid_control
  so_file: libmodule_pid_control.so
  config:
    loglevel: verbose
    classes:
      my_cc:
        name: $name
        inputs:
        - { name: position, pd: $my_handler.inputs.pd, field_name: position, 
          kp: 1., ki: 1., kd: 1., filter: 100., i_limit: 1., target: target_current_q }
        outputs:
        - { name: target_current_q, pd: $my_handler.outputs.pd, field_name: current_q, kt: 1.0, limit: 10. }
        overrides:
        - { name: target_current_d, field_name: $my_handler.outputs.pd.current_d, value: 0 } 
        #power_states:
        #- { name: control, field_name: $my_handler.outputs.pd.control, value: 1, mask: 1 }    
        trigger: $my_handler.inputs.trigger
    instances:
    - { name: my_1_cc, use_class: my_cc, my_handler: my_mod }
    - { name: my_2_cc, use_class: my_cc, my_handler: my_mod }
    - { name: my_3_cc, use_class: my_cc, my_handler: my_mod }
    - { name: my_4_cc, use_class: my_cc, my_handler: my_mod }
  depends: [ my_mod, ]
  power_up: op
```
