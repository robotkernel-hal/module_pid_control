# module_pid_control

The **module_pid_control** is used to do PID control of given measurements to a command value. 

The output is calculation with the following equations: 

<img src="https://latex.codecogs.com/svg.latex?\Large&space;p_{part}=kp*(des_{in}-msr_{in})" title="\Large x=kp*(des_{in}-msr_{in})" />
<img src="https://latex.codecogs.com/svg.latex?\Large&space;i_{part}=\sum_{t} (ki * (des_{in} - msr_{in})" title="\Large x=kp*(des_{in}-msr_{in})" />
<img src="https://latex.codecogs.com/svg.latex?\Large&space;d_{part}=kd * (\frac{des_{in,n} - des_{in,n-1}}{t}-\frac{msr_{in,n} - msr_{in,n-1}}{t})" title="\Large x=kp*(des_{in}-msr_{in})" />
<img src="https://latex.codecogs.com/svg.latex?\Large&space;x_{out}=p_{part}+i_{part}+d_{part}" title="\Large x=kp*(des_{in}-msr_{in})" />

The derivative component are also filtered using a first order filter with frequency given in configuration.

## example configuration

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
