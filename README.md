# module_pid_control

The **module_pid_control** is used to do PID control of given measurements to a command value. 

The output is calculated with the following equations: 

<img src="https://latex.codecogs.com/svg.latex?\Large&space;p_{part}=kp*(des_{in}-msr_{in})" title="\Large p_{part}=kp*(des_{in}-msr_{in})" />
<img src="https://latex.codecogs.com/svg.latex?\Large&space;i_{part}=\sum_{t} (ki * (des_{in} - msr_{in})" title="\Large i_{part}=\sum_{t} (ki * (des_{in} - msr_{in})" />
<img src="https://latex.codecogs.com/svg.latex?\Large&space;d_{part}=kd * (\frac{des_{in,n} - des_{in,n-1}}{t}-\frac{msr_{in,n} - msr_{in,n-1}}{t})" title="\Large kd * (\frac{des_{in,n} - des_{in,n-1}}{t}-\frac{msr_{in,n} - msr_{in,n-1}}{t})" />
<img src="https://latex.codecogs.com/svg.latex?\Large&space;x_{out}=kt*(p_{part}+i_{part}+d_{part})" title="\Large x_{out}=kt*(p_{part}+i_{part}+d_{part})" />

The derivative component are also filtered using a first order filter with frequency given in configuration.

## example configuration

The following example shows a configuration used to calculate linmot demand current depending on actual and target position.

```yaml
- name: my_pid_control
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
        trigger: $lm_handler.inputs.trigger
    instances:
    - { name: linmot_1_cc, use_class: linmot_cc, lm_handler: linmot_1_handler }
    - { name: linmot_2_cc, use_class: linmot_cc, lm_handler: linmot_2_handler }
    - { name: linmot_3_cc, use_class: linmot_cc, lm_handler: linmot_3_handler }
  depends: [ linmot_1_handler, linmot_2_handler, linmot_3_handler ]
  power_up: op
```

### Classes and Instances
------
To made it easier to instanciate multiple pid controller with the same config you can create **classes** and **instances**.

### PID config options
------
__name__: Each PID controller instance must have a unique name. This name is used to generate the appropriate robotkernel-5 process data devices.  

__inputs__: jajj  
··__name__:
=======
#### inputs
  
__name__:

#### outputs

#### power_states

#### overrides
