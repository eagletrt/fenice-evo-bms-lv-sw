# Documentation

## TO-DO

- [ ] scs signals
- [ ] timers
- [ ] errors
- [ ] cli
- [ ] document the calculation for the conversion and the vref
- [ ] ADC send values to can

## Notes

### Can manager

- copy default_can_manager_config.h to can_manager_config.h
- aggiungere CAN_MGR_STM32_APPLICATION se serve

### Timer

- slave mode
- differenze tra internal clock division e clock prescaler.
- output compare vs periodic elapsed callback*

### Errors

- setti errore e finche' hai un errore.
- te vuoi far scattare l'interrupt esattamente quando l'errore scade

### FSG2024 Rules

This are all the rules related do the LV system:

- T 11
- T 11.1
- T 11.2
- T 11.3
- T 11.6
- T 11.7
- T 11.9.5
- T 14.6.4
- EV 3.1
- EV 3.2.5
- EV 4.3
- EV 4.6.6
- EV 4.7.8
- EV 4.10.2
- EV 4.10.3
- EV 5.4.3
- EV 5.4.7
- EV 5.4.10
- EV 5.8.1
- EV 6.1 Figure 22
- EV 7 bms lv charger
- IN 4.1
