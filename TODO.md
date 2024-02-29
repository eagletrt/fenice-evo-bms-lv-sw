### Can manager docs

- copy default_can_manager_config.h to can_manager_config.h
- aggiungere CAN_MGR_STM32_APPLICATION se serve

#

setti errore e finche' hai un errore

te vuoi far scattare l'interrupt esattamente quando l'errore scade

# Documentation

TO-DO

- [ ] document the calculation for the conversion and the vref

## ADC 

- Timer 10
  - interrupt every 10ms
  - used to start the ADC_routine()

TO-DO

- [ ] send values to can


### timer

slave mode
differenze tra internal clock division e clock prescaler
- output compare vs periodic elapsed callback

