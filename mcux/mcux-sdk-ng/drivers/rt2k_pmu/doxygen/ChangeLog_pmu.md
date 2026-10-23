# PMU

## [2.1.0]
- New Features
  - Added PMU_GetDcdcConfig(), PMU_GetLdoVdd0V8Config(), PMU_GetLdoVdd1V8Config(), PMU_GetLdoVdda1V8Config().
- Bug Fixes
  - Fixed the field positions of PMU_LDO_CTRL_REG, PMU_SENSOR_CTRL0/1_REG, PMU_DIG_IRQ_EN_REG and
    PMU_RESETCON_WARM/COLD_CTRL to match the RT2660 reference manual, using the device header field macros.

## [2.0.0]
- Initial version.
