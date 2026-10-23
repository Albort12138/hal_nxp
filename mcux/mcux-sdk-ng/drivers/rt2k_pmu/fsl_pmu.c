/*
 * Copyright 2024-2026 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_pmu.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

static inline void PMU_SetOrClearBits(uint32_t *value, uint32_t mask, bool set)
{
    *value = set ? (*value | mask) : (*value & ~mask);
}

static bool PMU_IsBodyBiasStable(PMU_Type *base, const pmu_body_bias_config_t *config)
{
    uint32_t sts        = base->PMU_BODYBIAS_STATUS_STS;
    uint32_t nwellState = (sts & PMU_PMU_BODYBIAS_STATUS_STS_PMU_BB_NWELL_V_STATE_MASK) >>
                          PMU_PMU_BODYBIAS_STATUS_STS_PMU_BB_NWELL_V_STATE_SHIFT;
    uint32_t pwellState = (sts & PMU_PMU_BODYBIAS_STATUS_STS_PMU_BB_PWELL_V_STATE_MASK) >>
                          PMU_PMU_BODYBIAS_STATUS_STS_PMU_BB_PWELL_V_STATE_SHIFT;
    bool nwellOk = ((sts & PMU_PMU_BODYBIAS_STATUS_STS_PMU_BB_NWELL_V_OK_MASK) != 0UL);
    bool pwellOk = ((sts & PMU_PMU_BODYBIAS_STATUS_STS_PMU_BB_PWELL_V_OK_MASK) != 0UL);
    bool stable  = ((sts & PMU_PMU_BODYBIAS_STATUS_STS_PMU_BB_SAFE_STATE_MASK) != 0UL);

    if (config->mode == kPMU_BodyBiasFBBFull)
    {
        stable =
            stable && nwellOk && pwellOk && (nwellState == config->nwellVolSel) && (pwellState == config->pwellVolSel);
    }
    else if (config->mode == kPMU_BodyBiasFBBPWell)
    {
        stable = stable && pwellOk && (pwellState == config->pwellVolSel);
    }
    else if (config->mode == kPMU_BodyBiasFBBNWell)
    {
        stable = stable && nwellOk && (nwellState == config->nwellVolSel);
    }
    else if (config->mode == kPMU_BodyBiasZBB)
    {
        stable = stable && (nwellState == 0U) && (pwellState == 0U);
    }

    return stable;
}

uint32_t PMU_GetStatus(PMU_Type *base)
{
    return base->PMU_STS;
}

pmu_boot_error_reason_t PMU_GetBootErrorReason(PMU_Type *base)
{
    /* Extract the boot error reason code from the PMU status register. */
    return (pmu_boot_error_reason_t)((base->PMU_STS & PMU_PMU_STS_PMU_BOOT_ERROR_STS_MASK) >>
                                     PMU_PMU_STS_PMU_BOOT_ERROR_STS_SHIFT);
}

/* -------------------------------------------------------------------------- */
/* DCDC                                                                       */
/* -------------------------------------------------------------------------- */

void PMU_ConfigDcdc(PMU_Type *base, const pmu_dcdc_config_t *config)
{
    assert(config != NULL);

    /* The analog control register belongs to the DCDC alone, so it is written whole. IPK_ADJ selects the
     * over-current limit of each DCDC operating mode (PFM/STARTUP/PWM/DPD), which is why it carries config->mode. */
    uint32_t anaCtrl = 0U;
    anaCtrl |= PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_VOUT_ADJ(config->voutAdj);
    anaCtrl |= PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_VOUT_FINE_ADJ(config->voutFineAdj);
    anaCtrl |= PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_IPK_ADJ(config->mode);
    anaCtrl |= PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_OVL_VTH_ADJ(config->ovlVthAdj);
    anaCtrl |=
        PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_VSENSOR_VOUT_ADJ_EN(config->vsensorVoutAdjEn ? 1UL : 0UL);
    anaCtrl |= PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_BYPASS_EN(config->bypassEn ? 1UL : 0UL);
    anaCtrl |= PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_OVL_PD_EN(config->ovlPdEn ? 1UL : 0UL);
    base->PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG = anaCtrl;

    uint32_t sensorCtrl0 = base->PMU_SENSOR_CTRL0_REG;
    sensorCtrl0 &= ~(PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD_CORE_LOWER_THRES_MASK |
                     PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD_CORE_UPPER_THRES_MASK);
    sensorCtrl0 |= PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD_CORE_LOWER_THRES(config->sensorLowerThres);
    sensorCtrl0 |= PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD_CORE_UPPER_THRES(config->sensorUpperThres);
    base->PMU_SENSOR_CTRL0_REG = sensorCtrl0;

    /* BUCK_VDDCORE_IRQ is raised when the DCDC under-voltage lockout lasts longer than 8.96 us, so it
     * carries the UVLO actions. */
    uint32_t irqEnable = base->PMU_DIG_IRQ_EN_REG;
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDDCORE_VMIN_ERROR_IRQ_EN_MASK,
                       config->irqOnVminError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDDCORE_VMAX_ERROR_IRQ_EN_MASK,
                       config->irqOnVmaxError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_BUCK_VDDCORE_CLK_LOST_IRQ_EN_MASK,
                       config->irqOnClkLost);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_BUCK_VDDCORE_IRQ_IRQ_EN_MASK, config->irqOnUvlo);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_BUCK_VDDCORE_OCL_WARN_IRQ_EN_MASK,
                       config->irqOnOclWarn);
    base->PMU_DIG_IRQ_EN_REG = irqEnable;

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDDCORE_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnVminError);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDDCORE_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnVmaxError);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_BUCK_VDDCORE_CLK_LOST_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnClkLost);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_BUCK_VDDCORE_IRQ_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnUvlo);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_BUCK_VDDCORE_OCL_WARN_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnOclWarn);
    base->PMU_RESETCON_WARM_CTRL = warmReset;

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDDCORE_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnVminError);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDDCORE_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnVmaxError);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_BUCK_VDDCORE_CLK_LOST_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnClkLost);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_BUCK_VDDCORE_IRQ_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnUvlo);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_BUCK_VDDCORE_OCL_WARN_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnOclWarn);
    base->PMU_RESETCON_COLD_CTRL = coldReset;
}

void PMU_GetDcdcConfig(PMU_Type *base, pmu_dcdc_config_t *config)
{
    assert(config != NULL);

    uint32_t anaCtrl = base->PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG;
    config->mode = (pmu_dcdc_mode_t)((anaCtrl & PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_IPK_ADJ_MASK) >>
                                     PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_IPK_ADJ_SHIFT);
    config->voutAdj = (uint8_t)((anaCtrl & PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_VOUT_ADJ_MASK) >>
                                PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_VOUT_ADJ_SHIFT);
    config->voutFineAdj =
        (uint8_t)((anaCtrl & PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_VOUT_FINE_ADJ_MASK) >>
                  PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_VOUT_FINE_ADJ_SHIFT);
    config->ovlVthAdj = (uint8_t)((anaCtrl & PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_OVL_VTH_ADJ_MASK) >>
                                  PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_OVL_VTH_ADJ_SHIFT);
    config->vsensorVoutAdjEn =
        ((anaCtrl & PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_VSENSOR_VOUT_ADJ_EN_MASK) != 0UL);
    config->bypassEn = ((anaCtrl & PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_BYPASS_EN_MASK) != 0UL);
    config->ovlPdEn  = ((anaCtrl & PMU_PMU_BUCK_VDDCORE_ANACTRL_SETTING_REG_BUCK_VDDCORE_OVL_PD_EN_MASK) != 0UL);

    uint32_t sensorCtrl0 = base->PMU_SENSOR_CTRL0_REG;
    config->sensorLowerThres =
        (pmu_sensor_lower_thres_0v8_t)((sensorCtrl0 & PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD_CORE_LOWER_THRES_MASK) >>
                                       PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD_CORE_LOWER_THRES_SHIFT);
    config->sensorUpperThres =
        (pmu_sensor_upper_thres_0v8_t)((sensorCtrl0 & PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD_CORE_UPPER_THRES_MASK) >>
                                       PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD_CORE_UPPER_THRES_SHIFT);

    uint32_t irqEnable     = base->PMU_DIG_IRQ_EN_REG;
    config->irqOnVminError = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDDCORE_VMIN_ERROR_IRQ_EN_MASK) != 0UL);
    config->irqOnVmaxError = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDDCORE_VMAX_ERROR_IRQ_EN_MASK) != 0UL);
    config->irqOnClkLost   = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_BUCK_VDDCORE_CLK_LOST_IRQ_EN_MASK) != 0UL);
    config->irqOnUvlo      = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_BUCK_VDDCORE_IRQ_IRQ_EN_MASK) != 0UL);
    config->irqOnOclWarn   = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_BUCK_VDDCORE_OCL_WARN_IRQ_EN_MASK) != 0UL);

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    config->warmResetOnVminError =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDDCORE_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK) != 0UL);
    config->warmResetOnVmaxError =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDDCORE_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK) != 0UL);
    config->warmResetOnClkLost =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_BUCK_VDDCORE_CLK_LOST_ASYNC_RESET_WARM_EN_MASK) != 0UL);
    config->warmResetOnUvlo =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_BUCK_VDDCORE_IRQ_ASYNC_RESET_WARM_EN_MASK) != 0UL);
    config->warmResetOnOclWarn =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_BUCK_VDDCORE_OCL_WARN_ASYNC_RESET_WARM_EN_MASK) != 0UL);

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    config->coldResetOnVminError =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDDCORE_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK) != 0UL);
    config->coldResetOnVmaxError =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDDCORE_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK) != 0UL);
    config->coldResetOnClkLost =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_BUCK_VDDCORE_CLK_LOST_ASYNC_RESET_COLD_EN_MASK) != 0UL);
    config->coldResetOnUvlo =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_BUCK_VDDCORE_IRQ_ASYNC_RESET_COLD_EN_MASK) != 0UL);
    config->coldResetOnOclWarn =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_BUCK_VDDCORE_OCL_WARN_ASYNC_RESET_COLD_EN_MASK) != 0UL);
}

void PMU_ConfigDcdcPwmMode(PMU_Type *base, const pmu_dcdc_pwm_config_t *config)
{
    assert(config != NULL);

    uint32_t reg = 0U;
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_LOOPFILTER_RES_ADJ(config->loopFilterResAdj);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_LOOPFILTER_CAP_ADJ(config->loopFilterCapAdj);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_SAWTOOTH_ADJ(config->sawtoothAdj);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_GM_ADJ(config->gmAdj);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_GM_OVL_PULLUP_DIS(config->gmOvlPullupDis ? 1UL : 0UL);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_GM_INI_COND(config->gmIniCond ? 1UL : 0UL);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_IPK_OVR(config->ipkOvr ? 1UL : 0UL);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_PULSESKIP(config->pulseSkip);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_MASK_PEAKP(config->maskPeakP);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_MASK_SAWTOOTH(config->maskSawtooth);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_SET_PWM(config->setPwm);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_MASK_OCL(config->maskOcl);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_MASK_ZCS(config->maskZcs);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_DIS_PULSESKIPING(config->disPulseSkipping ? 1UL : 0UL);
    reg |= PMU_PMU_BUCK_VDDCORE_CTRL_PWM_REG_BUCK_VDDCORE_VOUT_STEPS(config->voutSteps ? 1UL : 0UL);
    base->PMU_BUCK_VDDCORE_CTRL_PWM_REG = reg;
}

/* -------------------------------------------------------------------------- */
/* LDOs                                                                       */
/* -------------------------------------------------------------------------- */

void PMU_ConfigLdoVdd0V8(PMU_Type *base, const pmu_ldo_vdd0v8_config_t *config)
{
    assert(config != NULL);

    uint32_t ldoCtrl = base->PMU_LDO_CTRL_REG;
    ldoCtrl &= ~(PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_SEL_MASK | PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_BYPASS_MASK |
                 PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_DISILIM_MASK | PMU_PMU_LDO_CTRL_REG_PMU_LPVDDREG_0V8_SEL_MASK);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_SEL(config->voltSel);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_BYPASS(config->bypassEn ? 1UL : 0UL);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_DISILIM(config->limitDisable ? 1UL : 0UL);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_LPVDDREG_0V8_SEL(config->lpVoltSel);
    base->PMU_LDO_CTRL_REG = ldoCtrl;

    uint32_t sensorCtrl0 = base->PMU_SENSOR_CTRL0_REG;
    sensorCtrl0 &= ~(PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD0V8_LOWER_THRES_MASK |
                     PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD0V8_UPPER_THRES_MASK);
    sensorCtrl0 |= PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD0V8_LOWER_THRES(config->sensorLowerThres);
    sensorCtrl0 |= PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD0V8_UPPER_THRES(config->sensorUpperThres);
    base->PMU_SENSOR_CTRL0_REG = sensorCtrl0;

    uint32_t irqEnable = base->PMU_DIG_IRQ_EN_REG;
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD0V8_VMIN_ERROR_IRQ_EN_MASK,
                       config->irqOnVminError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD0V8_VMAX_ERROR_IRQ_EN_MASK,
                       config->irqOnVmaxError);
    base->PMU_DIG_IRQ_EN_REG = irqEnable;

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD0V8_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnVminError);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD0V8_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnVmaxError);
    base->PMU_RESETCON_WARM_CTRL = warmReset;

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD0V8_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnVminError);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD0V8_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnVmaxError);
    base->PMU_RESETCON_COLD_CTRL = coldReset;
}

void PMU_GetLdoVdd0V8Config(PMU_Type *base, pmu_ldo_vdd0v8_config_t *config)
{
    assert(config != NULL);

    uint32_t ldoCtrl     = base->PMU_LDO_CTRL_REG;
    config->voltSel      = (pmu_ldo_volt_sel_t)((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_SEL_MASK) >>
                                           PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_SEL_SHIFT);
    config->lpVoltSel    = (pmu_ldo_volt_sel_t)((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_LPVDDREG_0V8_SEL_MASK) >>
                                             PMU_PMU_LDO_CTRL_REG_PMU_LPVDDREG_0V8_SEL_SHIFT);
    config->bypassEn     = ((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_BYPASS_MASK) != 0UL);
    config->limitDisable = ((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_VDD_0V8_DISILIM_MASK) != 0UL);

    uint32_t sensorCtrl0 = base->PMU_SENSOR_CTRL0_REG;
    config->sensorLowerThres =
        (pmu_sensor_lower_thres_0v8_t)((sensorCtrl0 & PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD0V8_LOWER_THRES_MASK) >>
                                       PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD0V8_LOWER_THRES_SHIFT);
    config->sensorUpperThres =
        (pmu_sensor_upper_thres_0v8_t)((sensorCtrl0 & PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD0V8_UPPER_THRES_MASK) >>
                                       PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD0V8_UPPER_THRES_SHIFT);

    uint32_t irqEnable     = base->PMU_DIG_IRQ_EN_REG;
    config->irqOnVminError = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD0V8_VMIN_ERROR_IRQ_EN_MASK) != 0UL);
    config->irqOnVmaxError = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD0V8_VMAX_ERROR_IRQ_EN_MASK) != 0UL);

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    config->warmResetOnVminError =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD0V8_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK) != 0UL);
    config->warmResetOnVmaxError =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD0V8_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK) != 0UL);

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    config->coldResetOnVminError =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD0V8_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK) != 0UL);
    config->coldResetOnVmaxError =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD0V8_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK) != 0UL);
}

void PMU_ConfigLdoVdd1V8(PMU_Type *base, const pmu_ldo_vdd1v8_config_t *config)
{
    assert(config != NULL);

    /* VDD_1V8_DISILIM has no config field, so it keeps its current value. */
    uint32_t ldoCtrl = base->PMU_LDO_CTRL_REG;
    ldoCtrl &= ~(PMU_PMU_LDO_CTRL_REG_PMU_VDD_1V8_SEL_MASK | PMU_PMU_LDO_CTRL_REG_PMU_VDD_1V8_BYPASS_MASK |
                 PMU_PMU_LDO_CTRL_REG_PMU_LPVDDREG_1V8_SEL_MASK);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_VDD_1V8_SEL(config->voltSel);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_VDD_1V8_BYPASS(config->bypassEn ? 1UL : 0UL);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_LPVDDREG_1V8_SEL(config->lpVoltSel);
    base->PMU_LDO_CTRL_REG = ldoCtrl;

    uint32_t sensorCtrl0 = base->PMU_SENSOR_CTRL0_REG;
    sensorCtrl0 &= ~(PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD1V8_LOWER_THRES_MASK |
                     PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD1V8_UPPER_THRES_MASK);
    sensorCtrl0 |= PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD1V8_LOWER_THRES(config->sensorLowerThres);
    sensorCtrl0 |= PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD1V8_UPPER_THRES(config->sensorUpperThres);
    base->PMU_SENSOR_CTRL0_REG = sensorCtrl0;

    uint32_t irqEnable = base->PMU_DIG_IRQ_EN_REG;
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD1V8_VMIN_ERROR_IRQ_EN_MASK,
                       config->irqOnVminError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD1V8_VMAX_ERROR_IRQ_EN_MASK,
                       config->irqOnVmaxError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_MONITOR_VDD1V8_OK_IRQ_EN_MASK,
                       config->irqOnMonitorOk);
    base->PMU_DIG_IRQ_EN_REG = irqEnable;

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD1V8_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnVminError);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD1V8_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnVmaxError);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_MONITOR_VDD1V8_OK_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnMonitorOk);
    base->PMU_RESETCON_WARM_CTRL = warmReset;

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD1V8_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnVminError);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD1V8_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnVmaxError);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_MONITOR_VDD1V8_OK_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnMonitorOk);
    base->PMU_RESETCON_COLD_CTRL = coldReset;
}

void PMU_GetLdoVdd1V8Config(PMU_Type *base, pmu_ldo_vdd1v8_config_t *config)
{
    assert(config != NULL);

    uint32_t ldoCtrl  = base->PMU_LDO_CTRL_REG;
    config->voltSel   = (pmu_ldo_volt_sel_t)((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_VDD_1V8_SEL_MASK) >>
                                           PMU_PMU_LDO_CTRL_REG_PMU_VDD_1V8_SEL_SHIFT);
    config->lpVoltSel = (pmu_ldo_volt_sel_t)((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_LPVDDREG_1V8_SEL_MASK) >>
                                             PMU_PMU_LDO_CTRL_REG_PMU_LPVDDREG_1V8_SEL_SHIFT);
    config->bypassEn  = ((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_VDD_1V8_BYPASS_MASK) != 0UL);

    uint32_t sensorCtrl0 = base->PMU_SENSOR_CTRL0_REG;
    config->sensorLowerThres =
        (pmu_sensor_lower_thres_1v8_t)((sensorCtrl0 & PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD1V8_LOWER_THRES_MASK) >>
                                       PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD1V8_LOWER_THRES_SHIFT);
    config->sensorUpperThres =
        (pmu_sensor_upper_thres_1v8_t)((sensorCtrl0 & PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD1V8_UPPER_THRES_MASK) >>
                                       PMU_PMU_SENSOR_CTRL0_REG_PMU_SENSOR_VDD1V8_UPPER_THRES_SHIFT);

    uint32_t irqEnable     = base->PMU_DIG_IRQ_EN_REG;
    config->irqOnVminError = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD1V8_VMIN_ERROR_IRQ_EN_MASK) != 0UL);
    config->irqOnVmaxError = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD1V8_VMAX_ERROR_IRQ_EN_MASK) != 0UL);
    config->irqOnMonitorOk = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_MONITOR_VDD1V8_OK_IRQ_EN_MASK) != 0UL);

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    config->warmResetOnVminError =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD1V8_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK) != 0UL);
    config->warmResetOnVmaxError =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD1V8_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK) != 0UL);
    config->warmResetOnMonitorOk =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_MONITOR_VDD1V8_OK_ASYNC_RESET_WARM_EN_MASK) != 0UL);

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    config->coldResetOnVminError =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD1V8_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK) != 0UL);
    config->coldResetOnVmaxError =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD1V8_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK) != 0UL);
    config->coldResetOnMonitorOk =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_MONITOR_VDD1V8_OK_ASYNC_RESET_COLD_EN_MASK) != 0UL);
}

void PMU_ConfigLdoVdda1V8(PMU_Type *base, const pmu_ldo_vdda1v8_config_t *config)
{
    assert(config != NULL);

    uint32_t ldoCtrl = base->PMU_LDO_CTRL_REG;
    ldoCtrl &= ~(PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_SEL_MASK | PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_BYPASS_MASK |
                 PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_DISILIM_MASK);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_SEL(config->voltSel);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_BYPASS(config->bypassEn ? 1UL : 0UL);
    ldoCtrl |= PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_DISILIM(config->limitDisable ? 1UL : 0UL);
    base->PMU_LDO_CTRL_REG = ldoCtrl;

    uint32_t sensorCtrl1 = base->PMU_SENSOR_CTRL1_REG;
    sensorCtrl1 &= ~(PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDDA1V8_LOWER_THRES_MASK |
                     PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDDA1V8_UPPER_THRES_MASK);
    sensorCtrl1 |= PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDDA1V8_LOWER_THRES(config->sensorLowerThres);
    sensorCtrl1 |= PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDDA1V8_UPPER_THRES(config->sensorUpperThres);
    base->PMU_SENSOR_CTRL1_REG = sensorCtrl1;

    uint32_t irqEnable = base->PMU_DIG_IRQ_EN_REG;
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDDA1V8_VMIN_ERROR_IRQ_EN_MASK,
                       config->irqOnVminError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDDA1V8_VMAX_ERROR_IRQ_EN_MASK,
                       config->irqOnVmaxError);
    base->PMU_DIG_IRQ_EN_REG = irqEnable;

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDDA1V8_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnVminError);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDDA1V8_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnVmaxError);
    base->PMU_RESETCON_WARM_CTRL = warmReset;

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDDA1V8_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnVminError);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDDA1V8_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnVmaxError);
    base->PMU_RESETCON_COLD_CTRL = coldReset;
}

void PMU_GetLdoVdda1V8Config(PMU_Type *base, pmu_ldo_vdda1v8_config_t *config)
{
    assert(config != NULL);

    uint32_t ldoCtrl     = base->PMU_LDO_CTRL_REG;
    config->voltSel      = (pmu_ldo_volt_sel_t)((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_SEL_MASK) >>
                                           PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_SEL_SHIFT);
    config->bypassEn     = ((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_BYPASS_MASK) != 0UL);
    config->limitDisable = ((ldoCtrl & PMU_PMU_LDO_CTRL_REG_PMU_VDDA_1V8_DISILIM_MASK) != 0UL);

    uint32_t sensorCtrl1 = base->PMU_SENSOR_CTRL1_REG;
    config->sensorLowerThres =
        (pmu_sensor_lower_thres_1v8_t)((sensorCtrl1 & PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDDA1V8_LOWER_THRES_MASK) >>
                                       PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDDA1V8_LOWER_THRES_SHIFT);
    config->sensorUpperThres =
        (pmu_sensor_upper_thres_1v8_t)((sensorCtrl1 & PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDDA1V8_UPPER_THRES_MASK) >>
                                       PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDDA1V8_UPPER_THRES_SHIFT);

    uint32_t irqEnable     = base->PMU_DIG_IRQ_EN_REG;
    config->irqOnVminError = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDDA1V8_VMIN_ERROR_IRQ_EN_MASK) != 0UL);
    config->irqOnVmaxError = ((irqEnable & PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDDA1V8_VMAX_ERROR_IRQ_EN_MASK) != 0UL);

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    config->warmResetOnVminError =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDDA1V8_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK) != 0UL);
    config->warmResetOnVmaxError =
        ((warmReset & PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDDA1V8_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK) != 0UL);

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    config->coldResetOnVminError =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDDA1V8_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK) != 0UL);
    config->coldResetOnVmaxError =
        ((coldReset & PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDDA1V8_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK) != 0UL);
}

/* -------------------------------------------------------------------------- */
/* VDD_PMU Monitor                                                            */
/* -------------------------------------------------------------------------- */

void PMU_ConfigVddPmu(PMU_Type *base, const pmu_vdd_pmu_config_t *config)
{
    assert(config != NULL);

    uint32_t sensorCtrl1 = base->PMU_SENSOR_CTRL1_REG;
    sensorCtrl1 &= ~(PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDD_PMU3V_LOWER_THRES_MASK |
                     PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDD_PMU3V_UPPER_THRES_MASK |
                     PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDD_PMU1V8_LOWER_THRES_MASK |
                     PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDD_PMU1V8_UPPER_THRES_MASK);
    sensorCtrl1 |= PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDD_PMU3V_LOWER_THRES(config->sensor3VLowerThres);
    sensorCtrl1 |= PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDD_PMU3V_UPPER_THRES(config->sensor3VUpperThres);
    sensorCtrl1 |= PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDD_PMU1V8_LOWER_THRES(config->sensor1V8LowerThres);
    sensorCtrl1 |= PMU_PMU_SENSOR_CTRL1_REG_PMU_SENSOR_VDD_PMU1V8_UPPER_THRES(config->sensor1V8UpperThres);
    base->PMU_SENSOR_CTRL1_REG = sensorCtrl1;

    uint32_t irqEnable = base->PMU_DIG_IRQ_EN_REG;
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_MONITOR_VDDPMU_3V_OK_IRQ_EN_MASK,
                       config->irqOnMonitor3VOk);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD_PMU1V8_VMAX_ERROR_IRQ_EN_MASK,
                       config->irqOnSensor1V8VmaxError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD_PMU1V8_VMIN_ERROR_IRQ_EN_MASK,
                       config->irqOnSensor1V8VminError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD_PMU3V_VMAX_ERROR_IRQ_EN_MASK,
                       config->irqOnSensor3VVmaxError);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_SENSE_VDD_PMU3V_VMIN_ERROR_IRQ_EN_MASK,
                       config->irqOnSensor3VVminError);
    base->PMU_DIG_IRQ_EN_REG = irqEnable;

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_MONITOR_VDDPMU_3V_OK_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnMonitor3VOk);
    PMU_SetOrClearBits(&warmReset,
                       PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD_PMU1V8_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnSensor1V8VmaxError);
    PMU_SetOrClearBits(&warmReset,
                       PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD_PMU1V8_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnSensor1V8VminError);
    PMU_SetOrClearBits(&warmReset,
                       PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD_PMU3V_VMAX_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnSensor3VVmaxError);
    PMU_SetOrClearBits(&warmReset,
                       PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_SENSE_VDD_PMU3V_VMIN_ERROR_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnSensor3VVminError);
    base->PMU_RESETCON_WARM_CTRL = warmReset;

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_MONITOR_VDDPMU_3V_OK_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnMonitor3VOk);
    PMU_SetOrClearBits(&coldReset,
                       PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD_PMU1V8_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnSensor1V8VmaxError);
    PMU_SetOrClearBits(&coldReset,
                       PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD_PMU1V8_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnSensor1V8VminError);
    PMU_SetOrClearBits(&coldReset,
                       PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD_PMU3V_VMAX_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnSensor3VVmaxError);
    PMU_SetOrClearBits(&coldReset,
                       PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_SENSE_VDD_PMU3V_VMIN_ERROR_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnSensor3VVminError);
    base->PMU_RESETCON_COLD_CTRL = coldReset;
}

/* -------------------------------------------------------------------------- */
/* Body Bias                                                                  */
/* -------------------------------------------------------------------------- */

void PMU_ConfigBodyBias(PMU_Type *base, const pmu_body_bias_config_t *config)
{
    assert(config != NULL);

    /* Fields this call always owns. NWELL_VOL/PWELL_VOL are added to the mask only when
     * PMU_DRIVER_UPDATE_TRIM_VALUE is enabled; otherwise this is a read-modify-write that
     * never clobbers whatever well-bias trim is already latched into the register (e.g. ROM). */
    uint32_t ownedFieldsMask =
        PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_MODE_SEL_MASK | PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_PWELL_CLK_SEL_MASK |
        PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_RAMP_CLK_SEL_MASK | PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_ADVC_MASK;
#if PMU_DRIVER_UPDATE_TRIM_VALUE
    ownedFieldsMask |=
        PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_NWELL_V_SEL_MASK | PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_PWELL_V_SEL_MASK;
#endif /* PMU_DRIVER_UPDATE_TRIM_VALUE */

    uint32_t reg = base->PMU_BODYBIAS_CONFIG_REG & ~ownedFieldsMask;
    reg |= PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_MODE_SEL(config->mode);
    reg |= PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_PWELL_CLK_SEL(config->pwellClkSel);
    reg |= PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_RAMP_CLK_SEL(config->rampClkSel);
    reg |= PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_ADVC(config->advcEnable ? 1UL : 0UL);
#if PMU_DRIVER_UPDATE_TRIM_VALUE
    reg |= PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_NWELL_V_SEL(config->nwellVolSel);
    reg |= PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_PWELL_V_SEL(config->pwellVolSel);
#endif /* PMU_DRIVER_UPDATE_TRIM_VALUE */
    base->PMU_BODYBIAS_CONFIG_REG = reg;

    uint32_t intEn = 0U;
    intEn |= PMU_PMU_BODYBIAS_INTENABLE_REG_PMU_BB_ENA_CFG_DONE(config->irqOnCfgDone ? 1UL : 0UL);
    intEn |= PMU_PMU_BODYBIAS_INTENABLE_REG_PMU_BB_ENA_CFG_ERROR(config->irqOnCfgError ? 1UL : 0UL);
    base->PMU_BODYBIAS_INTENABLE_REG = intEn;
}

status_t PMU_ApplyBodyBiasBlocking(PMU_Type *base)
{
    uint32_t configReg = base->PMU_BODYBIAS_CONFIG_REG;
    pmu_body_bias_config_t appliedConfig;

    appliedConfig.mode        = (pmu_body_bias_mode_t)((configReg & PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_MODE_SEL_MASK) >>
                                                PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_MODE_SEL_SHIFT);
    appliedConfig.nwellVolSel = (uint8_t)((configReg & PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_NWELL_V_SEL_MASK) >>
                                          PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_NWELL_V_SEL_SHIFT);
    appliedConfig.pwellVolSel = (uint8_t)((configReg & PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_PWELL_V_SEL_MASK) >>
                                          PMU_PMU_BODYBIAS_CONFIG_REG_PMU_BB_PWELL_V_SEL_SHIFT);

    PMU_ApplyBodyBiasNonBlocking(base);

    /* Poll for completion or error. */
#if PMU_BB_UPDATE_TIMEOUT
    uint32_t timeout = PMU_BB_UPDATE_TIMEOUT;
#endif
    while (true)
    {
        uint32_t sts = base->PMU_BODYBIAS_INTSTATUS_STS;
        if ((sts & PMU_PMU_BODYBIAS_INTSTATUS_STS_PMU_BB_CFG_ERROR_MASK) != 0UL)
        {
            return kStatus_Fail; /* Error */
        }
        if ((sts & PMU_PMU_BODYBIAS_INTSTATUS_STS_PMU_BB_CFG_DONE_MASK) != 0UL)
        {
            break; /* Done */
        }
#if PMU_BB_UPDATE_TIMEOUT
        if ((--timeout) == 0U)
        {
            return kStatus_Timeout;
        }
#endif
    }

#if PMU_BB_UPDATE_TIMEOUT
    timeout = PMU_BB_UPDATE_TIMEOUT;
#endif
    while (!PMU_IsBodyBiasStable(base, &appliedConfig))
    {
#if PMU_BB_UPDATE_TIMEOUT
        if ((--timeout) == 0U)
        {
            return kStatus_Timeout;
        }
#endif
    }

    return kStatus_Success;
}

void PMU_ApplyBodyBiasNonBlocking(PMU_Type *base)
{
    base->PMU_BODYBIAS_INTCLEAR_REG = PMU_PMU_BODYBIAS_INTCLEAR_REG_PMU_BB_CLR_CFG_DONE_MASK |
                                      PMU_PMU_BODYBIAS_INTCLEAR_REG_PMU_BB_CLR_CFG_ERROR_MASK;
    base->PMU_BODYBIAS_COMMAND_REG = PMU_PMU_BODYBIAS_COMMAND_REG_PMU_BB_CFG_UPDATE_MASK;
}

void PMU_GetDefaultBodyBiasConfig(pmu_body_bias_config_t *config)
{
    assert(config != NULL);

    config->mode          = kPMU_BodyBiasZBB;
    config->nwellVolSel   = 0U;
    config->pwellVolSel   = 0U;
    config->advcEnable    = false;
    config->rampClkSel    = kPMU_BbRampClk750kHz;
    config->pwellClkSel   = kPMU_BbPwellClkDiv1;
    config->irqOnCfgDone  = false;
    config->irqOnCfgError = false;
}

/* -------------------------------------------------------------------------- */
/* Temperature Sensor                                                         */
/* -------------------------------------------------------------------------- */

void PMU_ConfigTemperatureSensor(PMU_Type *base, const pmu_temperature_sensor_config_t *config)
{
    assert(config != NULL);

    uint32_t reg = 0U;
    reg |= PMU_PMU_TEMP_SENSOR_PMU_TEMP_SENSOR_TH_THRE(config->htThreshold);
    reg |= PMU_PMU_TEMP_SENSOR_PMU_TEMP_SENSOR_TL_THRE(config->ltThreshold);
    reg |= PMU_PMU_TEMP_SENSOR_PMU_TEMP_SENSOR_TRIM_EN(config->trimEnable ? 1UL : 0UL);
    base->PMU_TEMP_SENSOR = reg;

    uint32_t irqEnable = base->PMU_DIG_IRQ_EN_REG;
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_TEMP_SENSOR_HT_OK_IRQ_EN_MASK, config->irqOnHtOk);
    PMU_SetOrClearBits(&irqEnable, PMU_PMU_DIG_IRQ_EN_REG_PMU_DIG_TEMP_SENSOR_LT_OK_IRQ_EN_MASK, config->irqOnLtOk);
    base->PMU_DIG_IRQ_EN_REG = irqEnable;

    uint32_t warmReset = base->PMU_RESETCON_WARM_CTRL;
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_TEMP_SENSOR_HT_OK_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnHtOk);
    PMU_SetOrClearBits(&warmReset, PMU_PMU_RESETCON_WARM_CTRL_PMU_DIG_TEMP_SENSOR_LT_OK_ASYNC_RESET_WARM_EN_MASK,
                       config->warmResetOnLtOk);
    base->PMU_RESETCON_WARM_CTRL = warmReset;

    uint32_t coldReset = base->PMU_RESETCON_COLD_CTRL;
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_TEMP_SENSOR_HT_OK_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnHtOk);
    PMU_SetOrClearBits(&coldReset, PMU_PMU_RESETCON_COLD_CTRL_PMU_DIG_TEMP_SENSOR_LT_OK_ASYNC_RESET_COLD_EN_MASK,
                       config->coldResetOnLtOk);
    base->PMU_RESETCON_COLD_CTRL = coldReset;
}

/* -------------------------------------------------------------------------- */
/* Oscillators                                                                */
/* -------------------------------------------------------------------------- */

void PMU_ConfigXOSC32K(PMU_Type *base, const pmu_xosc32k_config_t *config)
{
    assert(config != NULL);

    uint32_t reg = 0U;
    reg |= PMU_PMU_XOSC32KNP_CTRL_REG_XOSC32KNP_CLKOUT_EN(config->clkoutEnable ? 1UL : 0UL);
    reg |= PMU_PMU_XOSC32KNP_CTRL_REG_XOSC32KNP_MODE(config->tcxoMode ? 1UL : 0UL);
    reg |= PMU_PMU_XOSC32KNP_CTRL_REG_XOSC32KNP_BYPASS_EN(config->bypassEnable ? 1UL : 0UL);
    reg |= PMU_PMU_XOSC32KNP_CTRL_REG_XOSC32KNP_CLKMON_EN(config->clkMonEnable ? 1UL : 0UL);
    reg |= PMU_PMU_XOSC32KNP_CTRL_REG_XOSC32KNP_GM_SEL0(config->gmSel0 ? 1UL : 0UL);
    reg |= PMU_PMU_XOSC32KNP_CTRL_REG_XOSC32KNP_GM_SEL1(config->gmSel1 ? 1UL : 0UL);
    reg |= PMU_PMU_XOSC32KNP_CTRL_REG_XOSC32KNP_DLL0(config->dll0 ? 1UL : 0UL);
    reg |= PMU_PMU_XOSC32KNP_CTRL_REG_XOSC32KNP_DLL1(config->dll1 ? 1UL : 0UL);
    base->PMU_XOSC32KNP_CTRL_REG = reg;
}

/* -------------------------------------------------------------------------- */
/* Reset Gate Control                                                         */
/* -------------------------------------------------------------------------- */

void PMU_ConfigWarmResetGates(PMU_Type *base, uint32_t mask, bool enable)
{
    if (enable)
    {
        base->PMU_RESETCON_WARM_CTRL |= mask;
    }
    else
    {
        base->PMU_RESETCON_WARM_CTRL &= ~mask;
    }
}

void PMU_ConfigColdResetGates(PMU_Type *base, uint32_t mask, bool enable)
{
    if (enable)
    {
        base->PMU_RESETCON_COLD_CTRL |= mask;
    }
    else
    {
        base->PMU_RESETCON_COLD_CTRL &= ~mask;
    }
}

/* -------------------------------------------------------------------------- */
/* Interrupts                                                                 */
/* -------------------------------------------------------------------------- */

void PMU_EnableInterrupts(PMU_Type *base, uint32_t mask)
{
    base->PMU_DIG_IRQ_EN_REG |= mask;
}

void PMU_DisableInterrupts(PMU_Type *base, uint32_t mask)
{
    base->PMU_DIG_IRQ_EN_REG &= ~mask;
}

void PMU_ClearInterruptStatusFlags(PMU_Type *base, uint32_t clearMask)
{
    base->PMU_DIG_IRQ_FLAG_CLEAR_REG = clearMask;
}

uint32_t PMU_GetInterruptStatusFlags(PMU_Type *base)
{
    return base->PMU_DIG_IRQ_FLAG_REG;
}

uint32_t PMU_GetInterruptStatus(PMU_Type *base)
{
    return base->PMU_DIG_IRQ_STATUS_STS;
}
