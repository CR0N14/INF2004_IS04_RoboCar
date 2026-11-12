/** @file drv_ir.c
 * @brief Kernel ADC adapter; local analog setup leaves GP29 untouched.
 */
#include "drv_ir.h"
ER drv_ir_open (drv_ir_t * p_driver)
{
    if (NULL == p_driver) { return E_PAR; }
    for (UINT pin = 26u; pin <= 28u; ++pin)
    {
        out_w(GPIO_OE_CLR, 1u << pin);
        out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
        clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
    }
    p_driver->adc = tk_opn_dev((UB*)"adca", TD_READ);
    return (p_driver->adc < E_OK) ? p_driver->adc : E_OK;
}
uint16_t drv_ir_clock (void * p_context)
{
    drv_ir_t * p_driver = p_context;
    SYSTIM time = {0};
    if (E_OK != tk_get_otm(&time)) { p_driver->b_clock_failed = true; }
    return (uint16_t)time.lo;
}
bool drv_ir_read (void * p_context, uint8_t channel, uint16_t * p_raw)
{
    drv_ir_t * p_driver = p_context;
    if ((NULL == p_driver) || (NULL == p_raw) ||
        (channel >= IR_CHANNEL_COUNT)) { return false; }
    UW value = 0u;
    SZ size = 0;
    ER const error = tk_srea_dev(p_driver->adc, (W)channel, &value, 1, &size);
    if ((E_OK != error) ||
        (1 != size) || (value > IR_ADC_MAX)) { return false; }
    *p_raw = (uint16_t)value;
    return true;
}
/* End of file. */

