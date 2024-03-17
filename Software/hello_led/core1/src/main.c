#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_soc.h"
#include "hpm_sysctl_drv.h"
#include <stdio.h>


int main(void)
{
    clock_update_core_clock();

    HPM_IOC->PAD[IOC_PAD_PB11].FUNC_CTL = IOC_PB11_FUNC_CTL_GPIO_B_11;

    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, 11, gpiom_soc_gpio1);
    gpiom_disable_pin_visibility(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, 11, gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO1, GPIO_OE_GPIOB, 11);
    gpio_write_pin(HPM_GPIO1, GPIO_DO_GPIOB, 11, 1);

    while (1)
    {
        gpio_write_pin(HPM_GPIO1, GPIO_DO_GPIOB, 11, 0);
        clock_cpu_delay_ms(300);
        gpio_write_pin(HPM_GPIO1, GPIO_DO_GPIOB, 11, 1);
        clock_cpu_delay_ms(300);
    }
    return 0;
}
