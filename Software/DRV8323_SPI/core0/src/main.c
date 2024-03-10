#include <stdio.h>
#include <string.h>

#include "board.h"
#include "drv832x.h"
#include "hpm_trgm_drv.h"
#include "multicore_common.h"

static void secondary_core_image_load_and_run(void);

int main(void)
{
    board_init();

    secondary_core_image_load_and_run();

    printf("Hello world for multicore! Secondary core control led blinking...\n");

    drv832x_init_pins();
    drv832x_init_spi();
    drv832x_enable(true);
    drv832x_dump_regs();

    while (1)
    {
    }
    return 0;
}

static void secondary_core_image_load_and_run(void)
{
    multicore_release_cpu(HPM_CORE1, SEC_CORE_IMG_START);
}
