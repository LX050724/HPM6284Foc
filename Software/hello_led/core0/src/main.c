#include <stdio.h>
#include <string.h>

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_l1c_drv.h"
#include "hpm_debug_console.h"
#include "multicore_common.h"

static void secondary_core_image_load_and_run(void);

int main(void)
{
    board_init();

    secondary_core_image_load_and_run();

    printf("Hello world for multicore! Secondary core control led blinking...\n");

    int count = 0;
    while (1) {
        clock_cpu_delay_ms(1000);
        printf("hello world! %d\n", count);
        count += 1;
    }
    return 0;
}

static void secondary_core_image_load_and_run(void)
{
    multicore_release_cpu(HPM_CORE1, SEC_CORE_IMG_START);
}
