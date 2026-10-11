#include "app.h"
#include "hal.h"

int main(void)
{
    hal_init();
    app_init();
    for (;;) {
        app_main_loop_iteration();
    }
}
