static struct emcmot_debug_t lcl = {0};
struct emcmot_debug_t *emcmotDebug = &lcl;

// KLUDGE fix link error the ugly way
void rtapi_print_msg(msg_level_t level, const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

void rtapi_print(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

