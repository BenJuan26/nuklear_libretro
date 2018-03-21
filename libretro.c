#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#if defined(__CELLOS_LV2__) && !defined(__PSL1GHT__)
#include <sys/timer.h>
#elif defined(XENON)
#include <time/time.h>
#elif defined(GEKKO) || defined(__PSL1GHT__) || defined(__QNX__)
#include <unistd.h>
#elif defined(PSP)
#include <pspthreadman.h> 
#elif defined(VITA)
#include <psp2/kernel/threadmgr.h>
#elif defined(_3DS)
#include <3ds.h>
#else
#include <time.h>
#endif

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720

#include "libretro.h"

#ifdef RARCH_INTERNAL
#include "internal_cores.h"
#define NETRETROPAD_CORE_PREFIX(s) libretro_netretropad_##s
#else
#define NETRETROPAD_CORE_PREFIX(s) s
#endif

#define DESC_NUM_PORTS(desc) ((desc)->port_max - (desc)->port_min + 1)
#define DESC_NUM_INDICES(desc) ((desc)->index_max - (desc)->index_min + 1)
#define DESC_NUM_IDS(desc) ((desc)->id_max - (desc)->id_min + 1)

#define DESC_OFFSET(desc, port, index, id) ( \
   port * ((desc)->index_max - (desc)->index_min + 1) * ((desc)->id_max - (desc)->id_min + 1) + \
   index * ((desc)->id_max - (desc)->id_min + 1) + \
   id \
)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#if defined(_WIN32) || defined(__INTEL_COMPILER)
#define INLINE __inline
#elif defined(__STDC_VERSION__) && __STDC_VERSION__>=199901L
#define INLINE inline
#elif defined(__GNUC__)
#define INLINE __inline__
#else
#define INLINE
#endif

/**
 * retro_sleep:
 * @msec         : amount in milliseconds to sleep
 *
 * Sleeps for a specified amount of milliseconds (@msec).
 **/
static INLINE void retro_sleep(unsigned msec)
{
#if defined(__CELLOS_LV2__) && !defined(__PSL1GHT__)
   sys_timer_usleep(1000 * msec);
#elif defined(PSP) || defined(VITA)
   sceKernelDelayThread(1000 * msec);
#elif defined(_3DS)
   svcSleepThread(1000000 * (s64)msec);
#elif defined(_WIN32)
   Sleep(msec);
#elif defined(XENON)
   udelay(1000 * msec);
#elif defined(GEKKO) || defined(__PSL1GHT__) || defined(__QNX__)
   usleep(1000 * msec);
#else
   struct timespec tv = {0};
   tv.tv_sec = msec / 1000;
   tv.tv_nsec = (msec % 1000) * 1000000;
   nanosleep(&tv, NULL);
#endif
}

#include <assert.h>
#include <limits.h>
#include <sys/time.h>
#include <unistd.h>
#include <time.h>

#define RAWFB_XRGB_8888
#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_IMPLEMENTATION
#define NK_RAWFB_IMPLEMENTATION
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_INCLUDE_SOFTWARE_FONT

#include "nuklear/nuklear.h"
#include "nuklear/nuklear_rawfb.h"

#define DTIME           20

#define UNUSED(a) (void)a
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define LEN(a) (sizeof(a)/sizeof(a)[0])

static void
die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputs("\n", stderr);
    exit(EXIT_FAILURE);
}

static long
timestamp(void)
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) < 0) return 0;
    return (long)((long)tv.tv_sec * 1000 + (long)tv.tv_usec/1000);
}

static void
sleep_for(long t)
{
    struct timespec req;
    const time_t sec = (int)(t/1000);
    const long ms = t - (sec * 1000);
    req.tv_sec = sec;
    req.tv_nsec = ms * 1000000L;
    while(-1 == nanosleep(&req, &req));
}

struct descriptor {
    int device;
    int port_min;
    int port_max;
    int index_min;
    int index_max;
    int id_min;
    int id_max;
    uint16_t *value;
};

struct remote_joypad_message {
    int port;
    int device;
    int index;
    int id;
    uint16_t state;
};

static struct retro_log_callback logger;

static retro_log_printf_t log_cb;
static retro_video_refresh_t video_cb;
static retro_audio_sample_t audio_cb;
static retro_audio_sample_batch_t audio_batch_cb;
static retro_environment_t environ_cb;
static retro_input_poll_t input_poll_cb;
static retro_input_state_t input_state_cb;

// static uint16_t *frame_buf;

static struct descriptor *desc;
static int size;
static int i;
static long dt;
static long started;
static int status;
static struct rawfb_context *rawfb;
static void *fb = NULL;
static unsigned char tex_scratch[512 * 512];

static struct descriptor joypad = {
    .device = RETRO_DEVICE_JOYPAD,
    .port_min = 0,
    .port_max = 0,
    .index_min = 0,
    .index_max = 0,
    .id_min = RETRO_DEVICE_ID_JOYPAD_B,
    .id_max = RETRO_DEVICE_ID_JOYPAD_R3
};

static struct descriptor analog = {
   .device = RETRO_DEVICE_ANALOG,
   .port_min = 0,
   .port_max = 0,
   .index_min = RETRO_DEVICE_INDEX_ANALOG_LEFT,
   .index_max = RETRO_DEVICE_INDEX_ANALOG_RIGHT,
   .id_min = RETRO_DEVICE_ID_ANALOG_X,
   .id_max = RETRO_DEVICE_ID_ANALOG_Y
};

static struct descriptor *descriptors[] = {
   &joypad,
   &analog
};

void retro_init(void)
{
    fb = malloc(WINDOW_HEIGHT * WINDOW_WIDTH * 4);

    /* GUI */
    rawfb = nk_rawfb_init(fb, tex_scratch, WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_WIDTH * 4);

   /* Allocate descriptor values */
   for (i = 0; i < ARRAY_SIZE(descriptors); i++) {
      desc = descriptors[i];
      size = DESC_NUM_PORTS(desc) * DESC_NUM_INDICES(desc) * DESC_NUM_IDS(desc);
      descriptors[i]->value = (uint16_t*)calloc(size, sizeof(uint16_t));
   }
}

void retro_deinit(void)
{
    nk_rawfb_shutdown(rawfb);

    int i;
    /* Free descriptor values */
    for (i = 0; i < ARRAY_SIZE(descriptors); i++) {
        free(descriptors[i]->value);
        descriptors[i]->value = NULL;
    }
}

unsigned retro_api_version(void)
{
   return RETRO_API_VERSION;
}

void retro_set_controller_port_device(
      unsigned port, unsigned device)
{
   (void)port;
   (void)device;
}

void retro_get_system_info(
      struct retro_system_info *info)
{
   memset(info, 0, sizeof(*info));
   info->library_name     = "Nuklear libretro test";
   info->library_version  = "0.01";
   info->need_fullpath    = false;
   info->valid_extensions = ""; /* Nothing. */
}

void retro_get_system_av_info(
      struct retro_system_av_info *info)
{
   info->timing.fps = 60.0;
   info->timing.sample_rate = 30000.0;

   info->geometry.base_width  = WINDOW_WIDTH;
   info->geometry.base_height = WINDOW_HEIGHT;
   info->geometry.max_width   = WINDOW_WIDTH;
   info->geometry.max_height  = WINDOW_HEIGHT;
   info->geometry.aspect_ratio = (double)WINDOW_WIDTH / (double)WINDOW_HEIGHT;
}

void retro_set_environment(retro_environment_t cb)
{
   static const struct retro_variable vars[] = {
      { NULL, NULL },
   };
   enum retro_pixel_format fmt = RETRO_PIXEL_FORMAT_XRGB8888;
   cb(RETRO_ENVIRONMENT_SET_VARIABLES, (void*)vars);


   environ_cb = cb;
   bool no_content = true;
   cb(RETRO_ENVIRONMENT_SET_SUPPORT_NO_GAME, &no_content);

   environ_cb(RETRO_ENVIRONMENT_SET_PIXEL_FORMAT, &fmt);

   if (cb(RETRO_ENVIRONMENT_GET_LOG_INTERFACE, &logger))
      log_cb = logger.log;
}

static void netretropad_check_variables(void)
{
}

void retro_set_audio_sample(retro_audio_sample_t cb)
{
   audio_cb = cb;
}

void retro_set_audio_sample_batch(
      retro_audio_sample_batch_t cb)
{
   audio_batch_cb = cb;
}

void retro_set_input_poll(retro_input_poll_t cb)
{
   input_poll_cb = cb;
}

void retro_set_input_state(retro_input_state_t cb)
{
   input_state_cb = cb;
}

void retro_set_video_refresh(retro_video_refresh_t cb)
{
   video_cb = cb;
}

void retro_reset(void)
{}

static void retropad_update_input(void)
{
   struct descriptor *desc;
   struct remote_joypad_message msg;
   uint16_t state;
   uint16_t old;
   int offset;
   int port;
   int index;
   int id;
   int i;

   /* Poll input */
   input_poll_cb();

   /* Parse descriptors */
   for (i = 0; i < ARRAY_SIZE(descriptors); i++) {
      /* Get current descriptor */
      desc = descriptors[i];

      /* Go through range of ports/indices/IDs */
      for (port = desc->port_min; port <= desc->port_max; port++)
         for (index = desc->index_min; index <= desc->index_max; index++)
            for (id = desc->id_min; id <= desc->id_max; id++) {
               /* Compute offset into array */
               offset = DESC_OFFSET(desc, port, index, id);

               /* Get old state */
               old = desc->value[offset];

               /* Get new state */
               state = input_state_cb(
                  port,
                  desc->device,
                  index,
                  id);

               /* Continue if state is unchanged */
               if (state == old)
                  continue;

               /* Update state */
               desc->value[offset] = state;

               /* Attempt to send updated state */
               msg.port = port;
               msg.device = desc->device;
               msg.index = index;
               msg.id = id;
               msg.state = state;
            }
   }
}

void retro_run(void)
{
    unsigned rle, runs;
    uint16_t *pixel      = NULL;
    unsigned input_state = 0;
    int offset;
    int i;

    /* Update input states and send them if needed */
    retropad_update_input();

    /* Combine RetroPad input states into one value */
    for (i = joypad.id_min; i <= joypad.id_max; i++) {
        offset = DESC_OFFSET(&joypad, 0, 0, i);
        if (joypad.value[offset])
            input_state |= 1 << i;
    }

    /* Input */
    nk_input_begin(&rawfb->ctx);
    nk_input_end(&rawfb->ctx);

    /* GUI */
    if (nk_begin(&rawfb->ctx, "Demo", nk_rect(50, 50, 200, 200),
        NK_WINDOW_BORDER|NK_WINDOW_MOVABLE|
        NK_WINDOW_CLOSABLE|NK_WINDOW_MINIMIZABLE|NK_WINDOW_TITLE)) {
        enum {EASY, HARD};
        static int op = EASY;
        static int property = 20;

        nk_layout_row_static(&rawfb->ctx, 30, 80, 1);
        if (nk_button_label(&rawfb->ctx, "button"))
            fprintf(stdout, "button pressed\n");
        nk_layout_row_dynamic(&rawfb->ctx, 30, 2);
        if (nk_option_label(&rawfb->ctx, "easy", op == EASY)) op = EASY;
        if (nk_option_label(&rawfb->ctx, "hard", op == HARD)) op = HARD;
        nk_layout_row_dynamic(&rawfb->ctx, 25, 1);
        nk_property_int(&rawfb->ctx, "Compression:", 0, &property, 100, 10, 1);
    }
    nk_end(&rawfb->ctx);

    /* -------------- EXAMPLES ---------------- */
    #ifdef INCLUDE_CALCULATOR
        calculator(ctx);
    #endif
    #ifdef INCLUDE_OVERVIEW
        overview(ctx);
    #endif
    #ifdef INCLUDE_NODE_EDITOR
        node_editor(ctx);
    #endif
    /* ----------------------------------------- */

    /* Draw framebuffer */
    nk_rawfb_render(rawfb, nk_rgb(30,30,30), 1);

    video_cb(fb, WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_WIDTH * 4);

    retro_sleep(4);
}

bool retro_load_game(const struct retro_game_info *info)
{
   netretropad_check_variables();

   return true;
}

void retro_unload_game(void)
{}

unsigned retro_get_region(void)
{
   return RETRO_REGION_NTSC;
}

bool retro_load_game_special(unsigned type,
      const struct retro_game_info *info, size_t num)
{
   (void)type;
   (void)info;
   (void)num;
   return false;
}

size_t retro_serialize_size(void)
{
   return 0;
}

bool retro_serialize(void *data, size_t size)
{
   (void)data;
   (void)size;
   return false;
}

bool retro_unserialize(const void *data,
      size_t size)
{
   (void)data;
   (void)size;
   return false;
}

void *retro_get_memory_data(unsigned id)
{
   (void)id;
   return NULL;
}

size_t retro_get_memory_size(unsigned id)
{
   (void)id;
   return 0;
}

void retro_cheat_reset(void)
{}

void retro_cheat_set(unsigned idx,
      bool enabled, const char *code)
{
   (void)idx;
   (void)enabled;
   (void)code;
}
