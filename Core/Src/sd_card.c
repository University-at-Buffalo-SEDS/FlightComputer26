/*
 * SD card logging utilities over FileX.
 */

#include "platform.h"
#include "sd_card.h"


/* =========================
   ThreadX objects
   ========================= */

TX_THREAD g_sd_log_thread;
static tx_align ULONG g_sd_log_thread_stack[LOGGER_STACK_ULONG];

static TX_QUEUE g_sd_log_queue;
/* Queue stores pointers (ULONG per message) */
static ULONG g_sd_log_queue_storage[SD_LOG_QUEUE_DEPTH];

static TX_MUTEX g_sd_pool_mutex;

/* Optional flush request flag */
static volatile UINT g_flush_requested = 0;

typedef struct {
  char buf[SD_LOG_LINE_MAX];
  uint16_t len;
  uint8_t in_use;
} sd_line_t;

static sd_line_t g_line_pool[SD_LOG_QUEUE_DEPTH];


/* =========================
   FileX objects/state
   ========================= */

static FX_MEDIA g_sd_media;
static FX_FILE g_sd_file;

static UINT g_fx_inited = 0;
static UINT g_media_open = 0;
static UINT g_file_open = 0;
static UINT g_sd_logger_up = 0;

/* FileX requires a media buffer */
static UCHAR g_fx_media_buffer[4096];

/* Config provided by user */
static const CHAR *g_filename = "seds_log.txt";
static SdFxDriverEntry g_driver_entry = 0;
static VOID *g_driver_info = 0;


/* =========================
   Pool helpers
   ========================= */

static sd_line_t *sd_pool_alloc(void) {
  sd_line_t *out = NULL;
  tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER);
  for (unsigned i = 0; i < SD_LOG_QUEUE_DEPTH; i++) {
    if (!g_line_pool[i].in_use) {
      g_line_pool[i].in_use = 1;
      out = &g_line_pool[i];
      break;
    }
  }
  tx_mutex_put(&g_sd_pool_mutex);
  return out;
}


static void sd_pool_free(sd_line_t *p) {
  if (!p)
    return;
  tx_mutex_get(&g_sd_pool_mutex, TX_WAIT_FOREVER);
  p->in_use = 0;
  tx_mutex_put(&g_sd_pool_mutex);
}


/* =========================
   FileX helpers
   ========================= */

static UINT ensure_fx_ready(void) {
  if (!g_fx_inited) {
    fx_system_initialize();
    g_fx_inited = 1;
  }
  return FX_SUCCESS;
}


static UINT ensure_media_open(void) {
  if (g_media_open)
    return FX_SUCCESS;
  if (!g_driver_entry)
    return FX_PTR_ERROR;

  UINT st = ensure_fx_ready();
  if (st != FX_SUCCESS)
    return st;

  st = fx_media_open(&g_sd_media, "SD", g_driver_entry, g_driver_info,
                     g_fx_media_buffer, sizeof(g_fx_media_buffer));
  if (st != FX_SUCCESS)
    return st;

  g_media_open = 1;
  return FX_SUCCESS;
}

static UINT sd_drop_oldest(UINT n) {
  UINT dropped = 0;

  for (UINT i = 0; i < n; i++) {
    ULONG msg = 0;
    if (tx_queue_receive(&g_sd_log_queue, &msg, TX_NO_WAIT) != TX_SUCCESS) {
      break; // queue empty
    }

    sd_line_t *line = (sd_line_t *)(uintptr_t)msg;
    if (line) {
      sd_pool_free(line);
    }
    dropped++;
  }

  return dropped;
}


static UINT ensure_file_open(void) {
  if (g_file_open)
    return FX_SUCCESS;

  UINT st = fx_file_open(&g_sd_media, &g_sd_file, (CHAR *)g_filename,
                         FX_OPEN_FOR_WRITE);

  if (st == FX_NOT_FOUND) {
    st = fx_file_create(&g_sd_media, (CHAR *)g_filename);
    if (st != FX_SUCCESS && st != FX_ALREADY_CREATED)
      return st;

    st = fx_file_open(&g_sd_media, &g_sd_file, (CHAR *)g_filename,
                      FX_OPEN_FOR_WRITE);
  }

  if (st != FX_SUCCESS)
    return st;

  /* Seek to end for append: use FX_FILE's current size field */
  st = fx_file_seek(&g_sd_file, g_sd_file.fx_file_current_file_size);
  if (st != FX_SUCCESS) {
    fx_file_close(&g_sd_file);
    return st;
  }

  g_file_open = 1;
  return FX_SUCCESS;
}

static VOID sd_close_all(void) {
  if (g_file_open) {
    fx_file_close(&g_sd_file);
    g_file_open = 0;
  }
  if (g_media_open) {
    fx_media_close(&g_sd_media);
    g_media_open = 0;
  }
}


/* =========================
   SD writer thread
   ========================= */

static VOID sd_log_thread_entry(ULONG arg) {
  (void)arg;

  const UINT flush_every_n = 10;
  UINT flush_ctr = 0;

  for (;;) {
    ULONG msg = 0;
    if (tx_queue_receive(&g_sd_log_queue, &msg, TX_WAIT_FOREVER) !=
        TX_SUCCESS) {
      continue;
    }

    sd_line_t *line = (sd_line_t *)(uintptr_t)msg;
    if (!line)
      continue;

    UINT st = ensure_media_open();
    if (st != FX_SUCCESS) {
      sd_close_all();
      sd_pool_free(line);
      continue;
    }

    st = ensure_file_open();
    if (st != FX_SUCCESS) {
      sd_close_all();
      sd_pool_free(line);
      continue;
    }

    st = fx_file_write(&g_sd_file, line->buf, line->len);
    if (st != FX_SUCCESS) {
      sd_close_all(); /* force reopen next time */
      sd_pool_free(line);
      continue;
    }

    /* periodic flush + user-request flush */
    flush_ctr++;
    if (g_flush_requested || flush_ctr >= flush_every_n) {
      (void)fx_media_flush(&g_sd_media);
      flush_ctr = 0;
      g_flush_requested = 0;
    }

    sd_pool_free(line);
  }
}


/* =========================
   Public API
   ========================= */

UINT sd_logger_init(const CHAR *filename, SdFxDriverEntry driver_entry,
                    VOID *driver_info) {
  if (g_sd_logger_up) {
    return FX_ALREADY_CREATED;
  }
  if (!filename || !driver_entry)
    return FX_PTR_ERROR;

  g_filename = filename;
  g_driver_entry = driver_entry;
  g_driver_info = driver_info;

  tx_mutex_create(&g_sd_pool_mutex, "sd_pool_mutex", TX_NO_INHERIT);

  tx_queue_create(&g_sd_log_queue, "sd_log_queue", TX_1_ULONG,
                  g_sd_log_queue_storage, sizeof(g_sd_log_queue_storage));

  tx_thread_create(&g_sd_log_thread, "sd_log_thread", sd_log_thread_entry, 0,
                   g_sd_log_thread_stack, LOGGER_STACK_BYTES, LOGGER_PRIORITY,
                   LOGGER_PRIORITY, LOGGER_TIME_SLICE, TX_DONT_START);

  g_sd_logger_up = 1;
  return FX_SUCCESS;
}


UINT sd_logger_enqueue_line(const CHAR *data, size_t len) {
  if (!g_sd_logger_up)
    return FX_NOT_AVAILABLE;
  if (!data)
    return FX_PTR_ERROR;

  /* These may need tuning */
  const UINT DROP_ON_PRESSURE = 1; // drop 1 oldest at a time
  const UINT MAX_RETRIES = 2;      // try a couple times then fail

  sd_line_t *rec = NULL;

  for (UINT attempt = 0; attempt < MAX_RETRIES; attempt++) {
    rec = sd_pool_alloc();
    if (rec)
      break;

    /* Pool exhausted -> relieve pressure by dropping oldest queued entry */
    sd_drop_oldest(DROP_ON_PRESSURE);
  }

  if (!rec) {
    return FX_NO_MORE_SPACE;
  }

  size_t n = len;
  if (n > (SD_LOG_LINE_MAX - 2))
    n = (SD_LOG_LINE_MAX - 2);

  memcpy(rec->buf, data, n);
  rec->buf[n++] = '\r';
  rec->buf[n++] = '\n';
  rec->len = (uint16_t)n;

  ULONG msg = (ULONG)(uintptr_t)rec;

  for (UINT attempt = 0; attempt < MAX_RETRIES; attempt++) {
    if (tx_queue_send(&g_sd_log_queue, &msg, TX_NO_WAIT) == TX_SUCCESS) {
      return FX_SUCCESS;
    }

    /* Queue full -> drop oldest and retry */
    sd_drop_oldest(DROP_ON_PRESSURE);
  }

  /* Couldn’t enqueue even after dropping -> free and report pressure */
  sd_pool_free(rec);
  return FX_NO_MORE_SPACE;
}


UINT sd_logger_request_flush(void) {
  if (!g_sd_logger_up)
    return FX_NOT_AVAILABLE;
  g_flush_requested = 1;
  return FX_SUCCESS;
}