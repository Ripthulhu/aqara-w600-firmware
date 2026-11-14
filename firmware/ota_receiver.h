#ifndef W600_OTA_RECEIVER_H
#define W600_OTA_RECEIVER_H
#include "../replacement-control/w600_frame.h"

enum { W600_OTA_STAGE=0x08010800u, W600_OTA_MAX_IMAGE=0xce00u,
       W600_OTA_PAGE=512, W600_OTA_CHUNK=224,
       W600_OTA_REPLY_MS=6000, W600_OTA_TOTAL_MS=600000,
       W600_OTA_MAX_DELAY_MS=60000 };
enum w600_ota_state {
    W600_OTA_IDLE, W600_OTA_START_ACK, W600_OTA_HEADER,
    W600_OTA_BODY, W600_OTA_FINISH, W600_OTA_DELAY,
    W600_OTA_COMMITTED, W600_OTA_FAILED
};
enum w600_ota_error {
    W600_OTA_OK, W600_OTA_FORMAT, W600_OTA_BUSY, W600_OTA_STOP,
    W600_OTA_HEADER_ERROR, W600_OTA_BOUNDS, W600_OTA_STORAGE,
    W600_OTA_CRC, W600_OTA_VECTOR, W600_OTA_AUTH,
    W600_OTA_TIMEOUT, W600_OTA_CLOCK, W600_OTA_RADIO_ABORT,
    W600_OTA_COMMIT
};
struct w600_ota_ops {
    bool (*quiesce)(void *);
    bool (*write_page)(void *,uint32_t address,const uint32_t image[128]);
    bool (*read)(void *,uint32_t address,void *out,size_t length);
    bool (*authorize)(void *,uint32_t address,uint32_t length,uint32_t revision);
    /* Must verify duplicate descriptors, then write the activation marker last.
     * Does not reset: the application owns any subsequent restart. */
    bool (*commit)(void *,uint32_t length,uint16_t crc,uint32_t revision);
};
struct w600_ota {
    const struct w600_ota_ops *ops;
    void *context;
    struct w600_frame tx;
    uint32_t page[128];
    uint32_t image_length,revision,received,page_base;
    uint32_t started_at,last_now,sent_at,delay_at,delay_ms;
    uint16_t image_crc,page_used;
    enum w600_ota_state state;
    enum w600_ota_error error;
    uint8_t retries,expected_length,start_sequence;
    bool pending,waiting;
};
void w600_ota_init(struct w600_ota *,const struct w600_ota_ops *,void *context);
/* Intercepts only dedicated OTA records. Returns true when consumed. One
 * outstanding TX is permitted; caller must deliver tx unchanged, then call
 * sent only after UART transmission completes. Process never resets hardware. */
bool w600_ota_process(struct w600_ota *,const struct w600_frame *,uint32_t now,
                     uint8_t *next_sequence);
/* Advances deadlines and prepares tx when pending. New requests allocate a
 * sequence once; retries keep it. All timing constants above are custom ms,
 * not a claim about the stock firmware's timer tick duration. */
void w600_ota_poll(struct w600_ota *,uint32_t now,uint8_t *next_sequence);
void w600_ota_sent(struct w600_ota *,uint32_t now);
bool w600_ota_active(const struct w600_ota *);
/* Failed transfers reboot only after the failure frame drains, with a bounded
 * fallback if transport is lost. Caller must independently check flash/motor safety. */
bool w600_ota_restart_ready(const struct w600_ota *,uint32_t now);
/* Replay this acknowledgement separately from the outstanding OTA request;
 * do not call sent() for it or alter the current request's retry deadline. */
bool w600_ota_duplicate_start(const struct w600_ota *,const struct w600_frame *);
#endif
