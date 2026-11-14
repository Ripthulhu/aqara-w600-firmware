#include "flash_page.h"
#include "flash_regs.h"
#include <stddef.h>
#include <stdint.h>

#ifdef FLASH_PAGE_HOST_TEST
uint32_t fp_test_read(uint32_t address);
void fp_test_write(uint32_t address, uint32_t value);
uint32_t fp_test_core(unsigned index);
uintptr_t fp_test_code(unsigned end);
#define RD(a) fp_test_read(a)
#define WR(a,v) fp_test_write(a,v)
#define CORE(n) fp_test_core(n)
#define CODE(n) fp_test_code(n)
#else
#define RD(a) (*(volatile const uint32_t *)(uintptr_t)(a))
#define WR(a,v) (*(volatile uint32_t *)(uintptr_t)(a) = (v))
extern uint8_t __flash_page_start[], __flash_page_end[];
#define CODE(n) ((uintptr_t)((n) ? __flash_page_end : __flash_page_start))
static uint32_t core(unsigned index)
{
    uint32_t value;
    switch (index) {
    case 0: __asm volatile("mrs %0, primask" : "=r"(value)); break;
    case 1: __asm volatile("mrs %0, ipsr" : "=r"(value)); break;
    case 2: __asm volatile("mrs %0, control" : "=r"(value)); break;
    case 3: __asm volatile("mrs %0, msp" : "=r"(value)); break;
    default: __asm volatile("mov %0, pc" : "=r"(value)); break;
    }
    return value;
}
#define CORE(n) core(n)
#endif

static int ram(uintptr_t address, size_t size)
{
    return !(address & 3u) && size <= 0x7000u &&
           address >= 0x20000000u && address <= 0x20007000u - size;
}

static int overlaps(uintptr_t a, size_t n, uintptr_t b, size_t m)
{
    return a < b + m && b < a + n;
}

static int private_ram(uintptr_t address, size_t size, uint32_t sp, uint32_t vectors)
{
    return ram(address, size) && !overlaps(address, size, CODE(0), CODE(1)-CODE(0)) &&
           !overlaps(address, size, sp-512u, 0x20007000u-(sp-512u)) &&
           !overlaps(address, size, vectors, 192u);
}

static uint32_t crc_word(uint32_t crc, uint32_t word)
{
    for (unsigned byte = 0; byte != 4; ++byte) {
        crc ^= word & 255u;
        word >>= 8;
        for (unsigned bit = 0; bit != 8; ++bit)
            crc = (crc >> 1) ^ ((0u-(crc & 1u)) & 0xedb88320u);
    }
    return crc;
}

static uint32_t image_crc(const uint32_t *image)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i != 128; ++i) crc = crc_word(crc, image[i]);
    return ~crc;
}

static uint32_t flash_crc(uint32_t address)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i != 128; ++i) crc = crc_word(crc, RD(address+4u*i));
    return ~crc;
}

static int motor_stopped(void)
{
    if ((RD(FP_IOPEN) & 7u) != 7u) return 0;
    return (RD(0x50000400u) & 0xf0000000u) == 0x50000000u &&
           !(RD(0x50000404u) & 0xc000u) &&
           (RD(0x50000414u) & 0xc000u) == 0xc000u &&
           (RD(0x50000410u) & 0xc000u) == 0xc000u &&
           (RD(0x50000000u) & 0xc0000000u) == 0x40000000u &&
           !(RD(0x50000004u) & 0x8000u) &&
           !(RD(0x50000014u) & 0x8000u) && !(RD(0x50000010u) & 0x8000u) &&
           (RD(0x50000800u) & 0x00300000u) == 0x00100000u &&
           !(RD(0x50000804u) & 0x400u) &&
           !(RD(0x50000814u) & 0x400u) && !(RD(0x50000810u) & 0x400u);
}

static int protected_page(uint32_t page)
{
    uint32_t start = RD(FP_WRPAS) & 511u, end = RD(FP_WRPAE) & 511u;
    if (start <= page && page <= end) return 1;
    start = RD(FP_WRPBS) & 511u; end = RD(FP_WRPBE) & 511u;
    return start <= page && page <= end;
}

static enum ciu_flash_status clear_status(void)
{
    WR(FP_SR, FP_EOP | FP_ERRORS);
    return (RD(FP_SR) & (FP_EOP | FP_ERRORS | FP_BUSY)) ?
           CIU_FLASH_CONTROLLER : CIU_FLASH_OK;
}

static enum ciu_flash_status timer_begin(void)
{
    /* Reading clears an old COUNTFLAG before issuing the new operation. */
    return (RD(FP_SYST_CTRL) & 7u) == 5u ? CIU_FLASH_OK : CIU_FLASH_CLOCK;
}

static enum ciu_flash_status wait_operation(uint32_t ticks, uint32_t cap,
                                           struct ciu_flash_report *out)
{
    uint32_t elapsed = 0;
    for (uint32_t poll = 0; poll != cap; ++poll) {
        uint32_t sr = RD(FP_SR), ctrl = RD(FP_SYST_CTRL);
        out->final_sr = sr;
        if ((ctrl & 7u) != 5u) return CIU_FLASH_CLOCK;
        if ((ctrl & 0x10000u) && ++elapsed >= ticks) return CIU_FLASH_TIMEOUT;
        if (!(sr & FP_BUSY)) {
            if (sr & FP_ERRORS) return CIU_FLASH_ERROR;
            return (sr & FP_EOP) ? CIU_FLASH_OK : CIU_FLASH_NO_EOP;
        }
    }
    return CIU_FLASH_POLL_LIMIT;
}

static enum ciu_flash_status finish(enum ciu_flash_status status, int owned,
                                     struct ciu_flash_report *out)
{
    if (RD(FP_AHBEN) & 0x100u) {
        out->final_sr = RD(FP_SR);
        out->final_cr = RD(FP_CR);
        if (owned && !(out->final_sr & FP_BUSY)) {
            /* RM forbids changing CR while BSY. Never retry an unknown write. */
            WR(FP_CR, FP_LOCK | FP_OPTLOCK);
            out->final_cr = RD(FP_CR);
        }
        out->locked = out->final_cr == (FP_LOCK | FP_OPTLOCK);
        if (status == CIU_FLASH_OK && !out->locked) status = CIU_FLASH_LOCK_FAILED;
    } else {
        out->locked = 0;
        if (status == CIU_FLASH_OK) status = CIU_FLASH_CLOCK;
    }
    out->status = status;
    return status;
}

static enum ciu_flash_status preflight(const struct ciu_flash_request *request,
                                       struct ciu_flash_report *out)
{
    uint32_t sp = CORE(3), vectors = RD(FP_VTOR);
    uintptr_t req = (uintptr_t)request, report = (uintptr_t)out;
    if (sp < 0x20000400u || sp > 0x20007000u || (sp & 7u) ||
        CODE(1) <= CODE(0) || !ram(CODE(0), CODE(1)-CODE(0)) ||
        !ram(vectors, 192u) || (vectors & 127u) ||
        !private_ram(report, sizeof(*out), sp, vectors) ||
        !private_ram(req, sizeof(*request), sp, vectors) ||
        overlaps(req, sizeof(*request), report, sizeof(*out))) return CIU_FLASH_REQUEST;
    out->status=CIU_FLASH_REQUEST; out->initial_sr=0; out->final_sr=0; out->final_cr=0;
    out->erase_issued=0; out->word_stores=0; out->words_verified=0; out->locked=0;
    out->observed_old_crc=0; out->observed_image_crc=0; out->observed_final_crc=0;
    uintptr_t image = (uintptr_t)request->image;
    uint32_t address = request->address;
    if (address < CIU_FLASH_APP_BEGIN || address > CIU_FLASH_APP_END-CIU_FLASH_PAGE_BYTES ||
        (address & 511u) || !private_ram(image, 512u, sp, vectors) ||
        overlaps(image, 512u, req, sizeof(*request)) ||
        overlaps(image, 512u, report, sizeof(*out)) ||
        !request->erase_ticks || request->erase_ticks > CIU_FLASH_MAX_ERASE_TICKS ||
        !request->program_ticks || request->program_ticks > CIU_FLASH_MAX_PROGRAM_TICKS ||
        !request->poll_limit || request->poll_limit > CIU_FLASH_MAX_POLLS)
        return CIU_FLASH_REQUEST;
    uint32_t nmi=RD(vectors+8u), fault=RD(vectors+12u);
    if (CORE(0) != 1u || CORE(1) != 0u || CORE(2) != 0u || RD(0xe000e100u) ||
        CORE(4) < CODE(0) || CORE(4) >= CODE(1) ||
        !(nmi & 1u) || nmi < 0x20000001u || nmi >= 0x20007000u ||
        !(fault & 1u) || fault < 0x20000001u || fault >= 0x20007000u) {
        out->status=CIU_FLASH_CONTEXT; return CIU_FLASH_CONTEXT;
    }
    if ((RD(FP_AHBEN) & 0x101u) != 0x100u || RD(FP_CFG) != 0x12u ||
        RD(FP_PLL) != 0x30260u || (RD(FP_CSR)&FP_READY_MASK) != FP_READY ||
        RD(FP_ACR) != 2u || RD(FP_SYST_LOAD) != 47999u ||
        (RD(FP_SYST_CTRL) & 7u) != 5u) {
        out->status=CIU_FLASH_CLOCK; return CIU_FLASH_CLOCK;
    }
    if (RD(0xe000ed00u) != 0x410cc601u || RD(0x1fff03a4u) != 2u ||
        RD(0x1fff03a8u) != 128u || RD(0x1fff03acu) != 28u) {
        out->status=CIU_FLASH_CONTEXT; return CIU_FLASH_CONTEXT;
    }
    if (!motor_stopped()) { out->status=CIU_FLASH_MOTOR; return CIU_FLASH_MOTOR; }
    out->initial_sr=RD(FP_SR); out->final_sr=out->initial_sr; out->final_cr=RD(FP_CR);
    if ((out->initial_sr & FP_BUSY) || out->final_cr != (FP_LOCK | FP_OPTLOCK) ||
        (RD(FP_OPTR) & 255u) != 0xaau) return finish(CIU_FLASH_CONTROLLER, 0, out);
    out->locked=1;
    if (protected_page((address-0x08000000u) >> 9)) return finish(CIU_FLASH_PROTECTED,0,out);
    out->observed_old_crc=flash_crc(address);
    if (out->observed_old_crc != request->expected_old_crc) return finish(CIU_FLASH_OLD_CRC,0,out);
    out->observed_image_crc=image_crc(request->image);
    if (out->observed_image_crc != request->image_crc) return finish(CIU_FLASH_IMAGE_CRC,0,out);
    out->status=CIU_FLASH_OK;
    return CIU_FLASH_OK;
}

enum ciu_flash_status ciu_flash_check_page(const struct ciu_flash_request *request,
                                          struct ciu_flash_report *out)
{
    return preflight(request,out);
}

enum ciu_flash_status ciu_flash_replace_page(const struct ciu_flash_request *request,
                                            struct ciu_flash_report *out)
{
    enum ciu_flash_status status=preflight(request,out);
    if (status != CIU_FLASH_OK) return status;
    uint32_t address=request->address;
    status=clear_status();
    if (status != CIU_FLASH_OK) return finish(status,0,out);
    WR(FP_KEY,FP_KEY1); WR(FP_KEY,FP_KEY2);
    if (RD(FP_CR) != FP_OPTLOCK) return finish(CIU_FLASH_CONTROLLER,1,out);
    uint32_t erase_control=FP_OPTLOCK | FP_PAGE_MODE | (((address-0x08000000u) >> 9) << 8);
    WR(FP_CR,erase_control);
    if (RD(FP_CR) != erase_control) return finish(CIU_FLASH_CONTROLLER,1,out);
    status=timer_begin();
    if (status != CIU_FLASH_OK) return finish(status,1,out);
    out->erase_issued=1;
    WR(FP_CR,erase_control | FP_ERASE);
    status=wait_operation(request->erase_ticks,request->poll_limit,out);
    if (status != CIU_FLASH_OK) return finish(status,1,out);
    for (unsigned i=0;i!=128;++i)
        if (RD(address+4u*i) != UINT32_MAX) return finish(CIU_FLASH_ERASE_VERIFY,1,out);
    WR(FP_CR,FP_OPTLOCK | FP_PROGRAM);
    if (RD(FP_CR) != (FP_OPTLOCK | FP_PROGRAM)) return finish(CIU_FLASH_CONTROLLER,1,out);
    for (unsigned i=0;i!=128;++i) {
        uint32_t word=request->image[i];
        if (word != UINT32_MAX) {
            status=clear_status();
            if (status != CIU_FLASH_OK) return finish(status,1,out);
            status=timer_begin();
            if (status != CIU_FLASH_OK) return finish(status,1,out);
            ++out->word_stores;
            WR(address+4u*i,word);
            status=wait_operation(request->program_ticks,request->poll_limit,out);
            if (status != CIU_FLASH_OK) return finish(status,1,out);
        }
        if (RD(address+4u*i) != word) return finish(CIU_FLASH_WORD_VERIFY,1,out);
        ++out->words_verified;
    }
    out->observed_final_crc=flash_crc(address);
    status=out->observed_final_crc == request->image_crc ? CIU_FLASH_OK : CIU_FLASH_FINAL_CRC;
    return finish(status,1,out);
}
