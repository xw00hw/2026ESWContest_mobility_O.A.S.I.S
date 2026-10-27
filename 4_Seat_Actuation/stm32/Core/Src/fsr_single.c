#include "fsr_single.h"
#include "fsr_board_pins.h"
#include "joint_control.h"
#ifdef OASIS_FALLBACK_CONTROL
#include "fallback_control.h"
#endif
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

/* FSR_12_SCAN v9: up to four shared MUX addresses, three ADC banks.
   Bank order A0/PA3/IN15, A1/PC0/IN10, A2/PC3_C->PC3/IN13.
   Integration fork: S0..S3=Arduino D8,D9,D10,D11; active-low shared EN=Arduino D12.
   100 Hz per selected address per bank, sequential (not simultaneous) ADC.
   EN starts HIGH. START_12 arms; PING must arrive every <3 s.
   No external EN pull-up: reset before MX_GPIO_Init remains uncontrolled. */
#define SEAT_RATE_HZ 100U
#define SEAT_SETTLE_US 500U
#define SEAT_UART_BAUD 460800U
#define SEAT_STATS_PARTS 15U
#define SEAT_BANKS 3U
#define SEAT_MAX_ACTIVE 4U
#define SEAT_WATCHDOG_MS 3000U
/* Three 810.5-cycle acquisitions need >=300 us at the configured ADC
   kernel 64 MHz / prescaler8, even before additional H7 ADC clock division. */
#define SEAT_ADC_MIN_GROUP_US 300U

enum ScanPhase { SCAN_IDLE, SCAN_SETTLING, SCAN_WAIT_SLOT, SCAN_READY };
static ADC_HandleTypeDef *adc;
static UART_HandleTypeDef *uart;
static uint32_t seq, last_stats;
static uint32_t adc_errors, uart_errors, rx_errors, skipped_cycles, overruns;
static uint32_t late_max_cycles, aborted_samples, channel_changes, adc_max_cycles;
static uint32_t period_cycles, settle_cycles, cycles_per_us;
static uint8_t calibrated, timing_ready;
static uint8_t rx_byte;
static volatile uint8_t rx_ring[128];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_fault, tx_fault;
static volatile uint8_t stats_tx_busy;
static char stats_tx_buffer[224];
static char command[64];
static uint8_t command_length, command_overflow;
static uint8_t scan_enabled, channels[SEAT_MAX_ACTIVE], channel_count, slot_index;
static uint16_t scan_mask;
static const uint32_t adc_channels[SEAT_BANKS] = {ADC_CHANNEL_15, ADC_CHANNEL_10, ADC_CHANNEL_13};
static uint8_t selected_channel = 255U;
static uint8_t stats_part = SEAT_STATS_PARTS;
static enum ScanPhase scan_phase;
static uint32_t scan_hz = SEAT_RATE_HZ;
/* Scan deadlines use wrapping DWT cycles; watchdog uses HAL milliseconds. */
static uint32_t cycle_started_at, next_slot_at, selected_at, last_alive_at;

static void force_en_high(void)
{
    HAL_GPIO_WritePin(FSR_MUX_EN_PORT, FSR_MUX_EN_PIN, GPIO_PIN_SET);
}

static void disable_outputs(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    force_en_high();
    scan_enabled = 0U;
    scan_mask = 0U;
    selected_channel = 255U;
    scan_phase = SCAN_IDLE;
    __set_PRIMASK(saved);
}

static void transmit(const char *text)
{
    /* An interrupt-driven statistics line may overlap ADC settling. Finish
       it before emitting any CSV/control line; never interleave UART rows. */
    uint32_t waiting_since = HAL_GetTick();
    while (stats_tx_busy) {
#ifdef OASIS_FALLBACK_CONTROL
        Fallback_SafetyPoll();
#endif
        if (tx_fault || (uint32_t)(HAL_GetTick() - waiting_since) >= 20U) {
            HAL_UART_AbortTransmit(uart);
            stats_tx_busy = 0U;
            uart_errors++;
            tx_fault = 1U;
            force_en_high();
            return;
        }
    }
#ifdef OASIS_FALLBACK_CONTROL
    Fallback_SafetyPoll();
#endif
    if (HAL_UART_Transmit(uart, (const uint8_t *)text,
                          (uint16_t)strlen(text), 20U) != HAL_OK) {
        uart_errors++;
        tx_fault = 1U;
        force_en_high();
    }
#ifdef OASIS_FALLBACK_CONTROL
    Fallback_SafetyPoll();
#endif
}

static uint8_t transmit_statistics(const char *text)
{
    if (stats_tx_busy || rx_fault || tx_fault) return 0U;
    size_t length = strlen(text);
    if (length >= sizeof stats_tx_buffer) return 0U;
    memcpy(stats_tx_buffer, text, length + 1U);
    stats_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(uart, (const uint8_t *)stats_tx_buffer,
                             (uint16_t)length) != HAL_OK) {
        stats_tx_busy = 0U;
        uart_errors++;
        tx_fault = 1U;
        force_en_high();
        return 0U;
    }
    return 1U;
}

static void acknowledge_disabled(void)
{
    transmit("#CONTROL,12,MASK=0,EN=1\r\n");
}

static void stop_with_error(const char *message)
{
    disable_outputs();
    Control_Fault("FSR_FAULT");
    acknowledge_disabled();
    transmit(message);
}

static uint8_t select_channel(uint8_t channel)
{
    /* Every EN LOW write is inside this short critical section. A fault
       already latched by the ISR cannot be overwritten by a late LOW write.
       A pending IRQ after unmasking immediately raises EN again. */
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (rx_fault || tx_fault || !scan_enabled || channel > 15U ||
        !(scan_mask & (1U << channel))) {
        force_en_high();
        __set_PRIMASK(saved);
        return 0U;
    }
    if (selected_channel == channel && !(FSR_MUX_EN_PORT->ODR & FSR_MUX_EN_PIN)) {
        /* A single selected channel stays continuously connected. Neither
           EN nor any address pin is rewritten between its ADC samples. */
        scan_phase = SCAN_READY;
        __set_PRIMASK(saved);
        return 1U;
    }
    force_en_high();
    FSR_MUX_WriteAddress(channel);
    HAL_GPIO_WritePin(FSR_MUX_EN_PORT, FSR_MUX_EN_PIN, GPIO_PIN_RESET);
    selected_channel = channel;
    selected_at = DWT->CYCCNT;
    channel_changes++;
    scan_phase = SCAN_SETTLING;
    __set_PRIMASK(saved);
    return 1U;
}

static void acknowledge_started(void)
{
    char line[64];
    snprintf(line, sizeof line, "#CONTROL,12,MASK=%u,EN=0\r\n", (unsigned)scan_mask);
    transmit(line);
}

static void start_scan(uint16_t mask)
{
    if (!timing_ready) {
        stop_with_error("#CONTROL_ERROR,DWT_UNAVAILABLE\r\n");
        return;
    }
    if (!calibrated) {
        stop_with_error("#CONTROL_ERROR,ADC_CALIBRATION\r\n");
        return;
    }
    if (scan_enabled && scan_mask == mask &&
        !rx_fault && !tx_fault) {
        last_alive_at = HAL_GetTick();
        acknowledge_started();
        return;
    }
    disable_outputs();
    channel_count = 0U;
    for (uint8_t channel = 0U; channel < 16U; channel++) {
        if (mask & (1U << channel)) channels[channel_count++] = channel;
    }
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    if (rx_fault || tx_fault) {
        __set_PRIMASK(saved);
        stop_with_error("#CONTROL_ERROR,UART_FAULT\r\n");
        return;
    }
    scan_mask = mask;
    scan_hz = SEAT_RATE_HZ;
    period_cycles = SystemCoreClock / SEAT_RATE_HZ;
    scan_enabled = 1U;
    slot_index = 0U;
    cycle_started_at = DWT->CYCCNT;
    last_alive_at = HAL_GetTick();
    next_slot_at = cycle_started_at;
    __set_PRIMASK(saved);
    if (!select_channel(channels[0])) {
        stop_with_error("#CONTROL_ERROR,UART_FAULT\r\n");
        return;
    }
    acknowledge_started();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *input)
{
    if (input != uart) return;
    uint16_t next = (uint16_t)((rx_head + 1U) & 127U);
    if (next == rx_tail) {
        rx_fault = 1U;
        force_en_high();
    } else {
        rx_ring[rx_head] = rx_byte;
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1U) != HAL_OK) {
        rx_fault = 1U;
        force_en_high();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *input)
{
    if (input != uart) return;
    rx_fault = 1U;
    force_en_high();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *output)
{
    if (output == uart) stats_tx_busy = 0U;
}

static uint8_t parse_decimal(const char **text, unsigned maximum, unsigned *result)
{
    const char *cursor = *text;
    if (*cursor < '0' || *cursor > '9') return 0U;
    unsigned value = 0U;
    while (*cursor >= '0' && *cursor <= '9') {
        unsigned digit = (unsigned)(*cursor++ - '0');
        /* Reject before multiplication: no integer overflow can wrap a bad
           channel/rate/mask into an accepted value. */
        if (value > maximum / 10U ||
            (value == maximum / 10U && digit > maximum % 10U)) return 0U;
        value = value * 10U + digit;
    }
    *text = cursor;
    *result = value;
    return 1U;
}

static uint8_t parse_mask(const char *text, uint16_t *mask)
{
    unsigned value;
    if (!parse_decimal(&text, 65535U, &value) || *text || value == 0U) return 0U;
    unsigned active = 0U;
    for (unsigned bits = value; bits; bits >>= 1U) active += bits & 1U;
    /* Keep the 2.5 ms minimum slot budget for three ADC samples plus UART. */
    if (active > SEAT_MAX_ACTIVE) return 0U;
    *mask = (uint16_t)value;
    return 1U;
}

static void poll_commands(void)
{
#ifdef OASIS_FALLBACK_CONTROL
    Fallback_SafetyPoll();
    unsigned byte_budget=32U;
#endif
    if (rx_fault || tx_fault) {
        Control_Fault("UART_FAULT");
        uint8_t had_rx_fault = rx_fault;
        disable_outputs();
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        HAL_UART_AbortReceive(uart);
        rx_head = rx_tail = 0U;
        command_length = command_overflow = 0U;
        rx_fault = tx_fault = 0U;
        if (HAL_UART_Receive_IT(uart, &rx_byte, 1U) != HAL_OK) rx_fault = 1U;
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        if (had_rx_fault) rx_errors++;
        acknowledge_disabled();
        transmit(had_rx_fault ? "#CONTROL_ERROR,RX_RECOVERY\r\n" :
                               "#CONTROL_ERROR,UART_TX\r\n");
        return;
    }
    while (rx_tail != rx_head) {
#ifdef OASIS_FALLBACK_CONTROL
        if (!byte_budget--) break;
        Fallback_SafetyPoll();
#endif
        char c = (char)rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & 127U);
        if (c == '\r') continue;
        if (c == '\n') {
            uint16_t mask;
            command[command_length] = '\0';
            if (command_overflow) {
                stop_with_error("#CONTROL_ERROR,INVALID_COMMAND\r\n");
            } else if (strcmp(command, "HELLO") == 0) {
                transmit("#HELLO,FSR_12_SCAN,v10,bits=12,hz=100,banks=3,max_channels=16\r\n");
            } else if (strcmp(command, "PING") == 0) {
                last_alive_at = HAL_GetTick();
                Control_Heartbeat();
                transmit("#PONG,12\r\n");
            } else if (strncmp(command, "START_12,", 9U) == 0 &&
                       parse_mask(command + 9, &mask)) {
                start_scan(mask);
            } else if (strcmp(command, "DISABLE") == 0) {
                disable_outputs();
                acknowledge_disabled();
            } else if (Control_Command(command)) {
                /* Shared UART ownership: servo replies cannot interleave FSR rows. */
            } else if (command_length) {
                stop_with_error("#CONTROL_ERROR,INVALID_COMMAND\r\n");
            }
            command_length = command_overflow = 0U;
#ifdef OASIS_FALLBACK_CONTROL
            /* At most one complete line per poll, including malformed/empty
               rows: ACK/error traffic cannot starve motor lease checks. */
            break;
#endif
        } else if ((unsigned char)c < 32U || (unsigned char)c > 126U) {
            command_overflow = 1U;
        } else if (command_length < sizeof command - 1U) {
            command[command_length++] = c;
        } else {
            command_overflow = 1U;
        }
        if (rx_fault || tx_fault) return;
    }
}

void FSR_Init(ADC_HandleTypeDef *input, UART_HandleTypeDef *output)
{
    adc = input;
    uart = output;
    seq = adc_errors = uart_errors = rx_errors = skipped_cycles = overruns = 0U;
    late_max_cycles = aborted_samples = channel_changes = adc_max_cycles = 0U;
    rx_head = rx_tail = 0U;
    rx_fault = tx_fault = command_length = command_overflow = 0U;
    stats_tx_busy = 0U;
    scan_hz = SEAT_RATE_HZ;
    disable_outputs();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycles_per_us = (SystemCoreClock + 999999U) / 1000000U;
    period_cycles = SystemCoreClock / SEAT_RATE_HZ;
    settle_cycles = cycles_per_us * SEAT_SETTLE_US;
    uint32_t before = DWT->CYCCNT;
    HAL_Delay(1U);
    timing_ready = (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) &&
                   !(DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) &&
                   DWT->CYCCNT != before && cycles_per_us != 0U;
    calibrated = HAL_ADCEx_Calibration_Start(adc, ADC_CALIB_OFFSET_LINEARITY,
                                            ADC_SINGLE_ENDED) == HAL_OK;
    char line[224];
    snprintf(line, sizeof line,
             "#BOOT,FSR_12_SCAN,v10,calibration=%u,sysclk=%" PRIu32
             ",adc_kernel=%" PRIu32 ",prescaler=8,sample_cycles=810.5,bits=12,hz=100,banks=3,dwt=%u\r\n",
             (unsigned)calibrated, HAL_RCC_GetSysClockFreq(),
             HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC), (unsigned)timing_ready);
    transmit(line);
    acknowledge_disabled();
    transmit("#COMMANDS,HELLO,START_12,DISABLE,PING;S0=PF3/D8,S1=PD15/D9,S2=PD14/D10,S3=PB5/D11,EN=PA6/D12\r\n");
    transmit("#ADC_BANKS,A0=PA3/IN15,A1=PC0/IN10,A2=PC3_C/IN13,sequential=1,max_active=4\r\n");
    transmit("seq,t_ms,channel,raw0,status0,raw1,status1,raw2,status2\r\n");
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1U) != HAL_OK) rx_fault = 1U;
    stats_part = SEAT_STATS_PARTS;
    last_stats = last_alive_at = HAL_GetTick();
}

static unsigned acquire_bank(uint8_t bank, uint32_t *raw)
{
    ADC_ChannelConfTypeDef config = {0};
    config.Channel = adc_channels[bank];
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.OffsetSignedSaturation = DISABLE;
    unsigned status = 0U;
    if (HAL_ADC_ConfigChannel(adc, &config) != HAL_OK) status = 1U;
    else if (HAL_ADC_Start(adc) != HAL_OK) status = 1U;
    else if (HAL_ADC_PollForConversion(adc, 5U) != HAL_OK) status = 2U;
    else {
        *raw = HAL_ADC_GetValue(adc);
        if (*raw > 4095U) status = 4U;
    }
    if (HAL_ADC_Stop(adc) != HAL_OK && status == 0U) status = 3U;
    return status;
}

static void acquire_selected(void)
{
    uint8_t channel = selected_channel;
    uint32_t timestamp = HAL_GetTick();
    uint32_t acquisition_started = DWT->CYCCNT;
    char raw_text[SEAT_BANKS][12] = {{0}};
    unsigned status[SEAT_BANKS] = {0};
    uint8_t any_error = 0U;
    /* All banks are acquired before any GPIO/address rewrite. The timestamp
       is the start of this sequential group, not simultaneous conversion. */
    for (uint8_t bank = 0U; bank < SEAT_BANKS; bank++) {
#ifdef OASIS_FALLBACK_CONTROL
        Fallback_SafetyPoll();
#endif
        uint32_t raw = 0U;
        if (rx_fault || tx_fault || !scan_enabled) {
            aborted_samples++;
            return;
        }
        status[bank] = calibrated ? acquire_bank(bank, &raw) : 5U;
#ifdef OASIS_FALLBACK_CONTROL
        Fallback_SafetyPoll();
#endif
        if (rx_fault || tx_fault || !scan_enabled) {
            aborted_samples++;
            return;
        }
        if (status[bank]) {
            adc_errors++;
            any_error = 1U;
        } else {
            snprintf(raw_text[bank], sizeof raw_text[bank], "%" PRIu32, raw);
        }
    }
    uint32_t acquisition_cycles = DWT->CYCCNT - acquisition_started;
    if (acquisition_cycles > adc_max_cycles) adc_max_cycles = acquisition_cycles;
    char line[128];
    snprintf(line, sizeof line, "%" PRIu32 ",%" PRIu32 ",%u,%s,%u,%s,%u,%s,%u\r\n",
             seq++, timestamp, (unsigned)channel,
             raw_text[0], status[0], raw_text[1], status[1], raw_text[2], status[2]);
    transmit(line);
    if (rx_fault || tx_fault) return;
    if (any_error) {
        stop_with_error("#CONTROL_ERROR,ADC\r\n");
        return;
    }
    slot_index++;
    if (slot_index >= channel_count) {
        slot_index = 0U;
        cycle_started_at += period_cycles;
    }
    next_slot_at = cycle_started_at + (uint32_t)slot_index * period_cycles / channel_count;
    scan_phase = SCAN_WAIT_SLOT;
}

void FSR_Poll(void)
{
    poll_commands();
    if (rx_fault || tx_fault) return;
    uint32_t now = HAL_GetTick();
    if (scan_enabled && (uint32_t)(now - last_alive_at) >= SEAT_WATCHDOG_MS) {
        stop_with_error("#CONTROL_ERROR,WATCHDOG\r\n");
        return;
    }
    if (scan_enabled) {
        uint32_t now_cycles = DWT->CYCCNT;
        if (scan_phase == SCAN_SETTLING) {
            if ((uint32_t)(now_cycles - selected_at) >= settle_cycles) acquire_selected();
        } else if (scan_phase == SCAN_READY) {
            acquire_selected();
        } else if (scan_phase == SCAN_WAIT_SLOT && (int32_t)(now_cycles - next_slot_at) >= 0) {
            uint32_t lateness = now_cycles - next_slot_at;
            if (lateness >= period_cycles) {
                uint32_t skipped = lateness / period_cycles;
                skipped_cycles += skipped;
                cycle_started_at += skipped * period_cycles;
                next_slot_at += skipped * period_cycles;
                lateness = now_cycles - next_slot_at;
            }
            if (lateness > late_max_cycles) late_max_cycles = lateness;
            if (lateness >= period_cycles / channel_count) overruns++;
            (void)select_channel(channels[slot_index]);
        }
    }
    now = HAL_GetTick();
    if (stats_part >= SEAT_STATS_PARTS && (uint32_t)(now - last_stats) >= 1000U) {
        last_stats = now;
        stats_part = 0U;
    }
    if (stats_part < SEAT_STATS_PARTS) {
        char line[96];
        switch (stats_part) {
        case 0U:
            snprintf(line, sizeof line, "#STATS,scan_mask=%u,scan_enabled=%u\r\n",
                     (unsigned)scan_mask, (unsigned)scan_enabled);
            break;
        case 1U:
            snprintf(line, sizeof line, "#STATS,selected=%u,mux_en=%u\r\n",
                     (unsigned)selected_channel, (unsigned)((FSR_MUX_EN_PORT->ODR & FSR_MUX_EN_PIN) != 0U));
            break;
        case 2U:
            snprintf(line, sizeof line, "#STATS,adc_errors=%" PRIu32 "\r\n", adc_errors);
            break;
        case 3U:
            snprintf(line, sizeof line, "#STATS,uart_errors=%" PRIu32 "\r\n", uart_errors);
            break;
        case 4U:
            snprintf(line, sizeof line, "#STATS,rx_errors=%" PRIu32 "\r\n", rx_errors);
            break;
        case 5U:
            snprintf(line, sizeof line, "#STATS,overruns=%" PRIu32 "\r\n", overruns);
            break;
        case 6U:
            snprintf(line, sizeof line, "#STATS,skipped_cycles=%" PRIu32 "\r\n", skipped_cycles);
            break;
        case 7U:
            snprintf(line, sizeof line, "#STATS,late_max_us=%" PRIu32 "\r\n",
                     cycles_per_us ? late_max_cycles / cycles_per_us : 0U);
            break;
        case 8U:
            snprintf(line, sizeof line, "#STATS,aborted=%" PRIu32 "\r\n", aborted_samples);
            break;
        case 9U:
            snprintf(line, sizeof line, "#STATS,watchdog_ms=%" PRIu32 "\r\n", (uint32_t)(now-last_alive_at));
            break;
        case 10U:
            snprintf(line, sizeof line, "#STATS,channel_changes=%" PRIu32 "\r\n", channel_changes);
            break;
        case 11U:
            snprintf(line, sizeof line, "#STATS,samples=%" PRIu32 "\r\n", seq);
            break;
        case 12U:
            snprintf(line, sizeof line, "#STATS,dwt_ready=%u\r\n", (unsigned)timing_ready);
            break;
        case 13U:
            snprintf(line, sizeof line, "#STATS,mode=12,hz=%u\r\n", (unsigned)scan_hz);
            break;
        default:
            snprintf(line, sizeof line, "#STATS,adc_max_us=%" PRIu32 "\r\n",
                     cycles_per_us ? adc_max_cycles / cycles_per_us : 0U);
            break;
        }
        /* Statistics use UART interrupts so their wire time can overlap the
           next channel's settling and three ADCs (never the next CSV line). This
           avoids starving telemetry after adding three sequential ADCs. */
        uint32_t wire_us = ((uint32_t)strlen(line) * 10000000U + SEAT_UART_BAUD-1U) / SEAT_UART_BAUD;
        uint32_t budget_cycles = (wire_us + 150U) * cycles_per_us;
        if (!scan_enabled || (scan_phase == SCAN_WAIT_SLOT &&
            (int32_t)(next_slot_at - DWT->CYCCNT) + (int32_t)settle_cycles +
            (int32_t)(SEAT_ADC_MIN_GROUP_US * cycles_per_us) >=
            (int32_t)budget_cycles)) {
            if (transmit_statistics(line)) stats_part++;
        }
    }
}

/* All control/telemetry use the same serialized UART owner. */
void FSR_Transmit(const char *text) { transmit(text); }
int FSR_TryTelemetry(const char *text)
{
    /* The 5 Hz control state must not starve behind the 400 Hz scan.
       Start between acquisitions and overlap the next settling/ADC group.
       transmit() completes this bounded line before writing the next CSV;
       any scan lateness remains visible in timestamps and scan statistics. */
    if (!scan_enabled || scan_phase == SCAN_WAIT_SLOT)
        return transmit_statistics(text);
    return 0;
}
