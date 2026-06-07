#include <xc.h>
#include "base.h"
#include "sensors.h"
#include "capacity.h"
#include "oled.h"

#define OLED_I2C_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_TEXT_CHARS 21

static uint8_t oled_available = 0;

static void I2C_WaitIdle(void) {
    uint16_t timeout = 60000;
    while (((SSPCON2 & 0x1F) || SSPSTATbits.R_nW) && --timeout);
}

static uint8_t I2C_Start(void) {
    uint16_t timeout = 60000;

    I2C_WaitIdle();
    SSPCON2bits.SEN = 1;
    while (SSPCON2bits.SEN && --timeout);
    return timeout > 0;
}

static uint8_t I2C_Stop(void) {
    uint16_t timeout = 60000;

    I2C_WaitIdle();
    SSPCON2bits.PEN = 1;
    while (SSPCON2bits.PEN && --timeout);
    return timeout > 0;
}

static uint8_t I2C_Write(uint8_t data) {
    uint16_t timeout = 60000;

    I2C_WaitIdle();
    PIR1bits.SSPIF = 0;
    SSPBUF = data;
    while (!PIR1bits.SSPIF && --timeout);
    PIR1bits.SSPIF = 0;

    if (timeout == 0)
        return 0;

    return !SSPCON2bits.ACKSTAT;
}

static uint8_t OLED_StartWrite(uint8_t control) {
    if (!I2C_Start())
        return 0;
    if (!I2C_Write(OLED_I2C_ADDR << 1))
        return 0;
    return I2C_Write(control);
}

static void OLED_Command(uint8_t command) {
    if (!oled_available)
        return;

    if (!OLED_StartWrite(0x00)) {
        oled_available = 0;
        I2C_Stop();
        return;
    }

    if (!I2C_Write(command))
        oled_available = 0;
    I2C_Stop();
}

static void OLED_SetCursor(uint8_t page, uint8_t column) {
    OLED_Command(0xB0 | page);
    OLED_Command(0x00 | (column & 0x0F));
    OLED_Command(0x10 | (column >> 4));
}

static void OLED_Clear(void) {
    for (uint8_t page = 0; page < 4; page++) {
        OLED_SetCursor(page, 0);
        if (!OLED_StartWrite(0x40)) {
            oled_available = 0;
            I2C_Stop();
            return;
        }

        for (uint8_t column = 0; column < OLED_WIDTH; column++) {
            if (!I2C_Write(0x00)) {
                oled_available = 0;
                break;
            }
        }
        I2C_Stop();
    }
}

static const uint8_t *OLED_GetFont(char c) {
    static const uint8_t font[][5] = {
        {0x00, 0x00, 0x00, 0x00, 0x00}, // space
        {0x62, 0x64, 0x08, 0x13, 0x23}, // %
        {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
        {0x08, 0x08, 0x08, 0x08, 0x08}, // -
        {0x00, 0x60, 0x60, 0x00, 0x00}, // .
        {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
        {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
        {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
        {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
        {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
        {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
        {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
        {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
        {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
        {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
        {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
        {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
        {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
        {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
        {0x46, 0x49, 0x49, 0x49, 0x31}, // S
        {0x1F, 0x20, 0x40, 0x20, 0x1F}  // V
    };

    if (c == ' ')
        return font[0];
    if (c == '%')
        return font[1];
    if (c == '+')
        return font[2];
    if (c == '-')
        return font[3];
    if (c == '.')
        return font[4];
    if (c >= '0' && c <= '9')
        return font[5 + c - '0'];
    if (c == 'A')
        return font[15];
    if (c == 'C')
        return font[16];
    if (c == 'I')
        return font[17];
    if (c == 'O')
        return font[18];
    if (c == 'S')
        return font[19];
    if (c == 'V')
        return font[20];

    return font[0];
}

static void OLED_WriteTextLine(uint8_t page, const char *text) {
    OLED_SetCursor(page, 0);

    if (!OLED_StartWrite(0x40)) {
        oled_available = 0;
        I2C_Stop();
        return;
    }

    for (uint8_t i = 0; i < OLED_TEXT_CHARS; i++) {
        char c = text[i];
        const uint8_t *columns;

        if (c == '\0')
            c = ' ';

        columns = OLED_GetFont(c);
        for (uint8_t column = 0; column < 5; column++) {
            if (!I2C_Write(columns[column])) {
                oled_available = 0;
                break;
            }
        }
        if (!oled_available || !I2C_Write(0x00)) {
            oled_available = 0;
            break;
        }
    }

    I2C_Stop();
}

static void AppendChar(char *text, uint8_t *pos, char c) {
    if (*pos < OLED_TEXT_CHARS) {
        text[*pos] = c;
        (*pos)++;
    }
}

static void AppendString(char *text, uint8_t *pos, const char *value) {
    while (*value)
        AppendChar(text, pos, *value++);
}

static void AppendUnsigned(char *text, uint8_t *pos, unsigned long value) {
    char digits[10];
    uint8_t count = 0;

    do {
        digits[count++] = (char)('0' + (value % 10));
        value /= 10;
    } while (value && count < sizeof(digits));

    while (count)
        AppendChar(text, pos, digits[--count]);
}

static void FormatVoltage(char *text, uint8_t *pos, long millivolts) {
    unsigned long decivolts = (unsigned long)((millivolts + 50) / 100);

    AppendUnsigned(text, pos, decivolts / 10);
    AppendChar(text, pos, '.');
    AppendChar(text, pos, (char)('0' + (decivolts % 10)));
    AppendChar(text, pos, 'V');
}

static void FormatSoc(char *text, uint8_t *pos, int value) {
    if (value < 0)
        value = 0;
    if (value > 100)
        value = 100;

    AppendString(text, pos, "SOC ");
    AppendUnsigned(text, pos, (unsigned long)value);
    AppendChar(text, pos, '%');
}

static void FormatCurrent(char *text, uint8_t *pos, long milliamps) {
    unsigned long centiamps;

    AppendString(text, pos, "I ");
    if (milliamps < 0) {
        AppendChar(text, pos, '-');
        milliamps = -milliamps;
    } else {
        AppendChar(text, pos, '+');
    }

    centiamps = (unsigned long)((milliamps + 5) / 10);
    AppendUnsigned(text, pos, centiamps / 100);
    AppendChar(text, pos, '.');
    AppendChar(text, pos, (char)('0' + ((centiamps / 10) % 10)));
    AppendChar(text, pos, (char)('0' + (centiamps % 10)));
    AppendChar(text, pos, 'A');
}

static void ClearText(char *text) {
    for (uint8_t i = 0; i <= OLED_TEXT_CHARS; i++)
        text[i] = '\0';
}

void OLED_Init(void) {
    static const uint8_t init_commands[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x1F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
        0xA1,
        0xC8,
        0xDA, 0x02,
        0x81, 0x8F,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0xAF
    };

    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
    SSPSTAT = 0x80;
    SSPCON1 = 0x28;
    SSPCON2 = 0x00;
    SSPADD = (uint8_t)((_XTAL_FREQ / (4UL * 100000UL)) - 1);

    __delay_ms(100);

    oled_available = 1;
    for (uint8_t i = 0; i < sizeof(init_commands); i++)
        OLED_Command(init_commands[i]);

    if (oled_available)
        OLED_Clear();
}

void OLED_Update(void) {
    char row1[OLED_TEXT_CHARS + 1];
    char row2[OLED_TEXT_CHARS + 1];
    uint8_t pos;

    if (!oled_available)
        return;

    ClearText(row1);
    ClearText(row2);

    pos = 0;
    FormatVoltage(row1, &pos, batt_voltage);
    AppendString(row1, &pos, " ");
    FormatSoc(row1, &pos, soc);

    pos = 0;
    FormatCurrent(row2, &pos, batt_current);
    AppendString(row2, &pos, " S ");
    FormatVoltage(row2, &pos, system_voltage);

    OLED_WriteTextLine(0, row1);
    OLED_WriteTextLine(2, row2);
}
