#ifndef EVNSevenSegmentLED_h
#define EVNSevenSegmentLED_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"
#include "ht16k33/EVN_HT16K33.h"

class EVNSevenSegmentLED : public EVN_HT16K33
{
public:
    enum letter : uint8_t
    {
        A = 0b01110111,
        B = 0b01111100,
        C = 0b00111001,
        D = 0b01011110,
        E = 0b01111001,
        F = 0b01110001,
        G = 0b00111101,
        H = 0b01110100,
        J = 0b00011110,
        L = 0b00111000,
        N = 0b01010100,
        O = 0b01011100,
        P = 0b01110011,
        R = 0b01010000,
        T = 0b01111000,
        U = 0b00011100,
        Y = 0b01101110,
    };

    enum number : uint8_t
    {
        N0 = 0b00111111,
        N1 = 0b00000110,
        N2 = 0b01011011,
        N3 = 0b01001111,
        N4 = 0b01100110,
        N5 = 0b01101101,
        N6 = 0b01111101,
        N7 = 0b00000111,
        N8 = 0b01111111,
        N9 = 0b01101111,
    };

    EVNSevenSegmentLED(uint8_t port) : EVN_HT16K33(port)
    {
        _addr = 0x74;
    };

    void writeDigit(uint8_t position, uint8_t digit, bool show = true)
    {
        if (position > 3)
            return;

        if (digit > 9)
            return;

        if (_sensor_started)
        {
            _buffer[_seg_lut[position]] &= 0b10000000;

            switch (digit)
            {
            case 0:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N0;
                break;
            case 1:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N1;
                break;
            case 2:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N2;
                break;
            case 3:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N3;
                break;
            case 4:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N4;
                break;
            case 5:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N5;
                break;
            case 6:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N6;
                break;
            case 7:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N7;
                break;
            case 8:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N8;
                break;
            case 9:
                _buffer[_seg_lut[position]] |= (uint8_t)number::N9;
                break;
            }

            if (show) update();
        }
    };

    void writeLetter(uint8_t position, char letter, bool show = true)
    {
        if (position > 3)
            return;

        if (_sensor_started)
        {
            _buffer[_seg_lut[position]] &= 0b10000000;

            switch (letter)
            {
            case 'a':
            case 'A':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::A;
                break;
            case 'b':
            case 'B':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::B;
                break;
            case 'c':
            case 'C':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::C;
                break;
            case 'd':
            case 'D':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::D;
                break;
            case 'e':
            case 'E':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::E;
                break;
            case 'f':
            case 'F':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::F;
                break;
            case 'g':
            case 'G':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::G;
                break;
            case 'h':
            case 'H':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::H;
                break;
            case 'j':
            case 'J':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::J;
                break;
            case 'l':
            case 'L':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::L;
                break;
            case 'n':
            case 'N':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::N;
                break;
            case 'o':
            case 'O':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::O;
                break;
            case 'p':
            case 'P':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::P;
                break;
            case 'r':
            case 'R':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::R;
                break;
            case 't':
            case 'T':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::T;
                break;
            case 'u':
            case 'U':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::U;
                break;
            case 'y':
            case 'Y':
                _buffer[_seg_lut[position]] |= (uint8_t)letter::Y;
                break;
            }

            if (show) this->update();
        }
    };

    void clearPosition(uint8_t position, bool clear_point = true, bool show = true)
    {
        if (position > 3) return;

        if (_sensor_started)
        {
            if (clear_point)
                _buffer[_seg_lut[position]] &= 0b00000000;
            else
                _buffer[_seg_lut[position]] &= 0b10000000;

            if (show) this->update();
        }
    };

    void writePoint(uint8_t position, bool on = true, bool show = true)
    {
        if (position > 3)
            return;

        if (_sensor_started)
        {
            this->writeRaw(_point_lut[position], on, show);
        }
    };

    void clearPoint(uint8_t position, bool show = true)
    {
        writePoint(position, false, show);
    };

    void writeColon(bool on = true, bool show = true)
    {
        if (_sensor_started)
        {
            this->writeRaw(_colon_index, on, show);
        }
    };

    void clearColon(bool show = true)
    {
        writeColon(false, show);
    };

    void writeNumber(float number, bool show = true)
    {
        if (_sensor_started)
        {
            float numberc = fabs(number);

            clearAll(false);

            if (numberc > 10000)
            {
                writeNumber((long)floor(numberc), show);
                return;
            }

            uint8_t digit_after_point = 4;

            if (numberc >= 1000)
            {
                writePoint(3, true);
                writeDigit(0, ((long)floor(numberc)) / 1000, false);
                writeDigit(1, ((long)floor(numberc) % 1000) / 100, false);
                writeDigit(2, ((long)floor(numberc) % 100) / 10, false);
                writeDigit(3, ((long)floor(numberc) % 10), false);
            }
            else if (numberc >= 100)
            {
                writePoint(2, true);
                writeDigit(0, ((long)floor(numberc)) / 100, false);
                writeDigit(1, ((long)floor(numberc) % 100) / 10, false);
                writeDigit(2, ((long)floor(numberc) % 10), false);
                digit_after_point = 3;
            }
            else if (numberc >= 10)
            {
                writePoint(1, true);
                writeDigit(0, ((long)floor(numberc)) / 10, false);
                writeDigit(1, ((long)floor(numberc) % 10), false);
                digit_after_point = 2;
            }
            else
            {
                writePoint(0, true);
                writeDigit(0, floor(numberc), false);
                digit_after_point = 1;
            }

            while (digit_after_point < 4)
            {
                numberc *= 10;
                writeDigit(digit_after_point, ((long)floor(numberc) % 10), false);
                digit_after_point++;
            }

            if (show) this->update();
        }
    };

    void writeNumber(double number, bool show = true)
    {
        writeNumber((float)number, show);
    };

    void writeNumber(long double number, bool show = true)
    {
        writeNumber((float)number, show);
    };

    void writeNumber(long number, bool show = true)
    {
        if (_sensor_started)
        {
            uint32_t numberc = abs(number);
            while (numberc > 9999) numberc /= 10;

            clearAll(false);
            for (int i = 3; i >= 0; i--)
            {
                if (numberc > 0) writeDigit(i, numberc % 10, false);
                numberc /= 10;
            }

            if (show) this->update();
        }
    };

    void writeNumber(unsigned int number, bool show = true)
    {
        writeNumber((long)number, show);
    };

    void writeNumber(unsigned long number, bool show = true)
    {
        writeNumber((long)number, show);
    };

    void writeNumber(int number, bool show = true)
    {
        writeNumber((long)number, show);
    };

private:
    uint8_t _point_lut[4] = { 7, 23, 55, 71 };
    uint8_t _seg_lut[4] = { 0, 2, 6, 8 };
    uint8_t _colon_index = 33;
};

#endif
