#pragma once

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <memory.h>

#include <optional>

constexpr uint8_t AHT21_DEFAULT_ADDRESS = 0x38;   // SENSOR ADDRESS

constexpr uint8_t AHT21_MEASURE_REG = 0xAC;

constexpr uint8_t AHT21_MEASURE_DATA[2] = { 0x33, 0x00 };

constexpr uint8_t AHT21_CALIBRATED_STATUS = 0x18;

constexpr uint8_t AHT21_STATUS_REG = 0x71;        // Status register

constexpr int32_t AHT21_POWER_ON_DELAY = 100;

constexpr int32_t AHT21_MEASUREMENT_DELAY = 80;

constexpr uint8_t AHT21_STATUS_BIT_BUSY = 0x80;
constexpr uint8_t AHT21_STATUS_BIT_CAL_ENABLE = 0x08;

namespace tadragon {

    class ii2c {
        virtual int8_t read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t length) = 0;
        virtual int8_t write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, uint16_t length) = 0;
    };

    class i2c : public ii2c {
    public:
        i2c(const char* dev_name) : dev_name_{ dev_name } {
        }
        /*! I2Cスレーブデバイスからデータを読み込む.
         * @param[in] dev_addr デバイスアドレス.
         * @param[in] reg_addr レジスタアドレス.
         * @param[out] data 読み込むデータの格納場所を指すポインタ.
         * @param[in] length 読み込むデータの長さ.
         * @note https://qiita.com/nhiro/items/feb91561a6752144af93
         */
        int8_t read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint16_t length) {
            /* I2Cデバイスをオープンする. */
            int32_t fd = open(dev_name_.c_str(), O_RDWR);
            if (fd == -1) {
                return -1;
            }

            /* I2C-Readメッセージを作成する. */
            struct i2c_msg messages[] = {
                { dev_addr, 0, 1, &reg_addr },         /* レジスタアドレスをセット. */
                { dev_addr, I2C_M_RD, length, data },  /* dataにlengthバイト読み込む. */
            };

            struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };

            /* I2C-Readを行う. */
            if (ioctl(fd, I2C_RDWR, &ioctl_data) != 2) {
                close(fd);
                return -1;
            }

            close(fd);
            return 0;
        }

        /*! I2Cスレーブデバイスにデータを書き込む.
         * @param[in] dev_addr デバイスアドレス.
         * @param[in] reg_addr レジスタアドレス.
         * @param[in] data 書き込むデータの格納場所を指すポインタ.
         * @param[in] length 書き込むデータの長さ.
         * @note https://qiita.com/nhiro/items/feb91561a6752144af93
         */
        int8_t write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, uint16_t length) {
            /* I2Cデバイスをオープンする. */
            int32_t fd = open(dev_name_.c_str(), O_RDWR);
            if (fd == -1) {
                return -1;
            }

            /* I2C-Write用のバッファを準備する. */
            uint8_t* buffer = (uint8_t*)malloc(length + 1);
            if (buffer == NULL) {
                close(fd);
                return -1;
            }

            buffer[0] = reg_addr;

            if (data != nullptr && length != 0) {
                memcpy(&buffer[1], data, length);  /* 2バイト目以降にデータをセット. */
            }

            /* I2C-Writeメッセージを作成する. */
            struct i2c_msg message = { dev_addr, 0,  static_cast<uint16_t>(length + 1) , buffer };
            struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };

            /* I2C-Writeを行う. */
            if (ioctl(fd, I2C_RDWR, &ioctl_data) != 1) {
                free(buffer);
                close(fd);
                return -1;
            }

            free(buffer);
            close(fd);
            return 0;
        }
    private:
        std::string dev_name_;
    };

    class PiAHT21 {
    public:
        PiAHT21(tadragon::i2c& i2c, uint8_t addr = AHT21_DEFAULT_ADDRESS) : i2c_{ i2c }, addr_{ addr } {
        }

        std::optional<uint8_t> read_status() {
            uint8_t buf[128] = { 0 };

            if (i2c_.write(addr_, AHT21_STATUS_REG, nullptr, 0) == -1) {
                return std::nullopt;
            }

            usleep(AHT21_MEASUREMENT_DELAY * 1000);

            if (i2c_.read(addr_, AHT21_STATUS_REG, buf, 1) == -1) {
                return std::nullopt;
            }
            return buf[0];
        }

        std::optional<uint8_t> read_register(uint8_t reg) {
            uint8_t buf[128] = { 0 };

            if (i2c_.write(addr_, reg, &reg, 1) == -1) {
                return std::nullopt;
            }

            usleep(AHT21_MEASUREMENT_DELAY * 1000);

            if (i2c_.read(addr_, reg, buf, 1) == -1) {
                return std::nullopt;
            }
            return buf[0];
        }

        bool read_measurement(float* humi, float* tempature) {

            uint8_t buf[16] = { 0 };

            if (i2c_.write(addr_, AHT21_MEASURE_REG, AHT21_MEASURE_DATA, sizeof(AHT21_MEASURE_DATA)) == -1) {
                return false;
            }

            uint8_t b = 0;
            do {
                usleep(AHT21_MEASUREMENT_DELAY * 1000);
                const auto s = read_status();

                if (s) {
                    b = s.value();
                }
                else {
                    return false;
                }
                // busyなら、もう一度readする
            } while (b & AHT21_STATUS_BIT_BUSY);

            // BUSY 解除後にreadする
            if (i2c_.read(addr_, AHT21_MEASURE_REG, buf, 6) == -1) {
                return false;
            }

            if (humi) {
                int32_t value = ((buf[1] << 16) | (buf[2] << 8) | buf[3]) >> 4;
                *humi = static_cast<float>((static_cast<float>(value) * 100.f) / 1048576); // 1048576は2の20乗
            }
            if (tempature) {
                int32_t value = ((buf[3] & 0x0F) << 16) | (buf[4] << 8) | buf[5];
                *tempature = static_cast<float>(((200.f * static_cast<float>(value)) / 1048576) - 50); // 1048576は2の20乗
            }
            return true;
        }

    private:
        tadragon::i2c& i2c_;
        uint8_t addr_;
    };
}


