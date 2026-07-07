#include "crt_calibration_storage.hpp"

#include "crc.h"
#include "para_gimbal.hpp"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include <cstring>

namespace
{
struct CalibrationRecord {
    uint32_t magic;
    uint32_t version;
    uint32_t offsetBits;
    uint32_t crc32;
};

static_assert(sizeof(CalibrationRecord) == 16, "Calibration record size must stay 16 bytes");

constexpr uint32_t kCalibrationFlashSector = FLASH_SECTOR_11;

uint32_t fp32ToBits(fp32 value)
{
    uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

fp32 bitsToFp32(uint32_t bits)
{
    fp32 value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

uint32_t calculateRecordCrc(const CalibrationRecord &record)
{
    uint32_t payload[3] = {record.magic, record.version, record.offsetBits};
    return HAL_CRC_Calculate(&hcrc, payload, 3);
}

CalibrationRecord makeRecord(fp32 offset)
{
    CalibrationRecord record = {
        YAW_GYRO_OFFSET_FLASH_MAGIC,
        YAW_GYRO_OFFSET_FLASH_VERSION,
        fp32ToBits(offset),
        0U};
    record.crc32 = calculateRecordCrc(record);
    return record;
}

const CalibrationRecord &getStoredRecord()
{
    return *reinterpret_cast<const CalibrationRecord *>(YAW_GYRO_OFFSET_FLASH_ADDRESS);
}

bool isRecordValid(const CalibrationRecord &record)
{
    if (record.magic != YAW_GYRO_OFFSET_FLASH_MAGIC ||
        record.version != YAW_GYRO_OFFSET_FLASH_VERSION) {
        return false;
    }

    return record.crc32 == calculateRecordCrc(record);
}

bool writeRecord(const CalibrationRecord &record)
{
    const uint32_t words[4] = {record.magic, record.version, record.offsetBits, record.crc32};
    uint32_t sectorError    = 0U;
    HAL_StatusTypeDef status;

    FLASH_EraseInitTypeDef eraseInit = {};
    eraseInit.TypeErase              = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange           = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector                 = kCalibrationFlashSector;
    eraseInit.NbSectors              = 1;

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return false;
    }

    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t address = YAW_GYRO_OFFSET_FLASH_ADDRESS;
    for (uint32_t index = 0; index < 4; ++index) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, words[index]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        address += sizeof(uint32_t);
    }

    HAL_FLASH_Lock();

    const CalibrationRecord &storedRecord = getStoredRecord();
    return std::memcmp(&storedRecord, &record, sizeof(record)) == 0;
}
} // namespace

namespace GimbalCalibrationStorage
{
bool loadYawGyroOffset(fp32 &offsetY)
{
    const CalibrationRecord &record = getStoredRecord();
    if (!isRecordValid(record)) {
        offsetY = 0.0f;
        return false;
    }

    offsetY = bitsToFp32(record.offsetBits);
    return true;
}

bool saveYawGyroOffset(fp32 offsetY)
{
    CalibrationRecord targetRecord     = makeRecord(offsetY);
    const CalibrationRecord &storedRecord = getStoredRecord();

    if (isRecordValid(storedRecord) && storedRecord.offsetBits == targetRecord.offsetBits) {
        return true;
    }

    return writeRecord(targetRecord);
}
} // namespace GimbalCalibrationStorage
