//
// Created by Sammy Guo on 3/20/22.
//

#include <chrono>

#include "DataLogger.h"

// TODO Add #ifdefs to override this function for platform-specific timekeeping
double datalogger_clock_get()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const double fp_ms = std::chrono::duration<double, std::milli>(now).count();
    return fp_ms * 1e-3;
}


DataLogger::DataLogger(
        const std::string& file_path,
        DataLogger::DataFields data_fields)
    : m_fstream(),
      m_init_timestamp(datalogger_clock_get()),
      m_data_fields(std::move(data_fields))
{
    // Populate the backing datastores
    for (const auto& field : m_data_fields)
    {
        const auto& key = field.first;
        switch (field.second) {
            case UINT8:
                m_uint8_data[key] = 0;
                break;
            case INT32:
                m_int32_data[key] = 0;
                break;
            case UINT32:
                m_uint32_data[key] = 0;
                break;
            case FLOAT32:
                m_float32_data[key] = 0.0;
                break;
            case FLOAT64:
                m_float64_data[key] = 0.0;
                break;
        }
    }

    // Open the datafile
    m_fstream.open(file_path, std::ios::out | std::ios::binary);
    if (!m_fstream) {
        return;
    }

    // Write the header
    // 1) Write a uint32 specifying how many fields
    uint32_t num_fields = m_data_fields.size();
    m_fstream.write(reinterpret_cast<const char*>(&num_fields), sizeof(num_fields));

    // 2) Then for each field, in alphabetical order...
    for (const auto& field : m_data_fields)
    {
        // 2.1) Write the uint8 enum of the datatype
        uint8_t type_enum = field.second;
        m_fstream.write(reinterpret_cast<const char*>(&type_enum), sizeof(type_enum));

        // 2.2) Write the uint32 num chars length of the field key
        const auto& field_key = field.first;
        uint32_t key_length = field_key.length();
        m_fstream.write(reinterpret_cast<const char*>(&key_length), sizeof(key_length));

        // 2.3) Write the chars of the field name
        m_fstream.write(field_key.data(), key_length);
    }

    m_fstream.flush();
}

DataLogger::~DataLogger()
{
    if (m_fstream.is_open()) {
        m_fstream.close();
    }
}

void DataLogger::publish()
{
    // Fallthrough if stream not in good health
    if (!m_fstream) {
        return;
    }

    // Record the timestamp
    const double timestamp = datalogger_clock_get() - m_init_timestamp;
    m_fstream.write(reinterpret_cast<const char*>(&timestamp), sizeof(timestamp));

    // Write out the stream
    for (const auto& field : m_data_fields)
    {
        const auto& key = field.first;
        const char* data = nullptr;
        unsigned int size = 0;
        switch (field.second)
        {
            case UINT8:
                data = reinterpret_cast<const char*>(&m_uint8_data[key]);
                size = sizeof(uint8_t);
                break;
            case INT32:
                data = reinterpret_cast<const char*>(&m_int32_data[key]);
                size = sizeof(int32_t);
                break;
            case UINT32:
                data = reinterpret_cast<const char*>(&m_uint32_data[key]);
                size = sizeof(uint32_t);
                break;
            case FLOAT32:
                data = reinterpret_cast<const char*>(&m_float32_data[key]);
                size = sizeof(float);
                break;
            case FLOAT64:
                data = reinterpret_cast<const char*>(&m_float64_data[key]);
                size = sizeof(double);
                break;
        }
        m_fstream.write(data, size);
    }
    m_fstream.flush();
}