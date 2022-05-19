//
// Created by Sammy Guo on 3/20/22.
//

#ifndef DATA_LOGGER_DATALOGGER_HPP
#define DATA_LOGGER_DATALOGGER_HPP

#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>


class DataLogger {
public:
    enum DataType : uint8_t {
        UINT8 = 1,
        INT32 = 2,
        UINT32 = 3,
        FLOAT32 = 4,
        FLOAT64 = 5
    };
    using DataFields = std::map<std::string, DataType>;

    explicit DataLogger(const std::string& file_path, DataFields data_fields);
    ~DataLogger();

    uint8_t& get_uint8(const std::string& key) { return m_uint8_data[key]; }
    int32_t& get_int32(const std::string& key) { return m_int32_data[key]; }
    uint32_t& get_uint32(const std::string& key) { return m_uint32_data[key]; }
    float& get_float32(const std::string& key) { return m_float32_data[key]; }
    double& get_float64(const std::string& key) { return m_float64_data[key]; }

    void publish();

private:
    std::ofstream m_fstream;
    double m_init_timestamp;
    DataFields m_data_fields;

    // Data storage
    std::unordered_map<std::string, uint8_t>    m_uint8_data;
    std::unordered_map<std::string, int32_t>    m_int32_data;
    std::unordered_map<std::string, uint32_t>   m_uint32_data;
    std::unordered_map<std::string, float>    m_float32_data;
    std::unordered_map<std::string, double>   m_float64_data;
};


#endif //DATA_LOGGER_DATALOGGER_HPP