#include "Log.h"

namespace copter_utils {
    Log::Log(std::string log_file_directory) {
        // generate log file name
        _log_file_name = log_file_directory + SystemTimeStamp() + ".flightlog";

        _fout.open(_log_file_name);

        _log_buffer.clear();
    }

   Log::~Log() {
       _fout.close();
   }

    void Log::Buffer(int data) {
        _log_buffer << data << " ";
    }

    void Log::Buffer(double data) {
        _log_buffer << data << " ";
    }

    void Log::Buffer(std::string data) {
        _log_buffer << data << " ";
    }

    void Log::Buffer(Eigen::VectorXd data) {
        for (int i = 0;i < data.size();++i)
            _log_buffer << data[i] << " ";
    }

    void Log::Flush() {
        _fout << _log_buffer.str() << std::endl;
        _log_buffer.str("");
    }
}