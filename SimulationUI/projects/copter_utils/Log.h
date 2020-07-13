#include "sstream"
#include "fstream"
#include "CopterUtils.h"

namespace copter_utils {

class Log {
public:
    Log(std::string log_file_directory);
    ~Log();

    void Buffer(int data);
    void Buffer(double data);
    void Buffer(std::string data);
    void Buffer(Eigen::VectorXd data);

    void Flush();

private:
    std::string _log_file_name;
    std::ofstream _fout;
    std::stringstream _log_buffer;
};

}