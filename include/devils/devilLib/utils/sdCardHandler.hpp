#include <fstream>
#include <string>

namespace devils
{
    class SDCardHandler
    {
    public:
        std::string readFile(const std::string& filename)
        {
            std::ifstream file("/usd/" + filename);

            if (!file.is_open()) {
                return "";
            }

            std::string dataGrabbed(
                (std::istreambuf_iterator<char>(file)),
                std::istreambuf_iterator<char>()
            );

            file.close();
            return dataGrabbed;
        }
        
        void writeFile(const std::string& filename, const std::string& data)
        {
            std::ofstream file("/usd/" + filename);

            if (!file.is_open()) {
                return;
            }

            file << data;
            file.close();
        }
    };
}