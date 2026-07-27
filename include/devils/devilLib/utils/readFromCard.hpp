#include <fstream>
#include <string>

namespace devils
{
    class ReadFromCard
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
    };
}