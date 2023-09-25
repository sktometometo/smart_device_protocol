#include <variant>

typedef std::variant<int32_t, float, std::string, bool> SDPData;
typedef std::tuple<std::string, std::string> SDPInterfaceDescription;