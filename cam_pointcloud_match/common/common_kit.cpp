#include "common_kit.h"

std::string GetExecutablePath()
{
#ifdef _WIN32
    std::vector<wchar_t> buf;
    buf.resize(32768);
    DWORD len = ::GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
    if (len == 0) {
        throw std::system_error(GetLastError(), std::system_category(), "GetModuleFileNameW failed");
    }
    // 若缓冲不够，len == buf.size() 但可能被截断；按安全策略返回路径截断前的内容
    return PathToUtf8(std::filesystem::path(std::wstring(buf.data(), static_cast<size_t>(len))));
#elif defined(__APPLE__)
    uint32_t size = 0;
    if (_NSGetExecutablePath(nullptr, &size) == 0) {
        // unexpected
        return "";
    }
    std::vector<char> buf(size);
    if (_NSGetExecutablePath(buf.data(), &size) != 0) {
        return "";
    }
    char resolved[PATH_MAX];
    if (realpath(buf.data(), resolved) == nullptr) {
        return "";
    }
    return PathToUtf8(std::filesystem::path(resolved));
#else
    std::vector<char> buf(4096);
    ssize_t len = ::readlink("/proc/self/exe", buf.data(), static_cast<size_t>(buf.size()));
    while (len == static_cast<ssize_t>(buf.size())) {
        buf.resize(buf.size() * 2);
        len = ::readlink("/proc/self/exe", buf.data(), static_cast<size_t>(buf.size()));
    }
    if (len <= 0) {
        return "";
    }
    return PathToUtf8(std::filesystem::path(std::string(buf.data(), static_cast<size_t>(len))));
#endif
}

std::string GetExecutableDirectory()
{
    return PathToUtf8(std::filesystem::path(GetExecutablePath()).parent_path());
}

std::string PathToUtf8(const std::filesystem::path& p)
{
#ifdef _WIN32
    std::string out;
    const std::wstring ws = p.wstring();
    if (ws.empty()) 
        return out;
    int size = ::WideCharToMultiByte(CP_UTF8, 0,
        ws.data(), static_cast<int>(ws.size()),
        nullptr, 0, nullptr, nullptr);
    if (size <= 0)
        return out;
    out.resize(static_cast<size_t>(size));
    int res = ::WideCharToMultiByte(CP_UTF8, 0,
        ws.data(), static_cast<int>(ws.size()),
        out.data(), size, nullptr, nullptr);
    return out;
#else
    // POSIX 上通常使用 UTF-8 环境，直接返回 u8string()
    return p.u8string();
#endif
}