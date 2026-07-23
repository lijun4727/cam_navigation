#pragma once

#include <chrono>
#include <thread>
#include <functional>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <unistd.h>
#  include <limits.h>
#  if defined(__APPLE__)
#    include <mach-o/dyld.h>
#  endif
#endif

//等待函数的返回值等于指定的值，直到超时
template<typename FuncT, typename ValueT>
bool WaitForReturnVal(
    FuncT func,
    const ValueT& expectedValue,
    int msTimeout = 3000,
    int msPollInterval = 100)
{
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(msTimeout);
	auto pollInterval = std::chrono::milliseconds(msPollInterval);
    while (true) {
        if (func() == expectedValue) {
            return true;
        }
        if (std::chrono::steady_clock::now() - start > timeout) {
            return false;
        }
        std::this_thread::sleep_for(pollInterval);
    }
}

// 示例用法：
// bool success = WaitForReturnVal([&]{ return camera_->GetOption(optionType); }, value, msTimeout);

// 返回可执行文件的绝对路径（std::filesystem::path，支持中文）。
std::string GetExecutablePath();

// 可执行文件所在目录
std::string GetExecutableDirectory();

// 将 std::filesystem::path 转为 UTF-8 编码的 std::string（确保中文正确）。
std::string PathToUtf8(const std::filesystem::path& p);
