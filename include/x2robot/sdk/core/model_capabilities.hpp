#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace x2robot::sdk::client {
// EX002-CAP-v1 只描述公开合同，不代表当前数据源或控制状态就绪。
std::string ModelNameFromType(int model);

// 设备自报型号字符串（System.GetStaticInfo 的 model 字段，来源
// /application/request_robot_static_info）→ 客户端内部档案名
// （quanta_x1 / quanta_x2 / quanta_x3 / desktop / quanta_x1_master）。
//
// 为什么不能只看 GetModelType：服务端把内部档案折叠成公开产品族
// （desktop、ex001_master、cx001、ex001 都报 QUANTA_X1_PRO），只看枚举会把
// desktop 当成 quanta_x1，客户端随之暴露 head/lift 等并不存在的能力，采集时
// 直接撞 UNIMPLEMENTED。认不出来（空串/未知型号）返回空串，调用方回退
// ModelNameFromType。
std::string ModelKeyFromDeviceName(std::string_view device_model);
bool Ex002SupportsService(std::string_view service);
bool Ex002SupportsRpc(std::string_view service, std::string_view rpc);
std::vector<std::string> Ex002Services();
}  // namespace x2robot::sdk::client
