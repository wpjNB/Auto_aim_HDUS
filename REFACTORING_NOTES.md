# 重构说明 (Refactoring Notes)

本文档记录了对Auto_aim_HDUS项目进行的重构和优化工作。

## 2024年重构总结

### 1. 代码质量改进

#### 1.1 内存管理
- **问题**: `Thread/thread.cpp` 中使用 `new` 创建 `Armor` 对象但从未释放
- **解决**: 改用指针引用现有对象，避免不必要的内存分配
- **影响**: 消除内存泄漏，提高性能

#### 1.2 路径管理
- **问题**: 硬编码的绝对路径（如 `/home/wpj/RM_Vision_code_US/auto_aim_HDUS/`）
- **解决**: 所有路径改为相对路径（`./config.yaml`, `./AngleSolver/XML/...`）
- **影响**: 提高项目可移植性，便于部署

#### 1.3 命名空间污染
- **问题**: 头文件中使用 `using namespace std;` 和 `using namespace cv;`
- **解决**: 移除所有头文件中的命名空间声明，使用完整命名空间
- **影响**: 避免命名冲突，提高代码可维护性

### 2. 错误处理改进

#### 2.1 配置文件验证
- **问题**: 配置文件打开失败时没有错误处理
- **解决**: 添加 `FileStorage::isOpened()` 检查和异常抛出
- **影响**: 更早发现配置错误，提供清晰的错误信息

#### 2.2 空指针检查
- **问题**: 使用 `oneArmor` 指针前没有检查是否为 nullptr
- **解决**: 添加空指针检查
- **影响**: 防止程序崩溃

#### 2.3 串口性能优化
- **问题**: 串口离线时busy-wait导致CPU使用率高
- **解决**: 添加明确的延迟常量和说明注释
- **影响**: 降低CPU占用

### 3. 代码组织改进

#### 3.1 魔法数字提取
提取的常量包括：
- `MIN_ANGLE_THRESHOLD = 0.5f` - 最小角度阈值
- `MAX_DISTANCE_INITIAL = 100000.0f` - 初始最大距离
- `SERIAL_RETRY_DELAY_US = 5000` - 串口重试延迟
- `LOST_FRAME_THRESHOLD = 6` - 装甲板丢失阈值
- `ROI_SCALE_FACTOR = 4` - ROI缩放因子
- `ThreadConstants::*` - 线程相关常量

#### 3.2 函数签名改进
添加const正确性：
- `Detector::drawResults(cv::Mat &img) const`
- `Detector::showDebuginfo(cv::Mat &img, const Armor &armor) const`
- `Detector::containLight(...) const`

### 4. 构建系统改进

#### 4.1 CMakeLists.txt 现代化
- 添加项目版本信息
- 设置 C++ 标准为必需（`CMAKE_CXX_STANDARD_REQUIRED ON`）
- 禁用编译器扩展（`CMAKE_CXX_EXTENSIONS OFF`）
- 添加编译警告选项
- 改进库链接方式（使用 PRIVATE）
- 添加 RPATH 设置
- 添加构建配置摘要输出

#### 4.2 .gitignore 优化
新增忽略项：
- IDE文件（.vscode, .idea）
- 系统文件（.DS_Store, Thumbs.db）
- 临时文件（*.tmp, *.bak）
- 可执行文件（HDUS）
- CMake生成文件

### 5. 代码风格统一

#### 5.1 类型声明
- 使用 `std::thread` 而非 `thread`
- 使用 `cv::Mat` 而非 `Mat`
- 使用 `cv::Rect` 而非 `Rect`
- 统一使用 `std::string` 而非 `string`

#### 5.2 注释改进
- 修正中文注释中的错别字
- 添加更清晰的函数说明
- 为魔法数字添加解释性注释

### 6. 待改进项

以下是识别但未在此次重构中处理的问题（需要更深入的架构改动）：

1. **线程安全**: Factory类的锁机制可以改进为条件变量
2. **异常处理**: 可以添加更多的异常处理和日志记录
3. **配置验证**: 可以添加更完整的配置参数验证
4. **单元测试**: 缺少单元测试框架
5. **文档**: 需要更详细的API文档

## 如何验证改进

### 编译测试
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 静态分析
```bash
# 使用 clang-tidy
clang-tidy *.cpp -- -std=c++17

# 使用 cppcheck
cppcheck --enable=all --std=c++17 .
```

### 运行测试
```bash
# 在build目录运行
./HDUS
```

## 兼容性说明

所有改进都保持了API的向后兼容性，不需要修改现有的调用代码。配置文件格式也保持不变，只是路径从绝对路径改为相对路径。

## 贡献者

- 重构工作: GitHub Copilot
- 原始代码: wpjNB 和贡献者
