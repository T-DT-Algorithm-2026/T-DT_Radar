# 项目开发规范

## 代码规范

- 在保证代码清晰、易读的前提下，一个分号结束的语句尽量写在一行。
- 括号内的内容尽量写在一行，避免不必要的换行。
- 变量名应简单、清晰且易于理解，避免含义不明的缩写和过长的命名。
- 修改代码时加入少量必要的注释，用于说明关键逻辑、修改原因或不直观的处理；不要添加重复代码含义的冗余注释。
- 修改时尽量遵循当前文件已有的代码风格，避免与任务无关的格式调整。
- 尽量不要使用 ROS 参数。

### C++ 控制流格式

- 所有控制流语句都必须使用大括号，包括 `if`、`else`、`else if`、`for`、范围 `for`、`while`、`do-while`、`switch` 和 `try-catch`。
- 即使控制流语句的主体只有一条语句，也禁止省略大括号。
- 禁止将控制流条件和主体写在同一行。
- 左大括号必须单独换行，采用 Allman 风格。
- 控制流主体统一缩进 4 个空格，不使用制表符。
- `else`、`catch` 等关键字应单独换行，并与对应的右大括号对齐。
- 新增代码必须遵循该规则；修改已有代码时，若触及附近未加大括号的控制流语句，应一并改为本规范。
- 不得因为当前文件中存在旧写法而继续沿用省略大括号或单行控制流写法。

正确写法：

```cpp
if (compress_thread_.joinable())
{
    compress_thread_.join();
}
```

```cpp
if (ready)
{
    process();
}
else
{
    stop();
}
```

```cpp
for (const auto & item : items)
{
    handle(item);
}
```

禁止写法：

```cpp
if (compress_thread_.joinable()) compress_thread_.join();
```

```cpp
if (compress_thread_.joinable())
    compress_thread_.join();
```

```cpp
if (ready) {
    process();
`    }
    ```

    ## 编译规范

    - 每次编译都必须在项目根目录执行：`./build.sh`
    - 不要使用其他命令替代 `./build.sh` 进行常规编译。
    - 修改完成后，应使用 `./build.sh` 编译并确认结果；若编译失败，应说明失败原因和相关错误。`
