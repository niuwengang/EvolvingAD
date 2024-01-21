# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import subprocess

# 定义您想要运行的终端命令
command = ["ls", "-l"]

# 使用subprocess.run()执行命令
result = subprocess.run(command, capture_output=True, text=True)

# 打印命令的标准输出
print(result.stdout)

# 如果有错误信息，也可以打印出来
if result.stderr:
    print("错误信息:")
    print(result.stderr)
