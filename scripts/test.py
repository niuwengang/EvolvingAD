# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import subprocess

commit_message = "commit_message"  # 您希望传递的提交信息
cmd = ["git", "commit", "-m", commit_message]

# 使用subprocess.run()执行命令，捕获输出并打印
result = subprocess.run(cmd, capture_output=True, text=True)

# 打印命令的标准输出
print(result.stdout)

# 如果有错误信息，也可以打印出来
if result.stderr:
    print("错误信息:")
    print(result.stderr)
