# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import sys
import os


if __name__ == "__main__":
    if len(sys.argv) == 2:
        commit_message=sys.argv[1]
        print(commit_message)
        os.system('git add .')
        os.system('git commit -m'+commit_message)
        os.system('git log --oneline -3')
    else:
        print("未输入commit message")
        sys.exit(1)
    


# <type>(<scope>): <subject>
# <BLANK LINE>
# <body>
# <BLANK LINE>
# <footer>
    
# type（必须）：提交的类型，用于说明 commit 的类别，通常有以下几种：
#     feat：新功能（feature）
#     fix：修复bug
#     docs：文档变更
#     style：代码格式（不影响代码运行的变动）
#     refactor：代码重构（不包括bug修复和功能新增）
#     test：增加测试
#     chore：构建过程或辅助工具的变动  
# scope（可选）：用于说明本次 commit 影响的范围，如数据层、控制层、视图层等。
# subject（必须）：commit 的简短描述，不超过50个字符。
# body（可选）：更详细的描述，可以分成多行。应该包括改变的动机和与之前行为的对比。
# footer（可选）：一些额外的信息，如 Breaking Changes（不兼容变动）、Closed Issues（关闭的issue）等。
    
# fix(controller): 修复用户登录时的数据验证错误
# 用户在尝试登录时，如果输入的数据不符合预期格式，系统将抛出异常。
# 原因是由于在处理用户输入时未进行正确的数据验证。
# 本次提交添加了对用户输入数据的验证，并优化了相关的错误处理。
# Fixes #123