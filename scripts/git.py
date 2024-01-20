# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import sys
import os

# 
# git commit -m $1
# git push





if __name__ == "__main__":
    if len(sys.argv) == 2:
        commit_message=sys.argv[1]
        print(commit_message)
    else:
        print("未输入commit message")
        sys.exit(1)

    os.system('git add .')
    os.system('git commit -m "commit_message"')
    os.system('git log --oneline -3')

# fix(controller): 修复用户登录时的数据验证错误
    
# 用户在尝试登录时，如果输入的数据不符合预期格式，系统将抛出异常。
# 原因是由于在处理用户输入时未进行正确的数据验证。
# 本次提交添加了对用户输入数据的验证，并优化了相关的错误处理。
    
# Fixes #123