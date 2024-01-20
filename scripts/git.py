# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import sys
import os

# 
# git commit -m $1
# git push
# git log --oneline -3

   
#Add、“Remove”、“Refactor”、“Update”
commit_str=""

if __name__ == "__main__":
    if len(sys.argv) == 2:
        commit_message=sys.argv[1]
        print(commit_str)
    else:
        print("未输入commit message")

    os.system('git add .')
    os.system('git commit -m "demo"')
