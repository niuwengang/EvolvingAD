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
    print(sys.argv)
    # if len(sys.argv) <= 2:
    #     commit_str=sys.argv[0]
    os.system('git add .')
    os.system('git commit -m "demo"')
