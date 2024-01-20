# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import os
import sys

status=os.system("cd build && cmake -j$(nproc) .. && make -j$(nproc)")
if status==0:
    print("--编译成功")
else:
    print("--编译失败")
    sys.exit(1)

