# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import os
import sys



if __name__ == "__main__":
    exec_bin="_test"
    if len(sys.argv) == 2:
        exec_bin=sys.argv[1]
        
    else:
        print("未输入可执行程序")
        sys.exit(1)

    status=os.system('cd build  && make -j$(nproc)')
    if status==0:
        print("--编译成功")
    else:
        print("--编译失败")
        sys.exit(1)

    status=os.system('./bin/\"exec_bin\" -j$(nproc)')
    if status==0:
        print("--运行成功")
    else:
        print("--运行失败")
        sys.exit(1)
