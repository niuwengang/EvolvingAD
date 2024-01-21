# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import requests
import json

#获取SD地图

# 定义经纬度边界
north, south, east, west = 39.9522, 39.8899, -75.1638, -75.2264  # 替换为您想要的边界

# 构建Overpass API请求
overpass_url = "http://overpass-api.de/api/interpreter"
overpass_query = f"""
[out:json];
way({south},{west},{north},{east})["highway"];
(._;>;);
out body;
"""

# 发送请求
response = requests.get(overpass_url, params={'data': overpass_query})
# 检查响应是否成功
if response.status_code == 200:
    data = response.json()
    print(data)
    # 保存JSON数据到文件
    with open('osm_data.json', 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=4)
    print("数据已保存到osm_data.json")
else:
    print("请求失败，状态码：", response.status_code)