#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import json
import io

try:
    from Utils import pcl_parser
except Exception:
    import pcl_parser


class Loader(object):
    def __init__(self):
        pass

    def _resolve_json_path(self, file_name):
        if file_name.endswith(".json"):
            return file_name
        return file_name + ".json"

    def _strip_json_comments(self, text):
        clean_lines = []
        for line in text.splitlines():
            if "#" in line:
                line = line.split("#", 1)[0]
            clean_lines.append(line)
        return "\n".join(clean_lines)

    def load(self, file_name):
        json_path = self._resolve_json_path(file_name)

        if not os.path.exists(json_path):
            print("Scenario json not found: {}".format(json_path))
            return None

        with io.open(json_path, "r", encoding="utf-8") as f:
            raw_text = f.read()

        clean_text = self._strip_json_comments(raw_text)

        try:
            data = json.loads(clean_text)
        except Exception as e:
            print("Failed to parse json: {}".format(json_path))
            print(e)
            return None

        scenario = pcl_parser.Scenario()
        scenario.boundaries = data.get("boundaries", [])
        scenario.obstacles = data.get("obstacles", [])

        return scenario
# '''
# MIT License

# Copyright (c) [year] Wang Yue (ywang_86@stu.xidian.edu.cn), Xidian University, Shaanxi, Xian, China 

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# '''
                
# import sys
# sys.path.append("../")
# from Utils import pcl_parser


# import os
# import json


# class Loader:
#     def __init__(self):
#         pass

#     def _strip_json_comments(self, text):
#         """
#         原始 1.json 里有 # 中文注释，不是标准 JSON。
#         这里自动去掉 # 后面的注释。
#         """
#         clean_lines = []

#         for line in text.splitlines():
#             if "#" in line:
#                 line = line.split("#", 1)[0]
#             clean_lines.append(line)

#         return "\n".join(clean_lines)

#     def _resolve_json_path(self, file_name):
#         """
#         支持两种写法：
#         1. /xxx/1.json
#         2. /xxx/1
#         """
#         if file_name.endswith(".json"):
#             return file_name

#         return file_name + ".json"

#     def load(self, file_name):
#         json_path = self._resolve_json_path(file_name)

#         if not os.path.exists(json_path):
#             print("Scenario json not found: {}".format(json_path))
#             return None

#         with open(json_path, "r") as f:
#             raw_text = f.read()

#         clean_text = self._strip_json_comments(raw_text)

#         try:
#             data = json.loads(clean_text)
#         except Exception as e:
#             print("Failed to parse scenario json: {}".format(json_path))
#             print(e)
#             return None

#         scenario = pcl_parser.Scenario()

#         scenario.boundaries = data.get("boundaries", [])
#         scenario.obstacles = data.get("obstacles", [])

#         scenario.GRF = 0
#         scenario.ORFs = [0 for _ in scenario.obstacles]
#         scenario.Mindist = 100

#         return scenario


# class Saver:
#     def __init__(self, scenario):
#         assert type(scenario) == pcl_parser.Scenario, "Error: scenario type error!"
#         self.scenario = scenario

#     def save(self, file_name):
#         json_path = file_name

#         if not json_path.endswith(".json"):
#             json_path += ".json"

#         data = {
#             "boundaries": self.scenario.boundaries,
#             "obstacles": self.scenario.obstacles
#         }

#         with open(json_path, "w") as f:
#             json.dump(data, f, indent=4)
