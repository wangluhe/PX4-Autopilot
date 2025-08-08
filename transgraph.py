import json
import pygraphviz as pgv
from pathlib import Path

# 1. 加载JSON数据
json_path = "Tools/uorb_graph/graph_holybro_kakuteh7.json"
data = json.loads(Path(json_path).read_text())

# 2. 创建有向图
G = pgv.AGraph(directed=True, strict=False, rankdir="LR", fontname="Helvetica")

# 3. 添加节点和边
for node in data["nodes"]:
        G.add_node(node["name"], shape="box", style="filled", fillcolor="#F0F0F0")
for edge in data["links"]:
        G.add_edge(edge["source"], edge["target"], penwidth="1.5")

# 4. 生成HTML
html_path = "Tools/uorb_graph/graph_holybro_kakuteh7.html"
G.draw(html_path, prog="dot", format="svg")
print(f"Generated: {html_path}")
