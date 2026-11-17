import re

with open('app/static/index.html', encoding='utf-8') as f:
    lines = f.readlines()

print(f"Total lines: {len(lines)}")
for i, line in enumerate(lines):
    if any(k in line for k in ['<main', '<nav', 'class="tab', 'id="simCanvas', 'id="tab-', 'class="btn', 'id="rover-table']):
        print(f"{i+1}: {line.strip()[:100]}")

