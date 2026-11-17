with open('app/static/index.html', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if '<script>' in line or '</script>' in line or 'function ' in line:
        if i >= 1150:
            print(f"{i+1}: {line.strip()[:100]}")

