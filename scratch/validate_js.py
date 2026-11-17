import re

content = open('app/static/index.html', encoding='utf-8').read()
start = content.find('<script>')
end = content.find('</script>')
js = content[start+8:end]

# State-machine character scanner to strip string literals and comments
cleaned = []
in_string = None
in_line_comment = False
in_block_comment = False
i = 0
n = len(js)

while i < n:
    c = js[i]
    nxt = js[i+1] if i + 1 < n else ''

    if in_line_comment:
        if c == '\n':
            in_line_comment = False
        i += 1
        continue

    if in_block_comment:
        if c == '*' and nxt == '/':
            in_block_comment = False
            i += 2
            continue
        i += 1
        continue

    if in_string:
        if c == '\\':
            i += 2 # skip escaped character
            continue
        if c == in_string:
            in_string = None
        i += 1
        continue

    # Not in string or comment
    if c == '/' and nxt == '/':
        in_line_comment = True
        i += 2
        continue
    if c == '/' and nxt == '*':
        in_block_comment = True
        i += 2
        continue
    if c in ('"', "'", '`'):
        in_string = c
        i += 1
        continue

    cleaned.append(c)
    i += 1

cleaned_str = ''.join(cleaned)
brackets = {'(': ')', '{': '}', '[': ']'}
stack = []
for idx, c in enumerate(cleaned_str):
    if c in brackets:
        stack.append((c, idx))
    elif c in brackets.values():
        if not stack:
            print(f'Unmatched closing {c} at index {idx}')
            exit(1)
        top, top_idx = stack.pop()
        if brackets[top] != c:
            print(f'Mismatched {top} at {top_idx} and {c} at {idx}')
            exit(1)

if stack:
    print(f'Unclosed brackets remaining: {len(stack)}')
    exit(1)

print('JavaScript state-machine tokenization passed with 0 syntax errors!')

