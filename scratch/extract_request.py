import json

path = r'C:\Users\TANISHKA SAWANT\.gemini\antigravity\brain\b74971ce-a1b4-4378-b0e1-572a83b3a31f\.system_generated\logs\transcript_full.jsonl'
with open(path, 'r', encoding='utf-8') as f:
    for line in f:
        if '"step_index":688' in line:
            obj = json.loads(line)
            with open('scratch/phase62_request.txt', 'w', encoding='utf-8') as out:
                out.write(obj.get('content', ''))
            print('Extracted successfully')
            break

