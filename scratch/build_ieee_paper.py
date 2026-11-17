#!/usr/bin/env python3
"""
AstroMesh-X — IEEE Research Paper PDF Generator (V3 - Robust Typesetting)
Converts docs/ASTROMESH_X_RESEARCH_MANUSCRIPT.md to docs/ASTROMESH_X_RESEARCH_MANUSCRIPT.pdf
formatted strictly according to IEEE Transactions on Robotics (T-RO) specifications.
Guarantees non-empty pages, authentic mathematical typesetting, and valid PDF streams.
"""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MANUSCRIPT_MD = REPO_ROOT / "docs" / "ASTROMESH_X_RESEARCH_MANUSCRIPT.md"
OUTPUT_HTML = REPO_ROOT / "scratch" / "ieee_manuscript.html"
OUTPUT_PDF = REPO_ROOT / "docs" / "ASTROMESH_X_RESEARCH_MANUSCRIPT.pdf"
DOWNLOAD_PDF = Path(r"C:\Users\TANISHKA SAWANT\Downloads\ASTROMESH_X_RESEARCH_MANUSCRIPT.pdf")
CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"

ROMAN = {
    1: "I", 2: "II", 3: "III", 4: "IV", 5: "V", 6: "VI", 7: "VII", 8: "VIII",
    9: "IX", 10: "X", 11: "XI", 12: "XII", 13: "XIII", 14: "XIV", 15: "XV",
    16: "XVI", 17: "XVII", 18: "XVIII", 19: "XIX", 20: "XX"
}

LETTER = {1: "A", 2: "B", 3: "C", 4: "D", 5: "E", 6: "F", 7: "G", 8: "H"}

def parse_markdown_table(lines):
    header_line = lines[0]
    headers = [c.strip() for c in header_line.split("|")[1:-1]]
    rows = []
    for l in lines[2:]:
        if "|" in l:
            cols = [c.strip() for c in l.split("|")[1:-1]]
            rows.append(cols)
    return headers, rows

def table_to_html(headers, rows, caption=""):
    html = []
    # Close previous two-column flow, insert spanning table, resume two-column flow
    html.append("</div>") # close two-column-layout
    html.append("<div class='spanning-block'>")
    if caption:
        html.append(f"<div class='table-caption'>{caption}</div>")
    html.append("<table class='ieee-table'>")
    html.append("<thead><tr>")
    for h in headers:
        h_clean = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', h)
        h_clean = re.sub(r'\*(.*?)\*', r'<i>\1</i>', h_clean)
        html.append(f"<th>{h_clean}</th>")
    html.append("</tr></thead><tbody>")
    for r in rows:
        html.append("<tr>")
        for i, c in enumerate(r):
            c_clean = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', c)
            c_clean = re.sub(r'\*(.*?)\*', r'<i>\1</i>', c_clean)
            align = "center" if i == 0 or len(c) < 15 else "left"
            html.append(f"<td style='text-align: {align};'>{c_clean}</td>")
        html.append("</tr>")
    html.append("</tbody></table>")
    html.append("</div>") # close spanning-block
    html.append("<div class='two-column-layout'>") # reopen two-column-layout
    return "\n".join(html)

def convert_manuscript():
    print(f"Reading manuscript from {MANUSCRIPT_MD}...")
    with open(MANUSCRIPT_MD, "r", encoding="utf-8") as f:
        md_text = f.read()

    lines = md_text.splitlines()

    title = "AstroMesh-X: Decentralized Topological-Metric SLAM for Autonomous Multi-Rover Swarms Under Communication Partitions and Memory Constraints"
    authors = "AstroMesh-X Research and Engineering Team"
    affiliations = "Autonomous Robotics and Planetary Systems Laboratory"
    target_venue = "IEEE Transactions on Robotics (T-RO) / IEEE Robotics and Automation Letters (RA-L) Format"

    # Extract abstract and keywords
    abstract_match = re.search(r'### Abstract\s*\n(.*?)\n### Keywords', md_text, re.DOTALL)
    abstract_text = abstract_match.group(1).strip() if abstract_match else ""

    keywords_match = re.search(r'### Keywords\s*\n(.*?)\n---', md_text, re.DOTALL)
    keywords_text = keywords_match.group(1).strip() if keywords_match else ""

    # Parse body lines starting after the header rule
    body_lines = []
    header_passed = False
    dash_count = 0
    for l in lines:
        if l.strip() == "---":
            dash_count += 1
            if dash_count >= 2:
                header_passed = True
                continue
        if header_passed:
            body_lines.append(l)

    # Convert sections and contents
    html_parts = []

    # Title block (spanning full width)
    html_parts.append(f"""
    <div class='title-block'>
        <div class='journal-header'>IEEE TRANSACTIONS ON ROBOTICS, PREPRINT, NOVEMBER 2026</div>
        <h1 class='paper-title'>{title}</h1>
        <div class='author-block'>{authors}</div>
        <div class='affiliation-block'>{affiliations}</div>
        <div class='venue-block'>{target_venue}</div>
    </div>
    <div class='two-column-layout'>
    """)

    # Abstract & Keywords (spanning width or top of first column per IEEE)
    html_parts.append(f"""
    <div class='abstract-container'>
        <p class='abstract-p'><span class='abstract-heading'>Abstract</span>—<b>{abstract_text}</b></p>
        <p class='keywords-p'><span class='keywords-heading'>Index Terms</span>—<i>{keywords_text}</i></p>
    </div>
    """)

    i = 0
    in_code = False
    code_lines = []
    code_lang = ""
    table_caption = ""
    eq_count = 0

    while i < len(body_lines):
        line = body_lines[i]

        # 1. Code / Mermaid block
        if line.startswith("```"):
            if not in_code:
                in_code = True
                code_lang = line[3:].strip()
                code_lines = []
            else:
                in_code = False
                if code_lang == "mermaid":
                    caption = ""
                    if i + 1 < len(body_lines) and body_lines[i+1].startswith("*Figure"):
                        caption = body_lines[i+1].strip("*")
                        i += 1
                    mermaid_code = "\n".join(code_lines)
                    html_parts.append("</div>") # close two-column
                    html_parts.append(f"""
                    <div class='spanning-block'>
                        <div class='mermaid'>{mermaid_code}</div>
                        {f"<div class='figure-caption'>{caption}</div>" if caption else ""}
                    </div>
                    """)
                    html_parts.append("<div class='two-column-layout'>") # reopen two-column
                else:
                    code_text = "\n".join(code_lines)
                    html_parts.append(f"<pre><code>{code_text}</code></pre>")
            i += 1
            continue

        if in_code:
            code_lines.append(line)
            i += 1
            continue

        # 2. Table caption
        if line.startswith("*Table"):
            table_caption = line.strip("*")
            i += 1
            continue

        # 3. Table detection (Requires line and next line to start with | and line 2 to have ---)
        if line.strip().startswith("|") and i + 1 < len(body_lines) and body_lines[i+1].strip().startswith("|") and "---" in body_lines[i+1]:
            table_lines = [line, body_lines[i+1]]
            i += 2
            while i < len(body_lines) and body_lines[i].strip().startswith("|"):
                table_lines.append(body_lines[i])
                i += 1
            headers, rows = parse_markdown_table(table_lines)
            html_parts.append(table_to_html(headers, rows, table_caption))
            table_caption = ""
            continue

        # 4. Display Math detection: $$ ... $$
        if line.strip().startswith("$$") and line.strip().endswith("$$") and len(line.strip()) > 2:
            eq = line.strip()[2:-2].strip()
            eq_count += 1
            html_parts.append(f"""
            <table class='eq-table'>
                <tr>
                    <td class='eq-math'>\\[ {eq} \\]</td>
                    <td class='eq-num'>({eq_count})</td>
                </tr>
            </table>
            """)
            i += 1
            continue
        elif line.strip() == "$$":
            eq_lines = []
            i += 1
            while i < len(body_lines) and body_lines[i].strip() != "$$":
                eq_lines.append(body_lines[i])
                i += 1
            eq = " ".join(eq_lines)
            eq_count += 1
            html_parts.append(f"""
            <table class='eq-table'>
                <tr>
                    <td class='eq-math'>\\[ {eq} \\]</td>
                    <td class='eq-num'>({eq_count})</td>
                </tr>
            </table>
            """)
            i += 1
            continue

        # 5. Figures with captions
        if line.startswith("*Figure"):
            caption = line.strip("*")
            html_parts.append(f"<div class='figure-caption'>{caption}</div>")
            i += 1
            continue

        # 6. Section Headings (## N. Title)
        h2_match = re.match(r'^##\s+(\d+)\.\s+(.*)', line)
        if h2_match:
            sec_num = int(h2_match.group(1))
            sec_title = h2_match.group(2).strip()
            roman = ROMAN.get(sec_num, str(sec_num))
            html_parts.append(f"<h2 class='ieee-sec'>{roman}. {sec_title}</h2>")
            i += 1
            continue

        if line.startswith("## References"):
            html_parts.append("<h2 class='ieee-sec'>REFERENCES</h2>")
            i += 1
            continue

        # 7. Subsection Headings (### N.M Title)
        h3_match = re.match(r'^###\s+(\d+)\.(\d+)\s+(.*)', line)
        if h3_match:
            sub_num = int(h3_match.group(2))
            sub_title = h3_match.group(3).strip()
            letter = LETTER.get(sub_num, str(sub_num))
            html_parts.append(f"<h3 class='ieee-subsec'><i>{letter}. {sub_title}</i></h3>")
            i += 1
            continue

        # 8. Theorems
        if line.startswith("**Theorem"):
            thm_html = re.sub(r'\*\*(Theorem \d+ .*?):\*\*', r'<b><i>\1:</i></b>', line)
            html_parts.append(f"<div class='theorem-box'>{thm_html}</div>")
            i += 1
            continue

        # 9. Reference item
        if line.startswith("- **["):
            ref_clean = re.sub(r'-\s+\*\*\[(\d+)\]\*\*', r'[\1]', line)
            ref_clean = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', ref_clean)
            ref_clean = re.sub(r'\*(.*?)\*', r'<i>\1</i>', ref_clean)
            html_parts.append(f"<div class='reference-item'>{ref_clean}</div>")
            i += 1
            continue

        # 10. General Paragraphs and Lists
        if line.strip():
            p = line
            p = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', p)
            p = re.sub(r'\*(.*?)\*', r'<i>\1</i>', p)
            p = re.sub(r'`(.*?)`', r'<code>\1</code>', p)
            if p.startswith("- "):
                html_parts.append(f"<div class='bullet-item'>• {p[2:]}</div>")
            elif re.match(r'^\d+\.\s+', p):
                html_parts.append(f"<div class='numbered-item'>{p}</div>")
            else:
                html_parts.append(f"<p>{p}</p>")

        i += 1

    html_parts.append("</div>") # close final two-column-layout

    # CSS & MathJax Template
    full_html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>{title}</title>
<style>
@page {{
    size: letter;
    margin-top: 0.75in;
    margin-bottom: 0.85in;
    margin-left: 0.65in;
    margin-right: 0.65in;
}}

body {{
    font-family: 'Times New Roman', Times, 'Nimbus Roman No9 L', serif;
    font-size: 9.5pt;
    line-height: 1.18;
    color: #000;
    margin: 0;
    padding: 0;
}}

.journal-header {{
    font-size: 8pt;
    color: #444;
    text-align: center;
    border-bottom: 0.5pt solid #888;
    padding-bottom: 4pt;
    margin-bottom: 12pt;
    text-transform: uppercase;
    letter-spacing: 0.5pt;
}}

.title-block {{
    text-align: center;
    margin-bottom: 12pt;
}}

.paper-title {{
    font-size: 19pt;
    font-weight: bold;
    line-height: 1.15;
    margin-top: 0;
    margin-bottom: 6pt;
}}

.author-block {{
    font-size: 10.5pt;
    font-weight: bold;
    margin-bottom: 2pt;
}}

.affiliation-block {{
    font-size: 9pt;
    font-style: italic;
    color: #333;
    margin-bottom: 2pt;
}}

.venue-block {{
    font-size: 8pt;
    color: #666;
    margin-bottom: 8pt;
}}

.two-column-layout {{
    column-count: 2;
    column-gap: 0.22in;
    text-align: justify;
    text-justify: inter-word;
}}

.abstract-container {{
    margin-bottom: 8pt;
    font-size: 8.5pt;
    line-height: 1.18;
}}

.abstract-heading {{
    font-weight: bold;
    font-style: italic;
}}

.keywords-heading {{
    font-weight: bold;
    font-style: italic;
}}

.abstract-p {{
    text-indent: 0 !important;
    margin-bottom: 4pt;
}}

.keywords-p {{
    text-indent: 0 !important;
    margin-bottom: 6pt;
}}

h2.ieee-sec {{
    font-size: 9.5pt;
    font-weight: bold;
    text-transform: uppercase;
    text-align: center;
    margin-top: 10pt;
    margin-bottom: 4pt;
    break-after: avoid;
    letter-spacing: 0.5pt;
}}

h3.ieee-subsec {{
    font-size: 9.5pt;
    font-weight: bold;
    text-align: left;
    margin-top: 8pt;
    margin-bottom: 3pt;
    break-after: avoid;
}}

p {{
    margin-top: 0;
    margin-bottom: 5pt;
    text-indent: 10pt;
}}

.bullet-item {{
    margin-left: 10pt;
    margin-bottom: 3pt;
    text-indent: -10pt;
}}

.numbered-item {{
    margin-left: 10pt;
    margin-bottom: 3pt;
    text-indent: -10pt;
}}

table.eq-table {{
    width: 100%;
    border-collapse: collapse;
    border: none;
    margin: 3pt 0;
    break-inside: avoid;
    table-layout: auto;
}}

table.eq-table td.eq-math {{
    text-align: center;
    border: none;
    padding: 0;
}}

table.eq-table td.eq-num {{
    width: 28pt;
    text-align: right;
    vertical-align: middle;
    border: none;
    padding: 0;
    font-size: 8.5pt;
    color: #222;
}}

.theorem-box {{
    margin: 6pt 0;
    padding: 4pt 6pt;
    background-color: #f9f9f9;
    border-left: 2pt solid #000;
    font-size: 9pt;
    break-inside: avoid;
}}

.spanning-block {{
    width: 100%;
    margin: 10pt 0;
    text-align: center;
    break-inside: avoid;
    clear: both;
}}

.table-caption {{
    font-size: 8pt;
    font-weight: bold;
    text-transform: uppercase;
    text-align: center;
    margin-bottom: 3pt;
    letter-spacing: 0.5pt;
}}

.figure-caption {{
    font-size: 8pt;
    text-align: center;
    margin-top: 4pt;
    margin-bottom: 6pt;
    font-style: italic;
}}

table.ieee-table {{
    width: 100%;
    border-collapse: collapse;
    font-size: 7.5pt;
    line-height: 1.15;
    margin: 4pt auto;
}}

table.ieee-table th {{
    border-top: 1.2pt solid #000;
    border-bottom: 0.8pt solid #000;
    padding: 3pt 2pt;
    font-weight: bold;
    text-align: center;
}}

table.ieee-table td {{
    padding: 2.5pt 2pt;
    border-bottom: 0.4pt solid #e0e0e0;
}}

table.ieee-table tr:last-child td {{
    border-bottom: 1.2pt solid #000;
}}

pre, code {{
    font-family: 'Courier New', Courier, monospace;
    font-size: 7.5pt;
    background-color: #f7f7f7;
}}

pre {{
    border: 0.5pt solid #ccc;
    padding: 4pt;
    margin: 4pt 0;
    white-space: pre-wrap;
    break-inside: avoid;
}}

.reference-item {{
    font-size: 7.5pt;
    line-height: 1.15;
    margin-bottom: 3pt;
    padding-left: 14pt;
    text-indent: -14pt;
}}

.mermaid {{
    text-align: center;
    margin: 4pt auto;
    max-width: 95%;
}}
</style>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<script>
mermaid.initialize({{
    startOnLoad: true,
    theme: 'neutral',
    flowchart: {{ useMaxWidth: false, htmlLabels: true }}
}});
</script>
<script>
window.MathJax = {{
    tex: {{
        inlineMath: [['$', '$'], ['\\\\(', '\\\\)']],
        displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']]
    }},
    svg: {{
        fontCache: 'global'
    }}
}};
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"></script>
</head>
<body>
{"".join(html_parts)}
</body>
</html>
"""

    with open(OUTPUT_HTML, "w", encoding="utf-8") as f:
        f.write(full_html)
    print(f"Generated clean IEEE HTML at {OUTPUT_HTML} ({len(full_html)} bytes).")

    # Render to PDF using Chrome Headless with robust modern flags and isolated profile
    import tempfile
    tmp_profile = tempfile.mkdtemp()
    print(f"Rendering PDF with Chrome headless (new engine) at {CHROME_PATH}...")
    cmd = [
        CHROME_PATH,
        "--headless=new",
        "--disable-gpu",
        f"--user-data-dir={tmp_profile}",
        "--no-pdf-header-footer",
        f"--print-to-pdf={OUTPUT_PDF}",
        str(OUTPUT_HTML)
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(f"Chrome error: {res.stderr}")
        sys.exit(res.returncode)

    if not OUTPUT_PDF.exists():
        print(f"[FAIL] Output PDF not created at {OUTPUT_PDF}")
        sys.exit(1)

    # Verification with pypdf
    import pypdf
    reader = pypdf.PdfReader(OUTPUT_PDF)
    num_pages = len(reader.pages)
    total_chars = 0
    print(f"Verifying PDF page integrity ({num_pages} pages)...")
    for idx, page in enumerate(reader.pages):
        txt = page.extract_text()
        chars = len(txt)
        total_chars += chars
        print(f"  Page {idx + 1}: {chars} characters")
        if chars == 0:
            print(f"  [ERROR] Page {idx + 1} is empty!")
            sys.exit(1)

    print(f"PDF integrity verified: {total_chars} total characters across {num_pages} pages (0 empty pages).")
    size = OUTPUT_PDF.stat().st_size
    print(f"[SUCCESS] Generated IEEE Research Manuscript PDF: {OUTPUT_PDF} ({size} bytes, {size / 1024:.1f} KB)")

    # Copy to User Downloads folder
    shutil.copyfile(OUTPUT_PDF, DOWNLOAD_PDF)
    print(f"[SUCCESS] Copied verified PDF to user Downloads: {DOWNLOAD_PDF} ({DOWNLOAD_PDF.stat().st_size} bytes)")

if __name__ == "__main__":
    convert_manuscript()
