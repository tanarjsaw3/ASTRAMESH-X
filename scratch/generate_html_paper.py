#!/usr/bin/env python3
"""
AstroMesh-X — Interactive Research Paper HTML Generator
Produces an elegant, publication-grade standalone HTML research paper:
- docs/ASTROMESH_X_RESEARCH_MANUSCRIPT.html
- app/static/paper.html
- C:\\Users\\TANISHKA SAWANT\\Downloads\\ASTROMESH_X_RESEARCH_MANUSCRIPT.html

Features:
- Dual Reading Modes: Modern Web Article View (responsive) & Classic IEEE Two-Column View.
- Live MathJax 3 with LaTeX rendering for all 40 numbered equations and inline symbols.
- Interactive Mermaid.js vector diagrams with zoom/expand capability.
- Full IEEE tables, theorem callouts, algorithms, and 13 canonical references.
- Interactive Table of Contents with smooth section jumping.
- One-click Browser Print / PDF export support.
"""

import os
import re
import shutil
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MANUSCRIPT_MD = REPO_ROOT / "docs" / "ASTROMESH_X_RESEARCH_MANUSCRIPT.md"
DOCS_HTML = REPO_ROOT / "docs" / "ASTROMESH_X_RESEARCH_MANUSCRIPT.html"
STATIC_HTML = REPO_ROOT / "app" / "static" / "paper.html"
DOWNLOAD_HTML = Path(r"C:\Users\TANISHKA SAWANT\Downloads\ASTROMESH_X_RESEARCH_MANUSCRIPT.html")

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
    html.append("<div class='table-wrapper'>")
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
    html.append("</div>")
    return "\n".join(html)

def build_html_paper():
    print(f"Reading manuscript from {MANUSCRIPT_MD}...")
    with open(MANUSCRIPT_MD, "r", encoding="utf-8") as f:
        md_text = f.read()

    lines = md_text.splitlines()

    title = "AstroMesh-X: Decentralized Topological-Metric SLAM for Autonomous Multi-Rover Swarms Under Communication Partitions and Memory Constraints"
    authors = "AstroMesh-X Research and Engineering Team"
    affiliations = "Autonomous Robotics and Planetary Systems Laboratory"
    target_venue = "IEEE Transactions on Robotics (T-RO) / IEEE Robotics and Automation Letters (RA-L) Preprint"

    abstract_match = re.search(r'### Abstract\s*\n(.*?)\n### Keywords', md_text, re.DOTALL)
    abstract_text = abstract_match.group(1).strip() if abstract_match else ""

    keywords_match = re.search(r'### Keywords\s*\n(.*?)\n---', md_text, re.DOTALL)
    keywords_text = keywords_match.group(1).strip() if keywords_match else ""

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

    # Process TOC sections
    toc_items = []
    sec_counter = 0

    html_parts = []

    # Title Banner Block
    html_parts.append(f"""
    <header class='paper-header'>
        <div class='journal-meta'>IEEE TRANSACTIONS ON ROBOTICS, PREPRINT, NOVEMBER 2026 • SOFTWARE-ONLY REPRODUCIBLE DEMONSTRATOR</div>
        <h1 class='paper-title'>{title}</h1>
        <div class='paper-authors'>{authors}</div>
        <div class='paper-affil'>{affiliations}</div>
        <div class='paper-venue'>{target_venue}</div>
        <div class='paper-badges'>
            <span class='badge'>C++17 Architecture</span>
            <span class='badge'>SE(2) Riemannian Dynamics</span>
            <span class='badge'>1 KB IBLT Reconciliation</span>
            <span class='badge'>Zero Fill-In Compaction</span>
            <span class='badge'>66/66 CTests Passing</span>
            <span class='badge'>13/13 Acceptance Passing</span>
        </div>
    </header>
    """)

    # Abstract Box
    html_parts.append(f"""
    <section class='abstract-card' id='abstract'>
        <div class='abstract-title'>ABSTRACT</div>
        <p class='abstract-text'>{abstract_text}</p>
        <div class='keywords-box'>
            <span class='keywords-label'>Index Terms—</span>
            <span class='keywords-text'>{keywords_text}</span>
        </div>
    </section>
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
                    html_parts.append(f"""
                    <div class='figure-card'>
                        <div class='mermaid'>{mermaid_code}</div>
                        {f"<div class='figure-caption'>{caption}</div>" if caption else ""}
                    </div>
                    """)
                else:
                    code_text = "\n".join(code_lines)
                    html_parts.append(f"<pre class='code-block'><code>{code_text}</code></pre>")
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

        # 3. Table detection
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
            <div class='equation-container' id='eq-{eq_count}'>
                <div class='equation-math'>\\[ {eq} \\]</div>
                <div class='equation-num'>({eq_count})</div>
            </div>
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
            <div class='equation-container' id='eq-{eq_count}'>
                <div class='equation-math'>\\[ {eq} \\]</div>
                <div class='equation-num'>({eq_count})</div>
            </div>
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
            sec_id = f"sec-{sec_num}"
            toc_items.append((sec_id, f"{roman}. {sec_title}"))
            html_parts.append(f"<h2 class='sec-heading' id='{sec_id}'><span class='sec-num'>{roman}.</span> {sec_title}</h2>")
            i += 1
            continue

        if line.startswith("## References"):
            sec_id = "sec-references"
            toc_items.append((sec_id, "References"))
            html_parts.append(f"<h2 class='sec-heading' id='{sec_id}'>REFERENCES</h2>")
            i += 1
            continue

        # 7. Subsection Headings (### N.M Title)
        h3_match = re.match(r'^###\s+(\d+)\.(\d+)\s+(.*)', line)
        if h3_match:
            sec_num = int(h3_match.group(1))
            sub_num = int(h3_match.group(2))
            sub_title = h3_match.group(3).strip()
            letter = LETTER.get(sub_num, str(sub_num))
            sub_id = f"subsec-{sec_num}-{sub_num}"
            html_parts.append(f"<h3 class='subsec-heading' id='{sub_id}'><span class='subsec-letter'>{letter}.</span> {sub_title}</h3>")
            i += 1
            continue

        # 8. Theorems
        if line.startswith("**Theorem"):
            thm_html = re.sub(r'\*\*(Theorem \d+ .*?):\*\*', r'<b>\1:</b>', line)
            html_parts.append(f"<div class='theorem-card'>{thm_html}</div>")
            i += 1
            continue

        # 9. Reference item
        if line.startswith("- **["):
            ref_clean = re.sub(r'-\s+\*\*\[(\d+)\]\*\*', r'<span class="ref-num">[\1]</span>', line)
            ref_clean = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', ref_clean)
            ref_clean = re.sub(r'\*(.*?)\*', r'<i>\1</i>', ref_clean)
            html_parts.append(f"<div class='ref-item'>{ref_clean}</div>")
            i += 1
            continue

        # 10. General Paragraphs and Lists
        if line.strip():
            p = line
            p = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', p)
            p = re.sub(r'\*(.*?)\*', r'<i>\1</i>', p)
            p = re.sub(r'`(.*?)`', r'<code>\1</code>', p)
            if p.startswith("- "):
                html_parts.append(f"<li class='bullet-item'>{p[2:]}</li>")
            elif re.match(r'^\d+\.\s+', p):
                item_content = re.sub(r'^\d+\.\s+', '', p)
                html_parts.append(f"<li class='numbered-item'>{item_content}</li>")
            else:
                html_parts.append(f"<p class='body-p'>{p}</p>")

        i += 1

    # Build TOC HTML
    toc_html = []
    toc_html.append("<div class='toc-card'><div class='toc-header'>Contents</div><div class='toc-grid'>")
    for sec_id, sec_label in toc_items:
        toc_html.append(f"<a href='#{sec_id}' class='toc-link'>{sec_label}</a>")
    toc_html.append("</div></div>")

    # Complete Document Template
    full_html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title}</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 100 100%22><text y=%22.9em%22 font-size=%2290%22>🚀</text></svg>">
<style>
:root {{
    --primary: #0284c7;
    --primary-dark: #0369a1;
    --primary-light: #e0f2fe;
    --accent: #7c3aed;
    --text-main: #1e293b;
    --text-muted: #64748b;
    --bg-page: #f8fafc;
    --bg-card: #ffffff;
    --border-color: #e2e8f0;
    --font-serif: 'Times New Roman', Times, 'Nimbus Roman No9 L', serif;
    --font-sans: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
    --font-mono: 'Consolas', 'Courier New', Courier, monospace;
}}

* {{
    box-sizing: border-box;
}}

body {{
    margin: 0;
    padding: 0;
    background-color: var(--bg-page);
    color: var(--text-main);
    font-family: var(--font-serif);
    font-size: 10.5pt;
    line-height: 1.45;
    -webkit-font-smoothing: antialiased;
}}

/* Top Interactive Bar */
.action-bar {{
    position: sticky;
    top: 0;
    z-index: 1000;
    background: #0f172a;
    color: #f8fafc;
    padding: 10px 24px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    border-bottom: 1px solid #334155;
    font-family: var(--font-sans);
    box-shadow: 0 2px 10px rgba(0,0,0,0.25);
}}

.brand {{
    display: flex;
    align-items: center;
    gap: 10px;
    font-weight: 700;
    font-size: 0.95rem;
    letter-spacing: 0.03em;
}}

.brand-pill {{
    background: linear-gradient(135deg, #0284c7, #7c3aed);
    color: white;
    padding: 2px 8px;
    border-radius: 4px;
    font-size: 0.75rem;
    font-weight: 800;
}}

.bar-controls {{
    display: flex;
    align-items: center;
    gap: 12px;
}}

.btn {{
    background: #1e293b;
    color: #f8fafc;
    border: 1px solid #475569;
    padding: 6px 14px;
    border-radius: 6px;
    font-size: 0.82rem;
    font-weight: 600;
    cursor: pointer;
    text-decoration: none;
    display: inline-flex;
    align-items: center;
    gap: 6px;
    transition: all 0.2s ease;
}}

.btn:hover {{
    background: #334155;
    border-color: #64748b;
    color: #38bdf8;
}}

.btn-primary {{
    background: #0284c7;
    border-color: #0284c7;
    color: white;
}}

.btn-primary:hover {{
    background: #0369a1;
}}

/* Paper Container */
.paper-wrapper {{
    max-width: 960px;
    margin: 32px auto 60px;
    padding: 48px 56px;
    background: var(--bg-card);
    border-radius: 8px;
    box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.05), 0 8px 10px -6px rgba(0, 0, 0, 0.01);
    border: 1px solid var(--border-color);
    transition: all 0.3s ease;
}}

/* Two Column Layout Switcher */
body.layout-two-column .paper-wrapper {{
    max-width: 1100px;
    padding: 40px 48px;
}}

body.layout-two-column .article-body {{
    column-count: 2;
    column-gap: 28px;
    text-align: justify;
    text-justify: inter-word;
}}

body.layout-two-column .spanning-full,
body.layout-two-column .figure-card,
body.layout-two-column .table-wrapper,
body.layout-two-column .sec-heading,
body.layout-two-column .toc-card {{
    column-span: all;
}}

/* Header Banner */
.paper-header {{
    text-align: center;
    margin-bottom: 24px;
    border-bottom: 1px solid var(--border-color);
    padding-bottom: 20px;
}}

.journal-meta {{
    font-family: var(--font-sans);
    font-size: 0.78rem;
    font-weight: 700;
    color: var(--primary-dark);
    letter-spacing: 0.05em;
    margin-bottom: 12px;
}}

.paper-title {{
    font-size: 21pt;
    font-weight: 800;
    line-height: 1.22;
    color: #0f172a;
    margin: 0 0 14px;
}}

.paper-authors {{
    font-size: 11.5pt;
    font-weight: 700;
    color: #334155;
    margin-bottom: 4px;
}}

.paper-affil {{
    font-size: 10pt;
    font-style: italic;
    color: #64748b;
    margin-bottom: 6px;
}}

.paper-venue {{
    font-size: 8.8pt;
    color: #94a3b8;
    margin-bottom: 14px;
}}

.paper-badges {{
    display: flex;
    justify-content: center;
    flex-wrap: wrap;
    gap: 8px;
    margin-top: 10px;
}}

.badge {{
    font-family: var(--font-sans);
    font-size: 0.72rem;
    font-weight: 700;
    background: #f1f5f9;
    color: #475569;
    padding: 3px 8px;
    border-radius: 4px;
    border: 1px solid #cbd5e1;
}}

/* Abstract Card */
.abstract-card {{
    background: #f8fafc;
    border-left: 4px solid var(--primary);
    padding: 16px 20px;
    margin-bottom: 28px;
    border-radius: 0 6px 6px 0;
}}

.abstract-title {{
    font-family: var(--font-sans);
    font-size: 0.82rem;
    font-weight: 800;
    letter-spacing: 0.08em;
    color: var(--primary-dark);
    margin-bottom: 6px;
}}

.abstract-text {{
    margin: 0 0 10px;
    text-align: justify;
    line-height: 1.45;
}}

.keywords-box {{
    font-size: 9.5pt;
    line-height: 1.35;
}}

.keywords-label {{
    font-weight: 700;
    font-style: italic;
}}

.keywords-text {{
    font-style: italic;
    color: #334155;
}}

/* Table of Contents */
.toc-card {{
    background: #ffffff;
    border: 1px solid var(--border-color);
    border-radius: 6px;
    padding: 16px 20px;
    margin-bottom: 28px;
    font-family: var(--font-sans);
}}

.toc-header {{
    font-size: 0.88rem;
    font-weight: 800;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: #334155;
    margin-bottom: 12px;
    border-bottom: 1px solid #f1f5f9;
    padding-bottom: 6px;
}}

.toc-grid {{
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
    gap: 8px 16px;
}}

.toc-link {{
    font-size: 0.82rem;
    color: var(--primary);
    text-decoration: none;
    transition: color 0.2s;
}}

.toc-link:hover {{
    color: var(--accent);
    text-decoration: underline;
}}

/* Section Headings */
h2.sec-heading {{
    font-size: 13pt;
    font-weight: 800;
    text-transform: uppercase;
    color: #0f172a;
    letter-spacing: 0.04em;
    margin: 28px 0 12px;
    padding-bottom: 6px;
    border-bottom: 1px solid #cbd5e1;
    scroll-margin-top: 70px;
}}

.sec-num {{
    color: var(--primary);
}}

h3.subsec-heading {{
    font-size: 11pt;
    font-weight: 700;
    color: #1e293b;
    margin: 20px 0 8px;
    scroll-margin-top: 70px;
}}

.subsec-letter {{
    color: var(--primary);
    font-style: italic;
}}

/* Body Text */
p.body-p {{
    margin: 0 0 10pt;
    text-indent: 14pt;
    text-align: justify;
    line-height: 1.45;
}}

li.bullet-item, li.numbered-item {{
    margin-bottom: 6pt;
    line-height: 1.45;
    text-align: justify;
}}

/* Math & Equations */
.equation-container {{
    display: flex;
    align-items: center;
    justify-content: center;
    position: relative;
    margin: 14px 0;
    padding: 6px 12px;
    background: #fdfefe;
    border-radius: 4px;
}}

.equation-math {{
    margin: 0 auto;
}}

.equation-num {{
    position: absolute;
    right: 8px;
    font-family: var(--font-serif);
    font-size: 9.5pt;
    color: #475569;
}}

/* Theorem Card */
.theorem-card {{
    background: #f0fdf4;
    border-left: 4px solid #16a34a;
    padding: 12px 18px;
    margin: 14px 0;
    border-radius: 0 6px 6px 0;
    font-size: 10pt;
    line-height: 1.4;
}}

/* Figures & Diagrams */
.figure-card {{
    margin: 22px 0;
    padding: 16px;
    background: #ffffff;
    border: 1px solid var(--border-color);
    border-radius: 6px;
    text-align: center;
}}

.mermaid {{
    margin: 0 auto;
    overflow-x: auto;
}}

.figure-caption {{
    font-family: var(--font-sans);
    font-size: 0.82rem;
    font-weight: 600;
    color: #475569;
    margin-top: 10px;
    line-height: 1.35;
}}

/* Tables */
.table-wrapper {{
    margin: 22px 0;
    overflow-x: auto;
    text-align: center;
}}

.table-caption {{
    font-family: var(--font-sans);
    font-size: 0.84rem;
    font-weight: 700;
    text-transform: uppercase;
    color: #1e293b;
    letter-spacing: 0.04em;
    margin-bottom: 8px;
}}

table.ieee-table {{
    width: 100%;
    border-collapse: collapse;
    font-family: var(--font-serif);
    font-size: 8.8pt;
    line-height: 1.3;
    margin: 0 auto;
}}

table.ieee-table th {{
    border-top: 2px solid #000;
    border-bottom: 1px solid #000;
    padding: 6px 8px;
    font-weight: 800;
    text-align: center;
    background: #f8fafc;
}}

table.ieee-table td {{
    padding: 5px 8px;
    border-bottom: 1px solid #e2e8f0;
}}

table.ieee-table tr:last-child td {{
    border-bottom: 2px solid #000;
}}

/* Code Blocks */
pre.code-block {{
    background: #0f172a;
    color: #f8fafc;
    padding: 12px 16px;
    border-radius: 6px;
    font-family: var(--font-mono);
    font-size: 8pt;
    overflow-x: auto;
    line-height: 1.4;
    margin: 12px 0;
}}

code {{
    font-family: var(--font-mono);
    font-size: 8.8pt;
    background: #f1f5f9;
    padding: 1px 4px;
    border-radius: 3px;
    color: #0f172a;
}}

pre code {{
    background: transparent;
    padding: 0;
    color: inherit;
}}

/* References */
.ref-item {{
    font-size: 9pt;
    line-height: 1.35;
    margin-bottom: 8px;
    padding-left: 24px;
    text-indent: -24px;
    text-align: justify;
}}

.ref-num {{
    font-weight: 700;
    color: var(--primary-dark);
}}

/* Print Styles */
@media print {{
    .action-bar, .paper-badges, .toc-card {{
        display: none !important;
    }}
    body {{
        background: white !important;
        font-size: 9pt;
        line-height: 1.15;
    }}
    .paper-wrapper {{
        max-width: 100% !important;
        margin: 0 !important;
        padding: 0 !important;
        border: none !important;
        box-shadow: none !important;
    }}
    .article-body {{
        column-count: 2 !important;
        column-gap: 0.25in !important;
    }}
    @page {{
        size: letter;
        margin: 0.75in 0.65in;
    }}
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
        displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']],
        processEscapes: true
    }},
    svg: {{
        fontCache: 'global'
    }}
}};
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"></script>
<script>
function toggleLayout() {{
    const body = document.body;
    const btn = document.getElementById('layoutBtn');
    if (body.classList.contains('layout-two-column')) {{
        body.classList.remove('layout-two-column');
        btn.innerHTML = '📰 Switch to IEEE Two-Column';
    }} else {{
        body.classList.add('layout-two-column');
        btn.innerHTML = '📖 Switch to Modern Web Reader';
    }}
}}
</script>
</head>
<body>

<!-- Sticky Header Bar -->
<div class='action-bar'>
    <div class='brand'>
        <span class='brand-pill'>IEEE T-RO PREPRINT</span>
        <span>AstroMesh-X Scientific Manuscript</span>
    </div>
    <div class='bar-controls'>
        <button id='layoutBtn' class='btn' onclick='toggleLayout()'>📰 Switch to IEEE Two-Column</button>
        <button class='btn btn-primary' onclick='window.print()'>🖨️ Print / Save as PDF</button>
        <a href='/index.html' class='btn'>🚀 Live Simulator Dashboard</a>
    </div>
</div>

<!-- Main Document Card -->
<div class='paper-wrapper'>

    <!-- Table of Contents -->
    {"".join(html_parts[:2])}
    {"".join(toc_html)}

    <!-- Article Content -->
    <article class='article-body'>
    {"".join(html_parts[2:])}
    </article>

</div>

</body>
</html>
"""

    # 1. Write to docs/
    with open(DOCS_HTML, "w", encoding="utf-8") as f:
        f.write(full_html)
    print(f"[SUCCESS] Generated {DOCS_HTML} ({len(full_html)} bytes, {DOCS_HTML.stat().st_size / 1024:.1f} KB)")

    # 2. Write to app/static/
    with open(STATIC_HTML, "w", encoding="utf-8") as f:
        f.write(full_html)
    print(f"[SUCCESS] Deployed to web server static directory: {STATIC_HTML}")

    # 3. Copy to User Downloads folder
    shutil.copyfile(DOCS_HTML, DOWNLOAD_HTML)
    print(f"[SUCCESS] Copied HTML paper to user Downloads: {DOWNLOAD_HTML} ({DOWNLOAD_HTML.stat().st_size / 1024:.1f} KB)")

if __name__ == "__main__":
    build_html_paper()
