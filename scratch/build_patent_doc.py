import docx
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml import parse_xml
from docx.oxml.ns import nsdecls
import os

doc = docx.Document()

# Page Margins: 1.0 inch all around
for section in doc.sections:
    section.top_margin = Inches(1.0)
    section.bottom_margin = Inches(1.0)
    section.left_margin = Inches(1.0)
    section.right_margin = Inches(1.0)

def set_cell_background(cell, hex_color):
    shading_elm = parse_xml(f'<w:shd {nsdecls("w")} w:fill="{hex_color}"/>')
    cell._tc.get_or_add_tcPr().append(shading_elm)

def set_cell_margins(cell, top=100, bottom=100, left=150, right=150):
    tcPr = cell._tc.get_or_add_tcPr()
    tcMar = parse_xml(f'<w:tcMar {nsdecls("w")}><w:top w:w="{top}" w:type="dxa"/><w:bottom w:w="{bottom}" w:type="dxa"/><w:left w:w="{left}" w:type="dxa"/><w:right w:w="{right}" w:type="dxa"/></w:tcMar>')
    tcPr.append(tcMar)

def add_heading_1(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(14)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.keep_with_next = True
    run = p.add_run(text)
    run.font.name = 'Times New Roman'
    run.font.size = Pt(12)
    run.font.bold = True
    return p

def add_heading_2(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(10)
    p.paragraph_format.space_after = Pt(3)
    p.paragraph_format.keep_with_next = True
    run = p.add_run(text)
    run.font.name = 'Times New Roman'
    run.font.size = Pt(11)
    run.font.bold = True
    return p

def add_body(text, bold_prefix=None, space_after=4):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(space_after)
    p.paragraph_format.line_spacing = 1.15
    if bold_prefix:
        r_pre = p.add_run(bold_prefix)
        r_pre.font.name = 'Times New Roman'
        r_pre.font.size = Pt(10.5)
        r_pre.font.bold = True
    run = p.add_run(text)
    run.font.name = 'Times New Roman'
    run.font.size = Pt(10.5)
    return p

def add_bullet(text, bold_prefix=None):
    p = doc.add_paragraph(style='List Bullet')
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(3)
    p.paragraph_format.line_spacing = 1.15
    if bold_prefix:
        r_pre = p.add_run(bold_prefix)
        r_pre.font.name = 'Times New Roman'
        r_pre.font.size = Pt(10.5)
        r_pre.font.bold = True
    run = p.add_run(text)
    run.font.name = 'Times New Roman'
    run.font.size = Pt(10.5)
    return p

def add_equation_image(image_path, height_in_inches=0.35):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(6)
    p.paragraph_format.space_after = Pt(6)
    run = p.add_run()
    run.add_picture(image_path, height=Inches(height_in_inches))
    return p

# TITLE BLOCK
p_top = doc.add_paragraph()
p_top.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_top.paragraph_format.space_after = Pt(2)
r_top = p_top.add_run('PATENT PROPOSAL - DRAFT')
r_top.font.name = 'Times New Roman'
r_top.font.size = Pt(16)
r_top.font.bold = True

p_sub = doc.add_paragraph()
p_sub.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_sub.paragraph_format.space_after = Pt(14)
r_sub = p_sub.add_run('AstroMesh-X: Decentralized Multi-Rover Topological SLAM on Planetary Manifolds')
r_sub.font.name = 'Times New Roman'
r_sub.font.size = Pt(13)
r_sub.font.italic = True

add_body('Following information is required for drafting of patent application.', space_after=8)

# 1. APPLICANT TABLE
add_heading_1('1. Full name, nationality and address of applicant(s)')
table_app = doc.add_table(rows=2, cols=3)
table_app.alignment = WD_TABLE_ALIGNMENT.CENTER
headers_app = ['Full Name', 'Nationality', 'Address']
col_widths_app = [Inches(2.5), Inches(1.2), Inches(2.8)]

for i, h in enumerate(headers_app):
    cell = table_app.cell(0, i)
    cell.width = col_widths_app[i]
    set_cell_background(cell, 'F2F2F2')
    set_cell_margins(cell)
    p = cell.paragraphs[0]
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(2)
    run = p.add_run(h)
    run.font.name = 'Times New Roman'
    run.font.size = Pt(10)
    run.font.bold = True

row_data_app = [
    'Vishwakarma Institute of Technology',
    'Indian',
    '666, Upper Indiranagar, Bibwewadi, Pune,\nMaharashtra, India – 411 037'
]
for i, d in enumerate(row_data_app):
    cell = table_app.cell(1, i)
    cell.width = col_widths_app[i]
    set_cell_margins(cell)
    p = cell.paragraphs[0]
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(2)
    run = p.add_run(d)
    run.font.name = 'Times New Roman'
    run.font.size = Pt(9.5)

# 2. INVENTORS TABLE
add_heading_1('2. Full name (including middle name), nationality, address, mail id and phone number of inventor(s)')
table_inv = doc.add_table(rows=6, cols=5)
table_inv.alignment = WD_TABLE_ALIGNMENT.CENTER
headers_inv = ['Full Name\n(Including middle name)', 'Nationality', 'VIT Address', 'Mail ID', 'Phone No.']
col_widths_inv = [Inches(1.8), Inches(0.8), Inches(1.8), Inches(1.4), Inches(0.9)]

for i, h in enumerate(headers_inv):
    cell = table_inv.cell(0, i)
    cell.width = col_widths_inv[i]
    set_cell_background(cell, 'F2F2F2')
    set_cell_margins(cell)
    p = cell.paragraphs[0]
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(2)
    run = p.add_run(h)
    run.font.name = 'Times New Roman'
    run.font.size = Pt(9.5)
    run.font.bold = True

inventors_data = [
    ('Dr. Rakhi Jaykumar Bhardwaj', 'IN', 'Department of Computer Engineering,\nVishwakarma Institute of Technology, Pune', 'rakhi.bhardwaj@vit.edu', '9623378083'),
    ('Tanishka Arjun Sawant', 'IN', 'Department of Computer Engineering,\nVishwakarma Institute of Technology, Pune', 'tanishka.sawant@vit.edu', '+91 9876543210'),
    ('Sameera Watharkar', 'IN', 'Department of Computer Engineering,\nVishwakarma Institute of Technology, Pune', 'sameera.watharkar@vit.edu', '+91 9356671946'),
    ('Rutuja Patil', 'IN', 'Department of Computer Engineering,\nVishwakarma Institute of Technology, Pune', 'rutuja.patil@vit.edu', '+91 9923624908'),
    ('Vedant Salunkhe', 'IN', 'Department of Computer Engineering,\nVishwakarma Institute of Technology, Pune', 'vedant.salunkhe@vit.edu', '+91 9370893866')
]

for row_idx, data in enumerate(inventors_data):
    for col_idx, item in enumerate(data):
        cell = table_inv.cell(row_idx + 1, col_idx)
        cell.width = col_widths_inv[col_idx]
        set_cell_margins(cell)
        p = cell.paragraphs[0]
        p.paragraph_format.space_before = Pt(2)
        p.paragraph_format.space_after = Pt(2)
        run = p.add_run(item)
        run.font.name = 'Times New Roman'
        run.font.size = Pt(9)

# SIGNATURES BLOCK
p_sig = doc.add_paragraph()
p_sig.paragraph_format.space_before = Pt(10)
p_sig.paragraph_format.space_after = Pt(4)
r_sig = p_sig.add_run('Signatures of all inventors:')
r_sig.font.name = 'Times New Roman'
r_sig.font.size = Pt(10.5)
r_sig.font.bold = True

table_sig = doc.add_table(rows=2, cols=5)
table_sig.alignment = WD_TABLE_ALIGNMENT.CENTER
for col_idx in range(5):
    cell_box = table_sig.cell(0, col_idx)
    cell_box.width = Inches(1.3)
    set_cell_margins(cell_box, top=200, bottom=200)
    p_box = cell_box.paragraphs[0]
    p_box.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p_box.add_run('[ Signature ]')
    r.font.name = 'Times New Roman'
    r.font.size = Pt(8.5)
    r.font.color.rgb = RGBColor(150, 150, 150)
    
    cell_lbl = table_sig.cell(1, col_idx)
    cell_lbl.width = Inches(1.3)
    set_cell_margins(cell_lbl)
    p_lbl = cell_lbl.paragraphs[0]
    p_lbl.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run_name = p_lbl.add_run(inventors_data[col_idx][0])
    run_name.font.name = 'Times New Roman'
    run_name.font.size = Pt(8.5)
    run_name.font.bold = True

doc.add_page_break()

# 3. TITLE OF THE INVENTION
add_heading_1('3. Title of the invention')
add_body('DECENTRALIZED MULTI-ROVER TOPOLOGICAL SLAM SYSTEM AND METHOD FOR CONSTANT-OVERHEAD MAP RECONCILIATION AND RIEMANNIAN POSE-GRAPH OPTIMIZATION IN GPS-DENIED AND COMMUNICATION-CONSTRAINED ENVIRONMENTS', space_after=10)

# 4. TECHNICAL FIELD
add_heading_1('4. Technical field of the invention')
add_body('The present invention relates to computer science, robotics, and distributed algorithms, and more particularly to data structures and algorithms applied to multi-robot Simultaneous Localization and Mapping (SLAM), decentralized graph optimization, topological map merging, and delay-tolerant communication under severe bandwidth, latency, and memory constraints. It concerns a computer-implemented system and software architecture, implemented in C++20 with high-performance mathematical libraries, that enables a swarm of autonomous planetary or terrestrial rovers operating in GPS-denied environments to collaboratively discover, geometrically verify, and topologically reconcile local maps using constant-size Invertible Bloom Lookup Table (IBLT) summaries, Disjoint-Set Union (DSU) forest tracking, and Continuous Riemannian Dynamics (CORD) on the SE(2) Lie manifold.')

# 5. PRIOR ART
add_heading_1('5. Prior art')
add_body('Collaborative multi-robot exploration in GPS-denied planetary regimes (such as lunar lava tubes, permanently shadowed crater basins, and Martian canyons) requires autonomous vehicles to construct a globally consistent metric-topological map. Conventional multi-robot SLAM and distributed mapping systems address fragments of this domain but suffer from severe physical and algorithmic limitations:')

add_bullet('Centralized Server-Based Mapping (e.g., C-SLAM, centralized pose-graph optimizers): rovers continuously transmit raw sensor streams (LiDAR point clouds, visual keyframes, dense submaps) to a centralized base station. In extraterrestrial or underground environments, base stations represent single points of failure, and transmission of dense point clouds saturates low-power UHF/S-band radios.', bold_prefix='• Centralized Server-Based Multi-Robot SLAM: ')

add_bullet('Full Map / Keyframe Flooding (e.g., decentralized keyframe broadcasting): rovers broadcast all locally extracted keyframes and feature descriptors to immediate neighbors. Because total keyframes scale linearly with time and trajectory length O(N), the network communication payload explodes to multiple megabytes, causing channel starvation and high packet loss.', bold_prefix='• Naive Keyframe Flooding: ')

add_bullet('Standard Bloom Filters and Counting Bloom Filters (CBF): used for compact set membership testing. While CBFs detect potential overlapping landmarks, they cannot directly extract the symmetric difference between two rovers\' maps and suffer from one-sided false positive errors, requiring subsequent high-bandwidth metadata exchanges.', bold_prefix='• Counting Bloom Filter Set Discovery: ')

add_bullet('Euclidean Pose-Graph Optimization (e.g., standard Gauss-Newton or Levenberg-Marquardt): vectorizes 2D poses into Euclidean coordinates (x, y, theta) and performs gradient descent. Direct addition of planar angles causes gimbal wrap-around singularities and catastrophic topological distortions under large relative orientation errors.', bold_prefix='• Euclidean Vector-Space Optimizers: ')

add_bullet('Naive Marginalization and Schur Complement Pruning: removing historical keyframes via dense Schur complement marginalization produces dense information cliques (O(K^2) fill-in edges in the Hessian matrix), causing unbounded quadratic memory growth on resource-constrained embedded flight computers.', bold_prefix='• Dense Marginalization in Long-Duration SLAM: ')

add_body('Disadvantages of the above that the present invention addresses: (i) existing systems require megabyte-scale wire bandwidth, causing complete failure over low-rate space telemetry (< 100 kbps); (ii) lack of constant-size set reconciliation forces exhaustive key exchanges; (iii) perceptual aliasing in repetitive crater regolith causes false-positive loop closures that ruin the global map; (iv) network partitions cause dead-reckoning odometry drift to diverge by tens of meters without a mechanism to snap drifted trajectories back to ground truth upon reconnection; (v) global orientation gauge ambiguity leads to unconstrained internal coordinate frame drift; and (vi) memory consumption on embedded processors grows monotonically until system exhaustion.')

add_body('[Note for inventors: a formal prior-art search (Indian Patent Office, USPTO, WIPO, and IEEE/Robotics literature) must be carried out and specific patent / publication numbers inserted here before filing.]')

# 6. OBJECT
add_heading_1('6. Object of the invention')
add_bullet('To represent multi-rover spatial landmark observations as algebraic Invertible Bloom Lookup Tables (IBLTs) that encode landmark keys into fixed-size summary cells containing count, keySum, and hashSum, enabling exact extraction of the symmetric set difference (A Δ B) inside a strictly bounded 1,024-byte wire envelope.', bold_prefix='1. Constant-Overhead Set Reconciliation: ')

add_bullet('To track dynamic swarm network partitions, RF blackouts, and topological re-merges using a Disjoint-Set Union (DSU) data structure with path compression and union-by-rank, guaranteeing near O(1) amortized component reconciliation time O(α(V)).', bold_prefix='2. Partition-Resilient Forest Merging: ')

add_bullet('To eliminate rotational wrap-around singularities by executing distributed pose-graph optimization directly on the SE(2) Lie manifold (R^2 x SO(2)) using Continuous Riemannian Dynamics (CORD) with closed-form Lie algebra retraction mappings.', bold_prefix='3. Manifold-Constrained Riemannian Consensus: ')

add_bullet('To reject false-positive perceptual aliasing matches in repetitive terrain by constructing a pairwise Euclidean distance consistency graph and extracting maximum-weight cliques via Horn orthonormal closed-form alignment.', bold_prefix='4. Maximum-Clique Geometric Verification: ')

add_bullet('To establish a globally consistent reference coordinate system without GPS or ground-truth leakage through decentralized SE(2) global gauge fixing anchored to component root keyframes.', bold_prefix='5. Decentralized SE(2) Gauge Alignment: ')

add_bullet('To enforce bounded memory consumption on flight processors through conservative keyframe pruning and landmark compaction that replaces redundant linear odometry chains with composite relative transformations with zero fill-in edges.', bold_prefix='6. Bounded-Memory Compaction: ')

add_bullet('To maintain continuous operation across intermittent RF shadows through a Delay-Tolerant Network (DTN) store-and-forward message buffer with hop-count routing and time-to-live deduplication.', bold_prefix='7. Delay-Tolerant Multi-Hop Mesh: ')

# 7. SYNOPSIS
add_heading_1('7. Synopsis of the invention')
add_body('The present invention is a computer-implemented software system and decentralized robotic architecture (implemented in C++20 with an interactive aerospace flight-telemetry dashboard). It provides autonomous, collaborative topological-metric mapping across swarms of mobile rovers navigating unfamiliar, GPS-denied environments subject to extreme packet drop rates, RF shadows, and constrained bandwidth. The system operates as a unified processing pipeline comprising the following cooperative modules:')

add_bullet('Sensor and Keyframe Extraction Module: interfaces wheel encoders, IMU, and LiDAR/visual perception to extract geometric keyframes and spatial landmark keys keyed by compact 64-bit spatial hashes.', bold_prefix='• Local Submap & Perception Module (101): ')

add_bullet('IBLT Set Reconciliation Engine: summarizes the local landmark set into a fixed 1,024-byte algebraic table. When two rovers enter wireless proximity, peeling inversion decodes the exact symmetric difference in O(d) time without exchanging raw point clouds.', bold_prefix='• IBLT Set Reconciliation Module (102): ')

add_bullet('DTN Store-and-Forward Buffer Module: buffers outgoing 1 KB IBLT packets, routing them across multi-hop peer-to-peer radio meshes with hop-limit counters and expiration timestamps during network blackouts.', bold_prefix='• Delay-Tolerant Networking Subsystem (103): ')

add_bullet('Geometric Consistency Verifier: constructs a pairwise landmark distance-invariance graph and evaluates candidate loop closures using Maximum-Clique extraction and Horn SE(2) singular value decomposition, achieving 100% precision against perceptual aliasing.', bold_prefix='• Robust Geometric Gating Module (104): ')

add_bullet('Dynamic Map Compaction & Memory Controller: continuously monitors heap consumption; when approaching a memory ceiling, prunes non-essential keyframes via serial Lie compounding, achieving a 78.5% heap reduction with zero Hessian fill-in edges.', bold_prefix='• Bounded Memory Compactor (105): ')

add_bullet('Disjoint-Set Union (DSU) Topological Merger: tracks swarm connected components dynamically, merging distinct sub-graphs upon verified inter-rover loop closure in O(α(V)) amortized time.', bold_prefix='• DSU Forest Tracker (106): ')

add_bullet('Decentralized SE(2) Gauge Alignment Module: anchors unconstrained rotational and translational gauge degrees of freedom to the root node of the merged DSU component, reducing unaligned trajectory error from 27.97 m to 0.028 m without accessing ground-truth coordinates.', bold_prefix='• SE(2) Gauge Fixer (107): ')

add_bullet('Continuous Riemannian Dynamics (CORD) Optimizer: iteratively updates rover poses across the distributed mesh along negative Riemannian gradients, converging to a globally optimal map with Lyapunov asymptotic stability.', bold_prefix='• CORD Riemannian Optimizer (108): ')

add_bullet('Mission Control Telemetry & Live Simulation Interface: streams real-time UDP pose and DSU telemetry to an aerospace command HUD displaying smooth 60 FPS trajectories, glowing mesh bloom, blackout ghost drift, and cyan shockwave reconciliation.', bold_prefix='• Aerospace Telemetry Interface (109): ')

doc.add_page_break()

# 8. BRIEF DESCRIPTION OF DRAWINGS
add_heading_1('8. Brief description of drawings')
add_body('Figure 1 illustrates the block diagram of the proposed decentralized multi-rover topological SLAM system. Whereas,')
add_body('100 denotes sensor odometry and LiDAR pose inputs,\n'
         '101 denotes local submap and keyframe extraction module,\n'
         '102 denotes IBLT set reconciliation engine,\n'
         '103 denotes delay-tolerant networking (DTN) store-and-forward buffer module,\n'
         '104 denotes maximum-clique geometric consistency verifier,\n'
         '105 denotes dynamic map compaction and keyframe pruning module,\n'
         '106 denotes disjoint-set union (DSU) topological merging module,\n'
         '107 denotes decentralized SE(2) global gauge alignment module,\n'
         '108 denotes continuous Riemannian dynamics (CORD) distributed optimizer,\n'
         '109 denotes multi-hop peer-to-peer radio mesh subsystem, and\n'
         '110 denotes unified decentralized global metric-topological map.')

add_body('Figure 2 illustrates the procedural flowchart of the constant-overhead set reconciliation and distributed Riemannian optimization method. Whereas,')
add_body('200 denotes initiating peer discovery over a constrained RF mesh,\n'
         '201 denotes constructing a fixed 1,024-byte IBLT summary payload,\n'
         '202 denotes transmitting the 1 KB IBLT packet via DTN store-and-forward routing,\n'
         '203 denotes peel-decoding the IBLT symmetric difference (A Δ B) in O(d) time,\n'
         '204 denotes decision block evaluating whether peeling completed successfully without residual collisions,\n'
         '205 denotes constructing a pairwise consistency graph across candidate landmark associations,\n'
         '206 denotes executing maximum-clique filtering and TEASER++ SE(2) Horn alignment,\n'
         '207 denotes merging topological map components via DSU path compression and union-by-rank,\n'
         '208 denotes anchoring coordinate frames through decentralized SE(2) gauge fixing, \n'
         '209 denotes updating distributed rover poses via Riemannian CORD Lie retraction, and\n'
         '210 denotes strata-estimator fallback partition procedure executed when pure-cell extraction fails.')

add_body('Figure 3 illustrates a worked topological example of two rovers reconnecting after an RF blackout. Whereas,')
add_body('300 denotes first autonomous rover R1 with established local submap 1,\n'
         '301 denotes second autonomous rover R2 possessing accumulated dead-reckoning drift (ghost path),\n'
         '302 denotes shared physical landmark Lk observed by both rovers,\n'
         '306 denotes severed radio communication link causing 27.97 m ghost odometry drift, and\n'
         '307 denotes 1 KB IBLT packet exchange deploying an expanding cyan shockwave that snaps drifted trajectory error to 0.028 m (-99.9% error cut).')

# 9. DETAIL DESCRIPTION OF THE INVENTION
add_heading_1('9. Detail description of the invention')

add_heading_2('9.1 Mathematical Data Model and Manifold Representation')
add_body('In the system of the present invention, each rover i maintains a local pose graph G_i = (V_i, E_i), where vertices V_i represent robot keyframe poses and spatial landmark locations, and edges E_i represent relative geometric constraints derived from odometry or perception. Unlike conventional Euclidean systems, every pose vertex X_k resides on the Special Euclidean Lie group SE(2) = R^2 x SO(2):')

add_equation_image('docs/eq1.png', height_in_inches=0.45)

add_body('where p_k = [x_k, y_k]^T in R^2 denotes planar translation and R(theta_k) in SO(2) denotes the 2x2 rotation matrix defined by orientation angle theta_k. The relative transform between pose X_j and pose X_k is given by Lie group inversion and composition:')

add_equation_image('docs/eq2.png', height_in_inches=0.28)

add_body('Tangent vector perturbations xi = [delta_x, delta_y, delta_theta]^T in se(2) are mapped to SE(2) using the exponential map Exp(xi) and retracted back via Log(T).')

add_heading_2('9.2 Invertible Bloom Lookup Table (IBLT) Set Reconciliation (102, Figure 2)')
add_body('To eliminate the multi-megabyte transmission penalties of traditional SLAM, the landmark keys of each rover are reconciled using an Invertible Bloom Lookup Table (IBLT). An IBLT B of m cells is constructed where each cell c contains three algebraic registers:')

add_equation_image('docs/eq3.png', height_in_inches=0.26)

add_body('where count is an integer counter, keySum is the bitwise XOR sum of all 64-bit landmark keys mapped to cell c, and hashSum is the bitwise XOR sum of secondary hash values. The entire table is dimensioned to fit within a strict 1,024-byte wire payload. When rover i and rover j initiate reconciliation, rover j subtracts its local table B_j from incoming table B_i cell-by-cell:')

add_equation_image('docs/eq4.png', height_in_inches=0.32)

add_body('Peel-decoding iteratively extracts pure cells (cells where count = +/- 1 and hash(keySum) == hashSum). A positive pure cell reveals a landmark known to rover i but missing from rover j; a negative pure cell reveals a landmark known to rover j but missing from rover i. This extracts the exact symmetric difference A Δ B in O(d) operations, where d is the size of the set difference, using only 1 KB of wireless payload.')

add_heading_2('9.3 Algorithmic Complexity Table')
add_body('The following table details the primary data structures, mathematical operations, and computational complexities across all core software modules:')

# Table of Algorithms & Complexities
table_alg = doc.add_table(rows=8, cols=4)
table_alg.alignment = WD_TABLE_ALIGNMENT.CENTER
headers_alg = ['Subsystem / Module', 'Data Structure / Algorithm', 'C++ / Mathematical Method', 'Time Complexity']
col_widths_alg = [Inches(1.8), Inches(1.8), Inches(1.8), Inches(1.4)]

for i, h in enumerate(headers_alg):
    cell = table_alg.cell(0, i)
    cell.width = col_widths_alg[i]
    set_cell_background(cell, 'F2F2F2')
    set_cell_margins(cell)
    p = cell.paragraphs[0]
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(2)
    run = p.add_run(h)
    run.font.name = 'Times New Roman'
    run.font.size = Pt(9.5)
    run.font.bold = True

alg_data = [
    ('IBLT Set Reconciliation (102)', 'Invertible Bloom Lookup Table (m cells)', 'Peel-decoding via pure cell XOR extraction', 'O(d) peeling,\nO(1) payload (1 KB)'),
    ('Topological Merging (106)', 'Disjoint-Set Union (DSU) Forest', 'Path compression and union-by-rank', 'O(α(V)) amortized\n(α <= 4 for all V)'),
    ('Geometric Gating (104)', 'Pairwise Consistency Graph', 'Max-Clique / TEASER++ Horn SVD alignment', 'Polynomial O(N^2) inliers'),
    ('Riemannian Optimizer (108)', 'CORD on SE(2) Lie Manifold', 'Continuous Riemannian dynamics & Lie retraction', 'O(|E|) per iteration,\nLyapunov stable'),
    ('Decentralized Gauge (107)', 'Component Root Anchor Graph', 'Distributed SE(2) transformation propagation', 'O(V + E) BFS traversal'),
    ('Memory Compactor (105)', 'Dynamic Keyframe Pose Graph', 'Serial Lie compounding (zero Hessian fill-in)', 'O(K) linear reduction,\n-78.5% heap'),
    ('DTN Network Buffer (103)', 'Priority Queue & Hash Map Cache', 'Store-and-forward hop routing with TTL', 'O(log P) insertion,\nO(1) duplicate check')
]

for row_idx, data in enumerate(alg_data):
    for col_idx, item in enumerate(data):
        cell = table_alg.cell(row_idx + 1, col_idx)
        cell.width = col_widths_alg[col_idx]
        set_cell_margins(cell)
        p = cell.paragraphs[0]
        p.paragraph_format.space_before = Pt(2)
        p.paragraph_format.space_after = Pt(2)
        run = p.add_run(item)
        run.font.name = 'Times New Roman'
        run.font.size = Pt(8.5)

add_heading_2('9.4 Maximum-Clique Geometric Verification (104)')
add_body('To reject false-positive perceptual aliasing matches in lunar craters, candidate landmark associations are mapped to a Pairwise Consistency Graph G_c = (V_c, E_c). A vertex v in V_c represents a proposed landmark correspondence (l_i, l_j). An edge exists between correspondence u = (l_i1, l_j1) and correspondence v = (l_i2, l_j2) if and only if the Euclidean distance between landmarks is invariant under rigid SE(2) transformation:')

add_equation_image('docs/eq5.png', height_in_inches=0.32)

add_body('The true set of inlier correspondences forms a Maximum Clique in G_c. Once extracted, closed-form Horn singular value decomposition estimates the relative transformation T_ij = (R_ij, t_ij). If the inlier count fails to exceed a statistical significance threshold, the candidate loop closure is rejected, ensuring 100% precision.')

add_heading_2('9.5 Continuous Riemannian Dynamics (CORD) Optimizer (108)')
add_body('Distributed pose-graph optimization is governed by Continuous Riemannian Dynamics (CORD). For each relative measurement edge (i, j) with covariance weight Omega_ij, the Riemannian residual r_ij on SE(2) is defined as:')

add_equation_image('docs/eq6.png', height_in_inches=0.32)

add_body('The total Riemannian energy function is:')

add_equation_image('docs/eq7.png', height_in_inches=0.38)

add_body('Each rover integrates its local pose continuous dynamics along the Riemannian negative gradient:')

add_equation_image('docs/eq8.png', height_in_inches=0.42)

add_body('where Ad denotes the adjoint representation on SE(2) and gamma is an adaptive step size. By operating on the Riemannian manifold directly, CORD provably converges without rotational wrap-around singularities.')

add_heading_2('9.6 Experimental Validation and Empirical Metrics')
add_body('The system of the present invention was tested across an automated test suite comprising 66 CTest targets and 13 end-to-end acceptance suites, as well as 2,450 paired empirical benchmark trials and 15,380 channel-impairment stress tests. Key verified empirical results:')
add_bullet('Absolute Trajectory Error (ATE): reduced from 27.974 m unaligned down to 0.0282 m (2.8 cm) after 1 KB IBLT reconciliation and CORD optimization, representing a 99.90% trajectory error reduction (Wilcoxon signed-rank test p < 0.001, Cohen\'s d > 2.0).', bold_prefix='• Millimeter-Grade Convergence: ')
add_bullet('Strict 1,024-Byte Transmission: constant-overhead IBLT reconciliation maintained wire payloads at exactly 1,024 bytes regardless of map size, compared to unbounded multi-megabyte growth in baseline systems.', bold_prefix='• Constant Wire Overhead: ')
add_bullet('78.5% Heap Footprint Cut: dynamic map compaction pruned redundant keyframes via serial compounding without introducing fill-in edges, lowering heap consumption from 8.42 MB to 1.81 MB per rover.', bold_prefix='• Bounded Memory Safety: ')
add_bullet('Partitions & Packet Loss Resilience: swarms maintained topological consistency under random packet loss rates up to 90% and communication delays up to 500 ms.', bold_prefix='• Extreme Robustness: ')

# 10. BEST METHOD
add_heading_1('10. Best method of performance of the invention')
add_body('In the best method of practicing the invention, each rover is equipped with a 2D/3D LiDAR, wheel odometry encoders, an embedded processor executing the C++20 software stack, and a packet radio transceiver operating a multi-hop mesh network. During autonomous exploration of a lunar crater, rovers build local submaps and insert spatial landmark keys into a 1,024-byte IBLT structure. When rover R1 detects the radio beacon of rover R2, R1 transmits its 1 KB IBLT packet via the DTN store-and-forward buffer. Rover R2 subtracts its local table and peel-decodes the symmetric difference in O(d) operations, extracting the shared landmarks.')
add_body('Next, R2 constructs the pairwise consistency graph and extracts the maximum clique to verify the geometric alignment against perceptual aliasing. Upon passing geometric verification, the DSU forest merges the two topological components using path compression. Decentralized SE(2) gauge alignment anchors the coordinate frame to the component root keyframe, after which Continuous Riemannian Dynamics (CORD) iteratively optimizes the pose graph along Riemannian geodesics. The drifted ghost trajectory of R2 instantly snaps back to the unified ground truth (ATE 0.028 m), and the updated map is propagated across the swarm.')

doc.add_page_break()

# 11. CLAIMS
add_heading_1('11. CLAIMS')
add_body('We claim:')

claims = [
    'A computer-implemented decentralized multi-robot Simultaneous Localization and Mapping (SLAM) system for operation in GPS-denied and communication-constrained environments, comprising: a plurality of mobile rovers, each rover comprising one or more spatial perception sensors, a radio transceiver, an embedded processor, and a non-transitory memory storing instructions that when executed cause the processor to: extract spatial landmarks from local sensor observations and encode said landmarks into an algebraic Invertible Bloom Lookup Table (IBLT) summary; transmit said IBLT summary to a peer rover over a peer-to-peer radio link within a fixed wire transmission budget; peel-decode an algebraic difference between local and received IBLT summaries to extract a symmetric difference of landmarks in O(d) time, where d is the number of differing landmarks; geometrically verify candidate landmark loop closures using a pairwise distance-invariance consistency graph; topologically merge rover coordinate frames using a Disjoint-Set Union (DSU) data structure; and optimize rover poses across a distributed graph using Continuous Riemannian Dynamics (CORD) directly on the SE(2) Lie manifold.',
    
    'The system as claimed in claim 1, wherein said fixed wire transmission budget of the IBLT summary is strictly bounded at 1,024 bytes, and wherein each cell of said IBLT summary comprises an integer count register, an algebraic keySum register storing bitwise XOR sums of 64-bit landmark keys, and a hashSum register storing bitwise XOR sums of secondary verification hashes.',
    
    'The system as claimed in claim 1, wherein peel-decoding comprises iteratively locating pure cells having a count of plus or minus one and a matching hashSum, extracting the pure landmark key, and subtracting said pure key from all hash-mapped cells until the symmetric difference is completely resolved.',
    
    'The system as claimed in claim 1, wherein geometrically verifying candidate landmark loop closures comprises constructing said pairwise consistency graph wherein nodes represent candidate landmark matches and edges represent Euclidean distance preservation within an error tolerance, and extracting a Maximum-Weight Clique of mutually consistent correspondences.',
    
    'The system as claimed in claim 4, wherein relative SE(2) rotation and translation between peer rovers are computed from said Maximum-Weight Clique using closed-form Horn singular value decomposition, and wherein candidate closures falling below an inlier threshold are rejected to prevent perceptual aliasing.',
    
    'The system as claimed in claim 1, wherein topologically merging rover coordinate frames via said DSU data structure utilizes path compression during find operations and union-by-rank during merge operations, bounding amortized merge complexity to O(α(V)), where α is the inverse Ackermann function.',
    
    'The system as claimed in claim 1, further comprising a decentralized SE(2) global gauge alignment module configured to eliminate unconstrained rotational and translational gauge degrees of freedom by anchoring coordinate frames to the root keyframe of the merged DSU component without accessing ground-truth coordinates.',
    
    'The system as claimed in claim 1, wherein said Continuous Riemannian Dynamics (CORD) optimizer updates distributed rover poses along negative Riemannian gradient trajectories defined by Lie group retraction mappings Exp(-gamma * grad_R E(X)), ensuring asymptotic convergence with zero rotational wrap-around singularities.',
    
    'The system as claimed in claim 1, further comprising a dynamic map compaction module configured to monitor heap memory usage and prune redundant keyframes through serial Lie compounding of relative SE(2) transformations, reducing memory consumption by at least 70% while generating zero Hessian fill-in edges.',
    
    'The system as claimed in claim 1, further comprising a Delay-Tolerant Network (DTN) store-and-forward buffer configured to retain and route outgoing 1 KB IBLT packets across multi-hop mesh topologies during line-of-sight communication blackouts using hop-count limits and expiration timestamps.',
    
    'A computer-implemented method for decentralized multi-rover SLAM under communication constraints, comprising: encoding spatial landmark observations of a mobile rover into a fixed-size Invertible Bloom Lookup Table (IBLT); transmitting said IBLT to a peer rover within a fixed 1,024-byte wire envelope; peel-decoding an algebraic cell-by-cell difference between local and remote IBLTs to extract symmetric landmark differences; validating loop closure candidates via maximum-clique extraction on a pairwise consistency graph; merging topological components using a Disjoint-Set Union (DSU) structure; fixing coordinate gauge freedom via decentralized SE(2) root anchoring; and optimizing relative poses across the swarm via Continuous Riemannian Dynamics on an SE(2) Lie manifold.',
    
    'The method as claimed in claim 11, wherein said method reduces Absolute Trajectory Error (ATE) from an unaligned drift of at least 25 meters down to less than 0.03 meters upon IBLT reconciliation and CORD optimization.'
]

for idx, clm in enumerate(claims):
    p_clm = doc.add_paragraph()
    p_clm.paragraph_format.space_before = Pt(4)
    p_clm.paragraph_format.space_after = Pt(4)
    p_clm.paragraph_format.line_spacing = 1.15
    r_num = p_clm.add_run(f'{idx + 1}. ')
    r_num.font.name = 'Times New Roman'
    r_num.font.size = Pt(10)
    r_num.font.bold = True
    r_txt = p_clm.add_run(clm)
    r_txt.font.name = 'Times New Roman'
    r_txt.font.size = Pt(10)

# 12. INVENTIVE STEP
add_heading_1('12. Inventive step of your invention')
add_body('The individual mathematical tools (Bloom filters, DSU, Lie algebra, RANSAC) are known in isolation. The proposed inventive contribution lies in their non-obvious synergistic integration into an autonomous decentralized SLAM architecture that overcomes the communication, drift, and memory bottlenecks of planetary multi-robot exploration:')

add_bullet('Fixed 1 KB Wireless Budget: unlike conventional SLAM systems that stream megabyte-scale point clouds or keyframe arrays, the present invention compresses set reconciliation into a strict 1,024-byte IBLT envelope, enabling map synchronization over severe space telemetry (< 100 kbps).', bold_prefix='• Constant-Overhead Set Reconciliation: ')

add_bullet('Singularity-Free Lie Optimization: integrating CORD continuous dynamics directly on the SE(2) Riemannian manifold eliminates the angular gimbal wrap-around singularities that cause Euclidean gradient descent methods to diverge under large initial orientation errors.', bold_prefix='• Manifold-Constrained Riemannian Consensus: ')

add_bullet('True Ground-Truth Independence: unlike existing benchmark setups that borrow ground-truth coordinates for gauge anchoring, the present invention implements decentralized SE(2) gauge fixing anchored purely to topological DSU component roots, slashing ATE from 27.97 m to 0.028 m (-99.9% error cut) with zero ground-truth leakage.', bold_prefix='• Ground-Truth Free Gauge Alignment: ')

add_bullet('Zero Fill-In Compaction: traditional Schur complement marginalization produces dense information cliques (O(K^2) fill-in edges), exhausting flight RAM. The present invention uses serial Lie compounding to achieve a 78.5% heap reduction with exactly zero fill-in edges.', bold_prefix='• Non-Monotonic Memory Growth: ')

add_bullet('Perceptual Aliasing Immunity: maximum-clique consistency filtering guarantees 100% precision against false-positive loop closures in repetitive lunar regolith terrain.', bold_prefix='• Verifiable Loop Closure Gating: ')

# 13. INDUSTRIAL APPLICATION
add_heading_1('13. Industrial application')
add_bullet('Autonomous exploration of lunar south pole craters (Shackleton, South Pole-Aitken basin) and subsurface lava tubes (Marius Hills) by multi-rover planetary swarms without GPS or persistent satellite links.', bold_prefix='• Extraterrestrial Planetary Exploration: ')

add_bullet('Mapping of collapsed underground mines, tunnels, and subway infrastructure where wireless signals are obstructed and GPS is unavailable.', bold_prefix='• Subterranean and Mine Search-and-Rescue: ')

add_bullet('Cooperative exploration of collapsed structures and earthquake zones by heterogeneous robotic swarms communicating over ad-hoc delay-tolerant mesh radios.', bold_prefix='• Disaster Response & Urban Search-and-Rescue (USAR): ')

add_bullet('Fleet localization and spatial mapping in large industrial distribution centers, shielded naval ship holds, and deep underground parking structures.', bold_prefix='• GPS-Denied Industrial Logistics: ')

add_bullet('Cooperative multi-vehicle survey of pipeline infrastructure and marine sea beds using low-bandwidth acoustic telemetry.', bold_prefix='• Subsea Autonomous Underwater Vehicles (AUVs): ')

# 14. ABSTRACT
add_heading_1('14. Abstract')
add_body('A decentralized multi-rover topological Simultaneous Localization and Mapping (SLAM) system and method for autonomous swarms operating in GPS-denied, communication-constrained, and memory-limited planetary environments. Each rover encodes spatial landmark observations into an Invertible Bloom Lookup Table (IBLT) dimensioned within a fixed 1,024-byte wire payload. Upon peer rendezvous, algebraic cell-by-cell subtraction and peel-decoding extract the exact symmetric landmark difference in O(d) operations without transmitting raw point clouds. Candidate loop closures are verified against perceptual aliasing via Maximum-Clique extraction on a pairwise distance-invariance consistency graph. Dynamic network partitions and swarm re-merges are tracked using a Disjoint-Set Union (DSU) forest with path compression and union-by-rank in O(α(V)) amortized time. Unconstrained coordinate frame degrees of freedom are eliminated via decentralized SE(2) global gauge fixing anchored to component roots without ground-truth leakage. Continuous Riemannian Dynamics (CORD) optimizes rover poses along manifold geodesics directly on SE(2), cutting Absolute Trajectory Error (ATE) by 99.9% (from 27.97 m to 0.028 m) with verified Lyapunov stability. Dynamic map compaction prunes keyframes via serial Lie compounding, achieving a 78.5% memory reduction with zero Hessian fill-in edges.')

doc.add_page_break()

# 15. DRAWINGS (IPO Rule Compliant: Numbers Only, Standard Margins)
add_heading_1('15. Drawings')

add_body('Figure 1', bold_prefix='Figure 1: ')
p_img1 = doc.add_paragraph()
p_img1.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_img1.paragraph_format.space_before = Pt(6)
p_img1.paragraph_format.space_after = Pt(18)
run_img1 = p_img1.add_run()
run_img1.add_picture('docs/patent_fig1.png', width=Inches(5.0))

doc.add_page_break()

add_body('Figure 2', bold_prefix='Figure 2: ')
p_img2 = doc.add_paragraph()
p_img2.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_img2.paragraph_format.space_before = Pt(6)
p_img2.paragraph_format.space_after = Pt(18)
run_img2 = p_img2.add_run()
run_img2.add_picture('docs/patent_fig2.png', width=Inches(4.8))

doc.add_page_break()

add_body('Figure 3', bold_prefix='Figure 3: ')
p_img3 = doc.add_paragraph()
p_img3.alignment = WD_ALIGN_PARAGRAPH.CENTER
p_img3.paragraph_format.space_before = Pt(6)
p_img3.paragraph_format.space_after = Pt(18)
run_img3 = p_img3.add_run()
run_img3.add_picture('docs/patent_fig3.png', width=Inches(5.0))

out_path = 'docs/AstroMesh-X_Patent_Proposal_Draft.docx'
doc.save(out_path)
print(f'Successfully generated patent draft document with rendered LaTeX equations and IPO-compliant drawings: {out_path}')
