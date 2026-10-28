import os
import subprocess
import markdown
import sys

# List of files to aggregate
files_in_order = [
    "PGK_Phase1_System_Architecture.md",
    "PGK_Phase2_Mechanical_Design.md",
    "PGK_Phase3_Fuze_SA_System.md",
    "PGK_Phase4_GNC_Architecture.md",
    "PGK_Phase5_6DOF_MonteCarlo.md",
    "PGK_Phase6_Master_Deliverable.md",
    "PGK_Phase7_SIH_Blueprint.md"
]

def generate_report():
    print("Reading project files...")
    combined_md = "# YIL Precision Guidance Kit (PGK-155)\n\n## Master Project Report & Technical Blueprint\n\n---\n\n"
    
    for f in files_in_order:
        if os.path.exists(f):
            with open(f, 'r', encoding='utf-8') as file:
                combined_md += file.read() + "\n\n<div style='page-break-after: always;'></div>\n\n"
        else:
            print(f"Warning: {f} not found.")

    print("Converting Markdown to HTML...")
    html_body = markdown.markdown(combined_md, extensions=['extra', 'toc', 'tables'])
    
    html_content = f"""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <title>PGK-155 Master Project Report</title>
        <style>
            body {{
                font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                line-height: 1.6;
                color: #333;
                max-width: 900px;
                margin: 0 auto;
                padding: 40px;
            }}
            h1, h2, h3, h4 {{
                color: #2c3e50;
                margin-top: 24px;
            }}
            h1 {{
                border-bottom: 2px solid #3498db;
                padding-bottom: 10px;
                text-align: center;
                font-size: 2.5em;
            }}
            h2 {{
                border-bottom: 1px solid #bdc3c7;
                padding-bottom: 5px;
                color: #2980b9;
            }}
            pre {{
                background-color: #f4f4f4;
                padding: 15px;
                border-radius: 5px;
                overflow-x: auto;
                font-family: 'Consolas', monospace;
                font-size: 0.9em;
                border-left: 4px solid #34495e;
            }}
            code {{
                background-color: #f4f4f4;
                padding: 2px 4px;
                border-radius: 3px;
                font-family: 'Consolas', monospace;
            }}
            table {{
                border-collapse: collapse;
                width: 100%;
                margin: 20px 0;
            }}
            th, td {{
                border: 1px solid #ddd;
                padding: 12px;
                text-align: left;
            }}
            th {{
                background-color: #2c3e50;
                color: white;
            }}
            tr:nth-child(even) {{
                background-color: #f2f2f2;
            }}
            blockquote {{
                border-left: 4px solid #3498db;
                margin: 0;
                padding-left: 15px;
                color: #555;
                font-style: italic;
            }}
            @media print {{
                body {{
                    padding: 0;
                }}
                @page {{
                    margin: 20mm;
                }}
            }}
        </style>
    </head>
    <body>
        {html_body}
    </body>
    </html>
    """

    html_file = "PGK_Master_Report.html"
    pdf_file = "PGK_Master_Report.pdf"

    print(f"Writing HTML to {html_file}...")
    with open(html_file, 'w', encoding='utf-8') as file:
        file.write(html_content)

    print("Generating PDF using MS Edge headless mode...")
    edge_path = r"C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe"
    
    current_dir = os.path.abspath(os.path.dirname(__file__))
    html_abs = os.path.join(current_dir, html_file)
    pdf_abs = os.path.join(current_dir, pdf_file)

    cmd = [
        edge_path,
        "--headless",
        "--disable-gpu",
        "--no-pdf-header-footer",
        f"--print-to-pdf={pdf_abs}",
        f"file:///{html_abs.replace(chr(92), '/')}"
    ]
    
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        print(f"Successfully generated PDF: {pdf_file}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to generate PDF. Error: {e.stderr.decode()}")
        print(f"Command run: {' '.join(cmd)}")

if __name__ == "__main__":
    generate_report()
