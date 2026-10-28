import os
import subprocess
import markdown
import sys

def generate_onepager_pdf():
    md_file = "Creative_Rep_OnePager.md"
    html_file = "Creative_Rep_OnePager.html"
    pdf_file = "Creative_Rep_OnePager.pdf"

    with open(md_file, 'r', encoding='utf-8') as f:
        md_text = f.read()

    html_body = markdown.markdown(md_text, extensions=['extra', 'tables'])

    # Ultra-compact, stylish CSS to ensure it fits on one page for the designer
    html_content = f"""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <style>
            body {{
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 11.5px;
                line-height: 1.35;
                color: #111;
                margin: 0;
                padding: 15px;
            }}
            h1 {{
                font-size: 18px;
                text-align: center;
                text-transform: uppercase;
                background-color: #2c3e50;
                color: white;
                padding: 8px;
                margin-top: 0;
                border-radius: 4px;
            }}
            h3 {{
                font-size: 14px;
                color: #d35400;
                border-bottom: 1.5px solid #2980b9;
                margin-top: 10px;
                margin-bottom: 5px;
                padding-bottom: 2px;
                text-transform: uppercase;
            }}
            ul {{
                margin-top: 3px;
                margin-bottom: 8px;
                padding-left: 20px;
            }}
            li {{
                margin-bottom: 2px;
            }}
            strong {{
                color: #2980b9;
            }}
            code {{
                background-color: #f1c40f;
                color: #000;
                padding: 1px 4px;
                border-radius: 2px;
                font-weight: bold;
            }}
            em {{
                color: #555;
            }}
            @page {{
                margin: 10mm;
                size: A4;
            }}
        </style>
    </head>
    <body>
        {html_body}
    </body>
    </html>
    """

    with open(html_file, 'w', encoding='utf-8') as f:
        f.write(html_content)

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
    
    subprocess.run(cmd, check=True)

if __name__ == "__main__":
    generate_onepager_pdf()
