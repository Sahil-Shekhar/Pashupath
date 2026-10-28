from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.dml.color import RGBColor
import os

def create_official_sih_ppt():
    prs = Presentation()
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)

    # SIH Brand Colors
    sih_blue = RGBColor(22, 101, 172)
    sih_orange = RGBColor(242, 101, 34)
    sih_green = RGBColor(13, 148, 71)
    dark_gray = RGBColor(50, 50, 50)
    white = RGBColor(255, 255, 255)
    purple_circle = RGBColor(112, 48, 160)
    light_bg = RGBColor(240, 240, 240)

    def add_footer(slide, slide_num):
        # Blue footer bar
        footer = slide.shapes.add_shape(1, Inches(0), Inches(7.0), Inches(13.333), Inches(0.5))
        footer.fill.solid()
        footer.fill.fore_color.rgb = sih_blue
        footer.line.fill.background()
        
        # Footer text
        tf = footer.text_frame
        p = tf.paragraphs[0]
        p.text = "@SIH Idea submission- Template"
        p.alignment = PP_ALIGN.CENTER
        p.font.size = Pt(14)
        p.font.color.rgb = white
        
        # Slide number
        num_box = slide.shapes.add_textbox(Inches(12.5), Inches(7.0), Inches(0.8), Inches(0.5))
        np = num_box.text_frame.paragraphs[0]
        np.text = str(slide_num)
        np.alignment = PP_ALIGN.RIGHT
        np.font.size = Pt(14)
        np.font.color.rgb = white
        np.font.bold = True

    def apply_base_template(slide, slide_num, title_text):
        add_footer(slide, slide_num)
        
        # Top Left Team Circle
        circle = slide.shapes.add_shape(9, Inches(0.3), Inches(0.2), Inches(1.2), Inches(1.0)) # 9 is ellipse
        circle.fill.background()
        circle.line.color.rgb = purple_circle
        circle.line.width = Pt(2)
        tf = circle.text_frame
        tf.word_wrap = True
        p = tf.paragraphs[0]
        p.text = "Team\nAstra"
        p.alignment = PP_ALIGN.CENTER
        p.font.size = Pt(14)
        p.font.color.rgb = black_color
        
        # Main Title
        title_box = slide.shapes.add_textbox(Inches(2.0), Inches(0.2), Inches(9.333), Inches(0.8))
        tp = title_box.text_frame.paragraphs[0]
        tp.text = title_text
        tp.alignment = PP_ALIGN.CENTER
        tp.font.size = Pt(36)
        tp.font.bold = True
        tp.font.color.rgb = black_color

    def add_textbox(slide, left, top, width, height, text, font_size=16, bold=False, color=dark_gray, align=PP_ALIGN.LEFT):
        tx = slide.shapes.add_textbox(left, top, width, height)
        tf = tx.text_frame
        tf.word_wrap = True
        p = tf.paragraphs[0]
        p.text = text
        p.font.size = Pt(font_size)
        p.font.bold = bold
        p.font.color.rgb = color
        p.alignment = align
        return tx

    def add_rect(slide, left, top, width, height, bg_color, text="", font_size=14, text_color=white, bold=False):
        rect = slide.shapes.add_shape(1, left, top, width, height)
        rect.fill.solid()
        rect.fill.fore_color.rgb = bg_color
        rect.line.fill.background()
        if text:
            tf = rect.text_frame
            tf.word_wrap = True
            p = tf.paragraphs[0]
            p.text = text
            p.font.size = Pt(font_size)
            p.font.color.rgb = text_color
            p.font.bold = bold
            p.alignment = PP_ALIGN.LEFT
        return rect

    black_color = RGBColor(0,0,0)

    # ================= SLIDE 1 =================
    slide1 = prs.slides.add_slide(prs.slide_layouts[6])
    
    # Top Title
    add_textbox(slide1, Inches(0.5), Inches(0.3), Inches(10), Inches(1), "SMART INDIA HACKATHON 2026", 40, True, sih_blue)
    add_textbox(slide1, Inches(0.5), Inches(1.2), Inches(10), Inches(1), "TITLE PAGE", 32, True, black_color, PP_ALIGN.CENTER)

    # Bullets
    left_m = Inches(0.8)
    top_m = 2.2
    spacing = 0.6
    
    bullets = [
        "• Problem Statement ID – SIH1453",
        "• Problem Statement Title- Design & Development of a Precision Guidance Kit (PGK) for 155 mm Artillery",
        "• Theme- Defense & Security / Advanced Manufacturing",
        "• PS Category- Hardware",
        "• Team ID- 123456",
        "• Team Name (Registered on portal)- Team Astra"
    ]
    
    for i, b in enumerate(bullets):
        add_textbox(slide1, left_m, Inches(top_m + (i*spacing)), Inches(10), Inches(0.5), b, 20, True, black_color)

    # ================= SLIDE 2 =================
    slide2 = prs.slides.add_slide(prs.slide_layouts[6])
    apply_base_template(slide2, 2, "ASTRA-PGK 155")
    
    add_textbox(slide2, Inches(0.5), Inches(1.2), Inches(12), Inches(0.5), "❖Proposed Solution (Describe your Idea/Solution/Prototype)", 24, True, sih_blue)

    # Creative Blocks
    add_rect(slide2, Inches(0.5), Inches(2.0), Inches(4.0), Inches(4.5), sih_orange, 
             "Detailed Explanation\n\n• Drop-in modular Precision Guidance Kit (PGK).\n• Multi-Mode Fuze: Proximity/Height-of-Burst (HOB), Point Detonating (PD), Delay, & Electronic Time.\n• Contains 4× NACA-0010 Canards.\n• Uses 24GHz FMCW Radar & NavIC GPS.\n• Employs 16-State Extended Kalman Filter.", 16, white, True)

    add_rect(slide2, Inches(4.7), Inches(2.0), Inches(4.0), Inches(4.5), sih_blue, 
             "How it addresses the problem\n\n• Fixes unguided dispersion: Reduces CEP from 265m down to 10.2m.\n• Limits Collateral Damage: High precision saves civilian infrastructure.\n• Ammunition Logistics: Cuts required rounds per target from 50 down to 1-2.", 18, white, True)

    add_rect(slide2, Inches(8.9), Inches(2.0), Inches(4.0), Inches(4.5), sih_green, 
             "Innovation & Uniqueness\n\n• Hardened Mechatronics: Survives >15,000 g axial shock via micro-potting.\n• MIL-STD-1316 Compliant: Dual-environment arming (Setback + Spin).\n• SWaP-C Optimized: Achieves $100k Excalibur performance for ~₹2.13 Lakh.", 18, white, True)

    # ================= SLIDE 3 =================
    slide3 = prs.slides.add_slide(prs.slide_layouts[6])
    apply_base_template(slide3, 3, "TECHNICAL APPROACH")
    
    add_textbox(slide3, Inches(0.5), Inches(1.2), Inches(12), Inches(0.5), "• Technologies to be used (Hardware & Software)", 20, True, black_color)
    
    add_rect(slide3, Inches(0.5), Inches(1.8), Inches(6.0), Inches(1.5), light_bg, 
             "Hardware Stack:\n• Compute: STM32H743 MCU + Spartan-7 FPGA.\n• Fuze Hardware: 24GHz FMCW Radar (Proximity), Piezo (Point Detonating).\n• Sensors: ADIS16460 IMU, NavIC GPS, MS5611 Barometer.\n• Actuators: 50:1 Harmonic Drives, Maxon BLDCs.", 14, black_color)
    add_rect(slide3, Inches(6.8), Inches(1.8), Inches(6.0), Inches(1.5), light_bg, 
             "Software & Frameworks:\n• RTOS Firmware: Rate-monotonic 100Hz control loop.\n• Guidance Law: 2D Ballistic Impact-Point Prediction (BIPP).\n• Filter: 16-State Loosely Coupled EKF.", 16, black_color)

    add_textbox(slide3, Inches(0.5), Inches(3.5), Inches(12), Inches(0.5), "• Methodology and process for implementation (Flow Chart)", 20, True, black_color)
    
    # Flow Chart nodes
    nodes = [
        ("1. Muzzle Exit\n(15k g, 827m/s)", Inches(0.5), sih_orange),
        ("2. Arming\n(MIL-STD-1316)", Inches(3.0), sih_blue),
        ("3. Deploy\n(Canards Unlatch)", Inches(5.5), sih_green),
        ("4. Midcourse\n(GPS/EKF Active)", Inches(8.0), sih_blue),
        ("5. Terminal\n(Multi-Mode Detonate)", Inches(10.5), sih_orange)
    ]
    for text, left, color in nodes:
        add_rect(slide3, left, Inches(4.2), Inches(2.2), Inches(1.5), color, text, 16, white, True)
        # Add arrows except for last
        if left != Inches(10.5):
            add_textbox(slide3, left + Inches(2.2), Inches(4.6), Inches(0.8), Inches(0.5), "➔", 36, True, black_color, PP_ALIGN.CENTER)

    # ================= SLIDE 4 =================
    slide4 = prs.slides.add_slide(prs.slide_layouts[6])
    apply_base_template(slide4, 4, "FEASIBILITY AND VIABILITY")

    add_textbox(slide4, Inches(0.5), Inches(1.2), Inches(6), Inches(0.5), "• Analysis of Feasibility", 20, True, black_color)
    add_rect(slide4, Inches(0.5), Inches(1.8), Inches(6.0), Inches(1.5), light_bg, 
             "• Built on proven, scalable components (MEMS IMU, standard BLDCs).\n• 6-DOF Monte Carlo simulation physically verifies aerodynamic limits and 10.2m CEP.\n• Drops directly into existing NATO 2-inch threaded shells.", 16, black_color)

    add_textbox(slide4, Inches(0.5), Inches(3.5), Inches(12), Inches(0.5), "• Potential Challenges & Strategies (FMEA)", 20, True, black_color)
    
    # FMEA Table
    add_rect(slide4, Inches(0.5), Inches(4.1), Inches(3.8), Inches(2.2), sih_orange, "Risk: High-g Delamination\n\nStrategy:\nSub-assembly encapsulation via Polyurethane potting & 3× M3 Ti shear pins. MIL-STD-810H qualified.", 16, white, True)
    add_rect(slide4, Inches(4.7), Inches(4.1), Inches(3.8), Inches(2.2), sih_blue, "Risk: EW / GNSS Jamming\n\nStrategy:\nLoose-coupled EKF reverts to ADIS16460 dead-reckoning fallback with baseband outlier rejection.", 16, white, True)
    add_rect(slide4, Inches(8.9), Inches(4.1), Inches(3.8), Inches(2.2), sih_green, "Risk: Premature Detonation\n\nStrategy:\nElectronic In-line S&A limits. Dual-environment unlock (Axial >10k g + Spin >150 Hz).", 16, white, True)

    # ================= SLIDE 5 =================
    slide5 = prs.slides.add_slide(prs.slide_layouts[6])
    apply_base_template(slide5, 5, "IMPACT AND BENEFITS")
    
    add_textbox(slide5, Inches(0.5), Inches(1.2), Inches(12), Inches(0.5), "• Potential impact & Benefits of the solution", 20, True, black_color)

    # 4 Columns of impact
    add_rect(slide5, Inches(0.5), Inches(2.0), Inches(2.9), Inches(4.5), sih_blue, 
             "OPERATIONAL IMPACT\n\n• Dispersion Annihilation: Reduces CEP from 265m to 10.2m.\n• Logistics: Cuts ammunition transport burden exponentially (1-2 rounds per kill vs 50).", 16, white, True)
    
    add_rect(slide5, Inches(3.6), Inches(2.0), Inches(2.9), Inches(4.5), sih_green, 
             "ECONOMIC BENEFITS\n\n• Unit Cost: ~₹2.13 Lakh (at scale) compared to ~$100,000 for Excalibur.\n• Extends barrel life by requiring fewer shots per target.", 16, white, True)

    add_rect(slide5, Inches(6.7), Inches(2.0), Inches(2.9), Inches(4.5), sih_orange, 
             "SAFETY / SOCIAL\n\n• Collateral Damage: Pinpoint accuracy allows use in dense/contested terrain safely.\n• MIL-STD-1316 safety architecture protects operators.", 16, white, True)

    add_rect(slide5, Inches(9.8), Inches(2.0), Inches(2.9), Inches(4.5), dark_gray, 
             "STRATEGIC\n\n• Atmanirbhar Bharat: 100% domestic IP, machined from local Ti-6Al-4V.\n• Zero vendor lock-in. Massive export potential.", 16, white, True)

    # ================= SLIDE 6 =================
    slide6 = prs.slides.add_slide(prs.slide_layouts[6])
    apply_base_template(slide6, 6, "RESEARCH AND REFERENCES")

    add_textbox(slide6, Inches(0.5), Inches(1.2), Inches(12), Inches(0.5), "• Details / Links of the reference and research work", 20, True, black_color)

    add_rect(slide6, Inches(0.5), Inches(2.0), Inches(6.0), Inches(4.5), light_bg, 
             "Academic & Ballistic Research:\n\n• McCoy, 'Modern Exterior Ballistics': Theoretical foundation for our 6-DOF Modified Point Mass trajectory engine.\n\n• Magnus Drift Analysis: Integration of cross-range drift calculations (F_magnus ∝ V × ω) yielding ~35m unguided drift.\n\n• Validated via 1,000-run Python/NumPy Monte Carlo simulations.", 16, black_color)

    add_rect(slide6, Inches(6.8), Inches(2.0), Inches(6.0), Inches(4.5), light_bg, 
             "Defense Standardization Links:\n\n• MIL-STD-333B: Fuze contours and standard 2-inch threading specifications.\n\n• MIL-STD-1316: Design criteria for fuze safety and Electronic In-line Safety & Arming (EISA).\n\n• STANAG 4369: Inductive fuze setting protocols.", 16, black_color)

    prs.save("SIH2026_Official_PGK_Presentation_v3.pptx")
    print("Successfully generated SIH2026_Official_PGK_Presentation_v3.pptx")

if __name__ == "__main__":
    create_official_sih_ppt()
